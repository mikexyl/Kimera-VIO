#pragma once

#include <cuda_runtime.h>
#include <xfeat-cpp/faiss_database.h>
#include <xfeat-cpp/place_recognition/jist_onnx.h>
#include <xfeat-cpp/xfeat_cv.h>

#include "kimera-vio/loopclosure/FrameCache.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"

namespace VIO {

// JIST ONNX wrapper for sequence-based visual place recognition
struct JistONNXWrapper : xfeat::JistONNX {
  using Base = xfeat::JistONNX;
  using GlobalDesc = cv::Mat;
  using Desc = cv::Mat;
  using DescVector = std::vector<cv::Mat>;
  using DescMat = cv::Mat;
  using Database = xfeat::FaissDatabase;

  template <typename... Args>
  JistONNXWrapper(std::unique_ptr<Database> faiss_db, Args&&... args)
      : Base(std::forward<Args>(args)...), db_(std::move(faiss_db)) {}

  // Transform function now takes all cached frames and computes descriptor for
  // target frame by using a sequence of seq_length frames ending at
  // target_frame_id
  void transform(std::vector<LCDFrame::Ptr> frames, GlobalDesc& global_desc) {
    const int seq_length = Base::get_seq_length();

    // Collect sequence of frames for inference
    std::vector<cv::Mat> image_sequence;
    image_sequence.reserve(seq_length);

    CHECK_EQ(seq_length, frames.size());

    for (const auto& frame : frames) {
      image_sequence.push_back(frame->image_);
    }

    // Run JIST inference
    global_desc = Base::infer(image_sequence);
  }

  void add(const GlobalDesc& global_desc) {
    CHECK_NOTNULL(db_);
    CHECK(not global_desc.empty());
    faiss::idx_t id = id_to_desc_map_.size();
    id_to_desc_map_.emplace(id, global_desc.clone());
    try {
      db_->add(global_desc);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to add to database: " << e.what();
      throw;
    }
  }

  template <typename... Args>
  void search(Args&&... args) {
    CHECK_NOTNULL(db_);
    try {
      db_->search(std::forward<Args>(args)...);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to search in database: " << e.what();
      throw;
    }
  }

  template <typename... Args>
  auto sim(Args&&... args) {
    try {
      return db_->cosine_similarity(std::forward<Args>(args)...);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to compute similarity in database: " << e.what();
      throw;
    }
  }

  GlobalDesc get(const faiss::idx_t id) const {
    if (id_to_desc_map_.count(id)) {
      return id_to_desc_map_.at(id);
    } else {
      return GlobalDesc();  // Return an empty cv::Mat if id not found
    }
  }

 private:
  std::unique_ptr<Database> db_;
  std::map<faiss::idx_t, cv::Mat> id_to_desc_map_;
};

// dummy feature detector that does nothing
class DummyFeatureDetector : cv::FeatureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(DummyFeatureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DummyFeatureDetector);

  DummyFeatureDetector() = default;

  CV_WRAP void compute(cv::InputArray image,
                       CV_OUT CV_IN_OUT std::vector<cv::KeyPoint>& keypoints,
                       cv::OutputArray descriptors) final {}

  CV_WRAP void compute(cv::InputArrayOfArrays images,
                       CV_OUT CV_IN_OUT
                           std::vector<std::vector<cv::KeyPoint> >& keypoints,
                       cv::OutputArrayOfArrays descriptors) final {}
};

class VLADLoopClosureDetector
    : public LoopClosureDetector<JistONNXWrapper,
                                 DummyFeatureDetector,
                                 xfeat::LighterGlueCV> {
 public:
  KIMERA_POINTER_TYPEDEFS(VLADLoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VLADLoopClosureDetector);

  using Database = JistONNXWrapper;

  static constexpr bool kVLADLCDUseGPU = true;

  template <typename... Args>
  VLADLoopClosureDetector(Ort::Env& env, Args&&... args)
      : LoopClosureDetector(std::forward<Args>(args)...) {
    CHECK(!lcd_params_.lcd_lg_model_path_.empty())
        << "VLADLoopClosureDetector: lcd_lg_model_path_ must be set!";
    CHECK(!lcd_params_.jist_model_path_.empty())
        << "VLADLoopClosureDetector: jist_model_path_ must be set!";

    // Sparse stereo reconstruction members (only if stereo_camera is provided)
    if (stereo_camera_) {
      VLOG(5) << "LoopClosureDetector initializing in stereo mode.";
      auto lcd_stereo_params = stereo_matching_params_;
      // In LCD we set min_dist and max_dist to not discard points
      // TODO: Find better solution instead of hardcoding
      static const bool kVLADLCDDisableStereoMatchDepthCheck = false;
      if (kVLADLCDDisableStereoMatchDepthCheck) {
        lcd_stereo_params.min_point_dist_ = 0.01;
        lcd_stereo_params.max_point_dist_ = 100.0;
      }
      stereo_matcher_ =
          std::make_unique<StereoMatcher>(stereo_camera_, lcd_stereo_params);
    } else {
      VLOG(5) << "LoopClosureDetector initializing in mono mode.";
    }

    // should not need to run feature detection again, so the detector should be
    // empty
    feature_detector_.reset(new DummyFeatureDetector());

    feature_matcher_ = xfeat::LighterGlueCV::create(
        env,
        xfeat::LighterGlueCV::Params{
            .model_path = lcd_params_.lcd_lg_model_path_,
            .use_gpu = true,
            .min_score = -1,
            .n_kpts = lcd_params_.lcd_lg_num_features_,
        });

    size_t free_before, total;
    cudaMemGetInfo(&free_before, &total);

    auto faiss_mode = Database::Database::IndexMode::kIVFFlat;
    int faiss_dim = 0;
    if (lcd_params_.lcd_faiss_index_path_.empty()) {
      faiss_mode = Database::Database::IndexMode::kFlat;
      faiss_dim = 512;
    }

    auto faiss_db = std::make_unique<Database::Database>(
        faiss_mode, lcd_params_.lcd_faiss_index_path_, false, faiss_dim);

    size_t free_after, total_after;
    cudaMemGetInfo(&free_after, &total_after);
    LOG(INFO) << "GPU memory usage for loading FAISS index: "
              << (free_before - free_after) / (1024.0 * 1024.0) << " MB";

    // Initialize JIST ONNX model
    xfeat::JistONNX::Params jist_params;
    jist_params.model_path = lcd_params_.jist_model_path_;
    jist_params.use_gpu = kVLADLCDUseGPU;
    jist_params.seq_length = lcd_params_.jist_seq_length_;
    jist_params.img_height = lcd_params_.network_input_height_;
    jist_params.img_width = lcd_params_.network_input_width_;
    jist_params.descriptor_dim = lcd_params_.jist_descriptor_dim_;
    jist_params.normalize_output = true;

    db_ = std::make_unique<Database>(std::move(faiss_db), env, jist_params);
  }

  /* ------------------------------------------------------------------------
   */
  virtual ~VLADLoopClosureDetector() override = default;

  double computeSequenceScore(const FrameId anchor_frame_id) override;

  std::optional<FrameId> getCurrentAnchorFrameId() override {
    if (new_seq_frames_.empty()) {
      return std::optional<FrameId>();
    } else {
      return std::optional<FrameId>(new_seq_frames_.begin()->get()->id_);
    }
  }

  void computeSequenceGlobalDesc(const FrameId target_frame_id,
                                 bool add_to_sequence) override;

  void detectLoop(const FrameId& frame_id,
                  LoopResult* result,
                  FrameId* query_frame = nullptr,
                  FrameIdSet* global_candidates = nullptr) override;

  void detectLoopOutsideLocalWindow(const FrameId& frame_id,
                                    LoopResult* result,
                                    FrameId* query_frame = nullptr,
                                    FrameIdSet* global_candidates = nullptr);

  std::optional<FrameId> findFirstFrameIdOutsideLocalWindow(
      const FrameId& frame_id) const {
    if (frame_id < static_cast<FrameId>(lcd_params_.local_window_size_)) {
      return std::nullopt;  // No frames outside the local window.
    } else {
      return frame_id - lcd_params_.local_window_size_;
      // Return the first frame ID outside the local window.
    }
  }

  void getNewFeaturesAndDescriptors(
      const cv::Mat& img,
      std::vector<cv::KeyPoint>* keypoints,
      typename Database::Desc* descriptors_mat) override {
    throw std::runtime_error(
        "VLADLoopClosureDetector: getNewFeaturesAndDescriptors is deleted for "
        "VLAD LCD.");
  }

  void getNewFeaturesAndDescriptors(
      const Frame& frame,
      std::vector<cv::KeyPoint>* keypoints,
      typename Database::Desc* descriptors_mat) override;

  void descriptorMatToVec(
      const typename Database::DescMat& descriptors_mat,
      typename Database::DescVector* descriptors_vec) override {
    throw std::runtime_error(
        "VLADLoopClosureDetector: descriptorMatToVec is deleted for VLAD "
        "LCD.");
  }

  void descriptorMatToVec(
      const Frame& frame,
      const typename Database::DescMat& descriptors_mat,
      typename Database::DescVector* descriptors_vec) override;

  // TODO(mikexyl): try remove this
  std::map<int, double> globalDescToMap(
      const typename Database::GlobalDesc& global_desc) override {
    std::map<int, double> desc_map;
    CHECK_EQ(global_desc.rows, 1);
    for (int i = 0; i < global_desc.cols; ++i) {
      desc_map[i] = global_desc.at<float>(0, i);
    }
    return desc_map;
  }

  // using xfeat nv, lcd frame's descriptors_vec are 2 mats: M1 and x_prep
  // and lcd frames' descriptors_mat are the actual xfeat descriptors of each
  // keypoints
  void computeDescriptorMatches(const typename Database::Desc& ref_descriptors,
                                const typename Database::Desc& cur_descriptors,
                                KeypointMatches* matches_match_query,
                                bool cut_matches = false) const override {
    throw std::runtime_error(
        "VLADLoopClosureDetector: computeDescriptorMatches is deleted for VLAD "
        "LCD.");
  }

  void computeDescriptorMatches(const LCDFrame& ref,
                                const LCDFrame& curr,
                                KeypointMatches* matches_match_query,
                                bool cut_matches = false) const override {
    // the keypoint descriptors from frontend frame should be used, so the
    // ref/cur descriptors are empty, and this function is replace with the
    // function following
    CHECK_NOTNULL(matches_match_query);
    CHECK_NOTNULL(feature_matcher_);

    matches_match_query->clear();
    std::vector<cv::DMatch> matches;

    // Use configured network input size for matching (width, height)
    cv::Size image_size0(static_cast<int>(lcd_params_.network_input_width_),
                         static_cast<int>(lcd_params_.network_input_height_));

    cv::Mat ref_kp_mat(ref.keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < ref.keypoints_.size(); ++i) {
      ref_kp_mat.at<float>(i, 0) = ref.keypoints_[i].pt.x;
      ref_kp_mat.at<float>(i, 1) = ref.keypoints_[i].pt.y;
    }

    cv::Mat cur_kp_mat(curr.keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < curr.keypoints_.size(); ++i) {
      cur_kp_mat.at<float>(i, 0) = curr.keypoints_[i].pt.x;
      cur_kp_mat.at<float>(i, 1) = curr.keypoints_[i].pt.y;
    }

    xfeat::DetectionResult ref_ret{
        .keypoints = ref_kp_mat,
        .scores = {},
        .descriptors = ref.descriptors_mat_,
    },
        cur_ret{
            .keypoints = cur_kp_mat,
            .scores = {},
            .descriptors = curr.descriptors_mat_,
        };
    ref_ret.scores.create(ref_ret.keypoints.rows, 1, CV_32F);
    for (int i = 0; i < ref_ret.keypoints.rows; ++i) {
      ref_ret.scores.at<float>(i, 0) = 1.0f;
    }
    cur_ret.scores.create(cur_ret.keypoints.rows, 1, CV_32F);
    for (int i = 0; i < cur_ret.keypoints.rows; ++i) {
      cur_ret.scores.at<float>(i, 0) = 1.0f;
    }

    feature_matcher_->match(
        cur_ret, image_size0, ref_ret, image_size0, matches);

    if (matches.size() <
        static_cast<size_t>(lcd_params_.lcd_min_matched_features_)) {
      LOG(WARNING) << "VLADLCD: LG: Not enough matches found: "
                   << matches.size() << ".";
      return;
    }

    matches_match_query->reserve(matches.size());
    for (const auto& match : matches) {
      matches_match_query->emplace_back(match.trainIdx, match.queryIdx);
    }
  }

  LCDFrame::Ptr processMonoPnP(const Frame& frame,
                               const PointsWithIdMap& W_points_with_ids,
                               const gtsam::Pose3& W_Pose_Blkf) override;

  void cleanFrame(const LCDFrame::Ptr& frame) override {}

  std::vector<LCDFrame::Ptr> new_seq_frames_;
  static size_t new_seq_id_;
};

}  // namespace VIO