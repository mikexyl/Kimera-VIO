#pragma once

#include <optional>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cuda_runtime.h>
#include <xfeat-cpp/faiss_database.h>
#include <xfeat-cpp/lighterglue_trt.h>
#include <xfeat-cpp/place_recognition/jist_onnx.h>
#include <xfeat-cpp/place_recognition/jist_trt.h>
#include <xfeat-cpp/place_recognition/mixvpr_onnx.h>
#include <xfeat-cpp/place_recognition/mixvpr_trt.h>
#include <xfeat-cpp/place_recognition/patchnetvlad_onnx.h>
#include <xfeat-cpp/place_recognition/place_recognizer.h>

#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/loopclosure/FrameCache.h"
#include "kimera-vio/loopclosure/LandmarkManager.h"
#include "kimera-vio/loopclosure/LcdGridFrame.h"
#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/loopclosure/LcdThirdPartyWrapper.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

// Generic VPR wrapper: holds any PlaceRecognizer + a FAISS database.
struct VPRONNXWrapper {
  using GlobalDesc = cv::Mat;
  using Desc = cv::Mat;
  using DescVector = std::vector<cv::Mat>;
  using DescMat = cv::Mat;
  using Database = xfeat::FaissDatabase;

  VPRONNXWrapper(std::unique_ptr<Database> faiss_db,
                 std::unique_ptr<xfeat::PlaceRecognizer> model)
      : model_(std::move(model)), db_(std::move(faiss_db)) {}

  int get_seq_length() const { return model_->get_seq_length(); }
  int get_descriptor_dim() const { return model_->get_descriptor_dim(); }

  void transform(std::vector<LCDFrame::Ptr> frames, GlobalDesc& global_desc) {
    std::vector<cv::Mat> image_sequence;
    image_sequence.reserve(frames.size());
    CHECK_EQ(static_cast<int>(frames.size()), model_->get_seq_length());
    for (const auto& frame : frames) {
      image_sequence.push_back(frame->image_);
    }
    global_desc = model_->infer(image_sequence);
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
      return GlobalDesc();
    }
  }

 private:
  std::unique_ptr<xfeat::PlaceRecognizer> model_;
  std::unique_ptr<Database> db_;
  std::map<faiss::idx_t, cv::Mat> id_to_desc_map_;
};

class VLADLoopClosureDetector : public LoopClosureDetectorBase {
 public:
  KIMERA_POINTER_TYPEDEFS(VLADLoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VLADLoopClosureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr bool kVLADLCDUseGPU = true;

  VLADLoopClosureDetector(
      Ort::Env& env,
      const LoopClosureDetectorParams& lcd_params,
      const CameraParams& tracker_cam_params,
      const gtsam::Pose3& B_Pose_Cam,
      const std::optional<VIO::StereoCamera::ConstPtr>& stereo_camera,
      const std::optional<StereoMatchingParams>& stereo_matching_params,
      const std::optional<VIO::RgbdCamera::ConstPtr>& rgbd_camera,
      bool log_output);

  ~VLADLoopClosureDetector() override = default;

  /* ------------------------------------------------------------------------
   */
  /**
   * @brief Register a loop closure between two frames in a threadsafe manner
   */
  LoopResult registerFrames(FrameId query_id, FrameId match_id) override;

  LcdOutput::UniquePtr spinOnce(const LcdInput& input) override;

  inline void registerIsBackendQueueFilledCallback(
      const IsBackendQueueFilledCallback& cb) override {}

  /* ------------------------------------------------------------------------
   */
  /** @brief Computes the indices of keypoints that match between two frames
   * using LighterGlue.
   */
  void computeDescriptorMatches(const LCDFrame& ref,
                                const LCDFrame& curr,
                                KeypointMatches* matches_match_query,
                                bool cut_matches = false) const;

  void verifyAndRecoverPose(LoopResult* result);

  double computeSequenceScore(const FrameId anchor_frame_id);

  std::optional<FrameId> getCurrentAnchorFrameId() {
    if (new_seq_frames_.empty()) {
      return std::optional<FrameId>();
    } else {
      return std::optional<FrameId>(new_seq_frames_.begin()->get()->id_);
    }
  }

  void computeSequenceGlobalDesc(const FrameId target_frame_id,
                                 bool add_to_sequence);

  bool finalizeSequenceFrames(
      const std::vector<LCDFrame::Ptr>& sequence_frames,
      bool force_finalize_short_sequence);

  void detectLoop(const FrameId& frame_id,
                  LoopResult* result,
                  FrameId* query_frame = nullptr,
                  FrameIdSet* global_candidates = nullptr);

  void detectLoopOutsideLocalWindow(const FrameId& frame_id,
                                    LoopResult* result,
                                    FrameId* query_frame = nullptr,
                                    FrameIdSet* global_candidates = nullptr);

  std::optional<FrameId> findFirstFrameIdOutsideLocalWindow(
      const FrameId& frame_id) const {
    if (frame_id < static_cast<FrameId>(lcd_params_.local_window_size_)) {
      return std::nullopt;
    } else {
      return frame_id - lcd_params_.local_window_size_;
    }
  }

  void getNewFeaturesAndDescriptors(const Frame& frame,
                                    std::vector<cv::KeyPoint>* keypoints,
                                    cv::Mat* descriptors_mat);

  void descriptorMatToVec(const Frame& frame,
                          const cv::Mat& descriptors_mat,
                          std::vector<cv::Mat>* descriptors_vec);

  std::map<int, double> globalDescToMap(const cv::Mat& global_desc) {
    std::map<int, double> desc_map;
    CHECK_EQ(global_desc.rows, 1);
    for (int i = 0; i < global_desc.cols; ++i) {
      desc_map[i] = global_desc.at<float>(0, i);
    }
    return desc_map;
  }

  LCDFrame::Ptr processMonoPnP(const Frame& frame,
                               const PointsWithIdMap& W_points_with_ids,
                               const gtsam::Pose3& W_Pose_Blkf);

  /* ------------------------------------------------------------------------
   */
  /**
   * @brief Processed a single frame and adds it to relevant internal
   * databases.
   */
  FrameId processAndAddMonoFrame(const Frame& frame,
                                 const PointsWithIdMap& W_points_with_ids,
                                 const gtsam::Pose3& W_Pose_Blkf);

  FrameId processAndAddStereoFrame(const StereoFrame& stereo_frame,
                                   const Pose3& W_Pose_Blkf);

  FrameId processAndAddRgbdFrame(const RgbdFrame& rgbd_frame);

  const FrameCache& getFrameCache() const { return cache_; }

  LcdOutput::UniquePtr makeOutputPayload(Timestamp msg_timestamp,
                                         FrameId lcd_frame_id);

  void filterKeypointsWithGrid(
      int img_width,
      int img_height,
      int grid_cols,
      int grid_rows,
      std::vector<cv::KeyPoint>* keypoints,
      Landmarks* landmarks,
      cv::Mat* descriptors_mat,
      BearingVectors* bearing_vectors,
      std::vector<StatusKeypointCV>* left_kpts_rect = nullptr,
      std::vector<StatusKeypointCV>* right_kpts_rect = nullptr,
      std::vector<LandmarkId>* landmark_ids = nullptr) const;

  std::optional<LcdGridFrame> augmentAndFilterFrameFeatures(
      FrameId lcd_frame_id);

  bool geometricVerificationCam2d2d(const LCDFrame& ref_frame,
                                    const LCDFrame& cur_frame,
                                    const KeypointMatches& matches_match_query,
                                    gtsam::Pose3* camMatch_T_camQuery_2d,
                                    std::vector<int>* inliers);

  LCDStatus recoverPoseBody(const LCDFrame& ref_frame,
                            const LCDFrame& cur_frame,
                            const gtsam::Pose3& camMatch_T_camQuery_2d,
                            const KeypointMatches& matches_match_query,
                            gtsam::Pose3* bodyMatch_T_bodyQuery_3d,
                            gtsam::Pose3* bodyQuery_T_bodyMatch_3d,
                            std::vector<int>* inliers);

  gtsam::Pose3 refinePoses(const StereoLCDFrame& ref_frame,
                           const StereoLCDFrame& cur_frame,
                           const gtsam::Pose3& camMatch_T_camQuery_3d,
                           const KeypointMatches& matches_match_query);

  void transformCameraPoseToBodyPose(
      const gtsam::Pose3& camMatch_T_camQuery,
      gtsam::Pose3* bodyMatch_T_bodyQuery) const {
    CHECK_NOTNULL(bodyMatch_T_bodyQuery);
    *bodyMatch_T_bodyQuery =
        B_Pose_Cam_ * camMatch_T_camQuery * B_Pose_Cam_.inverse();
  }

  void transformBodyPoseToCameraPose(
      const gtsam::Pose3& bodyMatch_T_bodyQuery,
      gtsam::Pose3* camMatch_T_camQuery) const {
    CHECK_NOTNULL(camMatch_T_camQuery);
    *camMatch_T_camQuery =
        B_Pose_Cam_.inverse() * bodyMatch_T_bodyQuery * B_Pose_Cam_;
  }

  void cleanFrame(const FrameId& frame_id) {
    auto frame = cache_.getFrame(frame_id);
    if (frame) {
      cache_.removeFrame(frame_id);
    } else {
      LOG(WARNING) << "LoopClosureDetector: Attempted to clean frame with ID "
                   << frame_id << " but it does not exist in the cache.";
    }
  }

  void cleanFrame(const LCDFrame::Ptr& frame) {}

  void cleanFrameUntil(const FrameId& frame_id) {
    for (const auto& id : cache_.getFrameIds()) {
      if (id < frame_id) {
        cache_.removeFrame(id);
      }
    }
  }

  void updatePoseGraph(const gtsam::Values& smoother_states,
                       const gtsam::Pose3& T_W_B);

  void print() const { lcd_params_.print(); }

  std::vector<LCDFrame::Ptr> new_seq_frames_;
  static size_t new_seq_id_;
  std::optional<FrameId> last_seq_end_frame_id_;
  bool active_seq_boundary_reached_ = false;
  size_t total_sequence_frame_span_ = 0u;
  size_t num_finalized_sequences_ = 0u;

 protected:
  enum class LcdState {
    Bootstrap,
    Nominal
  };
  LcdState lcd_state_ = LcdState::Bootstrap;

  LoopClosureDetectorParams lcd_params_;

  std::unique_ptr<xfeat::LighterGlueTRT> feature_matcher_;

  StereoCamera::ConstPtr stereo_camera_;
  StereoMatchingParams stereo_matching_params_;
  StereoMatcher::UniquePtr stereo_matcher_;

  RgbdCamera::ConstPtr rgbd_camera_;

  FrameCache cache_;
  FrameIDTimestampMap timestamp_map_;

  gtsam::SharedNoiseModel shared_noise_model_;

  gtsam::Pose3 B_Pose_Cam_;

  std::unique_ptr<cv::Mat> latest_global_vec_;

  Tracker::UniquePtr tracker_;

  std::unique_ptr<LcdLandmarkManager> landmark_manager_{nullptr};

  std::unique_ptr<LcdThirdPartyWrapper> lcd_tp_wrapper_;

  std::map<std::pair<FrameId, FrameId>, gtsam::NonlinearFactor::shared_ptr> pg_;
  gtsam::Values pg_values_;

  std::unique_ptr<LoopClosureDetectorLogger> logger_;
  LcdDebugInfo debug_info_;

  std::vector<std::vector<FrameId>> seq_frames_;

  const bool log_output_ = false;

 private:
  std::unique_ptr<VPRONNXWrapper> vpr_db_;
};

}  // namespace VIO
