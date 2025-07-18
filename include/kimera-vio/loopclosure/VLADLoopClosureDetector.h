#pragma once

#include <xfeat-cpp/faiss_database.h>
#include <xfeat-cpp/xfeat_cv.h>
#include <xfeat-cpp/xfeat_netvlad_onnx.h>

#include "kimera-vio/loopclosure/LoopClosureDetector.h"

namespace VIO {

struct XfeatNVWrapper : xfeat::XfeatNetVLADONNX {
  using Base = xfeat::XfeatNetVLADONNX;
  using GlobalDesc = cv::Mat;
  using Desc = cv::Mat;
  using DescVector = std::vector<cv::Mat>;
  using DescMat = cv::Mat;
  using Database = xfeat::FaissDatabase;

  template <typename... Args>
  XfeatNVWrapper(std::unique_ptr<Database> faiss_db, Args&&... args)
      : Base(std::forward<Args>(args)...), db_(std::move(faiss_db)) {}

  void transform(const DescVector& desc_vec, GlobalDesc& global_desc) {
    CHECK(desc_vec.size() == 2) << "XfeatNVWrapper: the feature vector must be "
                                   "the vector of [M1, x_prep]";

    auto M1 = desc_vec[0];
    auto x_prep = desc_vec[1];

    if (M1.empty()) throw std::runtime_error("XfeatNVWrapper: M1 is empty");
    if (x_prep.empty())
      throw std::runtime_error("XfeatNVWrapper: x_prep is empty");

    if (M1.type() != CV_32F || x_prep.type() != CV_32F) {
      throw std::runtime_error(
          "XfeatNVWrapper: M1 and x_prep must be of type CV_32F");
    }

    global_desc = Base::transform(M1, x_prep);
  }

  template <typename... Args>
  void add(Args&&... args) {
    CHECK_NOTNULL(db_);
    try {
      db_->add(std::forward<Args>(args)...);
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
  auto distance(Args&&... args) {
    try {
      return db_->l2_distance(std::forward<Args>(args)...);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to compute distance in database: " << e.what();
      throw;
    }
  }

 private:
  std::unique_ptr<Database> db_;
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
    : public LoopClosureDetector<XfeatNVWrapper,
                                 DummyFeatureDetector,
                                 xfeat::LighterGlueCV> {
 public:
  KIMERA_POINTER_TYPEDEFS(VLADLoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VLADLoopClosureDetector);

  using Database = XfeatNVWrapper;

  static constexpr bool kVLADLCDUseGPU = true;

  template <typename... Args>
  VLADLoopClosureDetector(Ort::Env& env, Args&&... args)
      : LoopClosureDetector(std::forward<Args>(args)...) {
    if (stereo_camera_) {
      throw std::runtime_error("not implemented for stereo cameras");
    } else {
      VLOG(5) << "VLADLoopClosureDetector: using monocular camera";
    }

    CHECK(!lcd_params_.lcd_lg_model_path_.empty())
        << "VLADLoopClosureDetector: lcd_lg_model_path_ must be set!";
    CHECK(!lcd_params_.lcd_faiss_index_path_.empty())
        << "VLADLoopClosureDetector: lcd_faiss_index_path_ must be set!";

    // should not need to run feature detection again, so the detector should be
    // empty
    feature_detector_.reset(new DummyFeatureDetector());

    feature_matcher_ = xfeat::LighterGlueCV::create(
        env,
        xfeat::LighterGlueCV::Params{
            .model_path = lcd_params_.lcd_lg_model_path_,
            .use_gpu = true,
            .n_kpts = lcd_params_.lcd_lg_num_features_,
        });

    auto faiss_db =
        std::make_unique<Database::Database>(lcd_params_.lcd_faiss_index_path_);
    db_ = std::make_unique<Database>(std::move(faiss_db),
                                     env,
                                     lcd_params_.xfeat_nv_head_model_path_,
                                     lcd_params_.netvlad_model_path_,
                                     kVLADLCDUseGPU);
  }

  /* ------------------------------------------------------------------------
   */
  virtual ~VLADLoopClosureDetector() override = default;

  void detectLoop(const FrameId& frame_id,
                  const Database::GlobalDesc& bow_vec,
                  LoopResult* result) override;

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
};

}  // namespace VIO