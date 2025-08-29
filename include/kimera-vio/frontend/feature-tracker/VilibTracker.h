#pragma once

#include <vilib/config.h>
#include <vilib/feature_detection/detector_base_gpu.h>
#include <vilib/feature_detection/harris/harris_gpu.h>
#include <vilib/feature_tracker/feature_tracker_gpu.h>
#include <vilib/preprocess/conv_filter.h>
#include <vilib/storage/pyramid_pool.h>

#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {

class VilibTracker : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(VilibTracker);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VilibTracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct DetectorOptions {
    int cell_width{32};
    int cell_height{32};
    int min_level{0};
    int max_level{2};
    int horizontal_border{8};
    int vertical_border{8};
  };

  struct ShiTomasiOptions : public DetectorOptions {
    ShiTomasiOptions() = default;
    ShiTomasiOptions(const DetectorOptions& other) : DetectorOptions(other) {}
    vilib::conv_filter_border_type border_type{
        vilib::conv_filter_border_type::BORDER_SKIP};
    float quality_level{0.1};
  };

  struct Params {
    vilib::FeatureTrackerOptions feature_tracker_options_;
    int width, height;
    ShiTomasiOptions detector_options_;
    int n_pyramid_levels_{1};
  };

  VilibTracker(const Params& params) : params_(params) {
    std::shared_ptr<vilib::DetectorBaseGPU> detector =
        std::make_shared<vilib::HarrisGPU>(
            params_.width,
            params_.height,
            params_.detector_options_.cell_width,
            params_.detector_options_.cell_height,
            params_.detector_options_.min_level,
            params_.detector_options_.max_level,
            params_.detector_options_.horizontal_border,
            params_.detector_options_.vertical_border,
            params_.detector_options_.border_type,
            true,
            0.0,
            params_.detector_options_.quality_level);
    vilib::FeatureTrackerOptions ft_options = params.feature_tracker_options_;

    params_.n_pyramid_levels_ = params_.detector_options_.max_level + 1;
    vilib::PyramidPool::init(IMAGE_PYRAMID_PREALLOCATION_ITEM_NUM,
                             params_.width,
                             params_.height,
                             1,
                             params_.n_pyramid_levels_,
                             vilib::IMAGE_PYRAMID_MEMORY_TYPE);

    feature_tracker_ =
        std::make_shared<vilib::FeatureTrackerGPU>(ft_options, 1);
    feature_tracker_->setDetectorGPU(detector, 0);
  }

  void track(Frame* ref_frame,
             Frame* cur_frame,
             const std::vector<cv::Point2f>& prevPts,
             std::vector<cv::Point2f>* nextPts,
             std::vector<int>* prevNextIds,
             cv::OutputArray err,
             cv::Size /*winSize*/ = cv::Size(21, 21),
             int /*maxLevel*/ = 3,
             cv::TermCriteria /*criteria*/ = cv::TermCriteria(
                 cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                 30,
                 0.01),
             int /*flags*/ = 0,
             double /*minEigThreshold*/ = 1e-4) override {
    if (ref_frame) {
      CHECK_EQ(ref_frame->id_, prev_frame_id_);
    }

    size_t num_prev_keypoints = ref_frame ? ref_frame->keypoints_.size() : 0;

    cv::Mat gray_image;
    cv::cvtColor(cur_frame->img_, gray_image, cv::COLOR_BGR2GRAY);

    auto vilib_frame = std::make_shared<vilib::Frame>(
        gray_image, cur_frame->timestamp_, params_.n_pyramid_levels_);
    auto vilib_frame_bundle = std::make_shared<vilib::FrameBundle>(
        std::vector<std::shared_ptr<vilib::Frame>>({vilib_frame}));
    size_t n_tracked, n_detected;
    feature_tracker_->track(vilib_frame_bundle, n_tracked, n_detected);

    const Eigen::Matrix<double, 2, Eigen::Dynamic> features =
        vilib_frame->px_vec_;
    const Eigen::VectorXi ids = vilib_frame->track_id_vec_;
    std::map<int, size_t> feature_id_to_kp_id;
    for (size_t i = 0; i < vilib_frame->num_features_; ++i) {
      feature_id_to_kp_id[ids[i]] = i;
    }

    // resize nextPts to number of keypoints * 2
    CHECK_GT(vilib_frame->num_features_, 0);
    nextPts->resize(vilib_frame->num_features_);

    // populate next pts mat
    std::map<size_t, int> cur_kp_id_to_feature_id, cur_feature_id_to_kp_id;
    for (size_t i = 0; i < vilib_frame->num_features_; ++i) {
      nextPts->at(i).x = features(0, i);
      nextPts->at(i).y = features(1, i);
      cur_kp_id_to_feature_id[i] = ids[i];
      cur_feature_id_to_kp_id[ids[i]] = i;
    }

    // Always keep it vector-shaped (Nx1). N==0 is OK.
    prevNextIds->clear();

    if (num_prev_keypoints > 0 and not prev_kp_id_to_feature_id_.empty()) {
      prevNextIds->resize(num_prev_keypoints, 1);
      // populate the id matching
      for (size_t i = 0; i < num_prev_keypoints; ++i) {
        // this is possible if other detector adds new keypoints
        if (prev_kp_id_to_feature_id_.find(i) ==
            prev_kp_id_to_feature_id_.end()) {
          continue;
        }
        int prev_feature_id = prev_kp_id_to_feature_id_.at(i);
        if (cur_feature_id_to_kp_id.find(prev_feature_id) !=
            cur_feature_id_to_kp_id.end()) {
          prevNextIds->at(i) = cur_feature_id_to_kp_id[prev_feature_id];
        } else {
          prevNextIds->at(i) = -1;
        }
      }
    }

    prev_kp_id_to_feature_id_ = cur_kp_id_to_feature_id;
    prev_frame_id_ = cur_frame->id_;
  }

  std::shared_ptr<vilib::FeatureTrackerGPU> feature_tracker_;
  std::map<size_t, int> prev_kp_id_to_feature_id_;
  FrameId prev_frame_id_{0};

  Params params_;
};

}  // namespace VIO