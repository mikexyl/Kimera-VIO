#pragma once

#include <cuda_runtime_api.h>
#include <glog/logging.h>
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
        vilib::conv_filter_border_type::BORDER_WRAP};
    float quality_level{0.1};
  };

  struct Params {
    vilib::FeatureTrackerOptions feature_tracker_options_;
    int width, height;
    ShiTomasiOptions detector_options_;
    int n_pyramid_levels_{1};
    // If true, images will be downsampled before tracking.
    // The downsample scale is multiplied by the input width/height to obtain
    // the internal tracker image size used to initialize the detector and
    // pyramid structures. Must be in (0, 1].
    bool downsample{false};
    float downsample_scale{0.5f};
  };

  VilibTracker(const Params& params) : params_(params) {
    // Compute internal tracker size. By default this is the input image size,
    // but if downsampling is enabled use the provided scale.
    tracker_width_ = params_.width;
    tracker_height_ = params_.height;
    if (params_.downsample) {
      CHECK_GT(params_.downsample_scale, 0.0f);
      CHECK_LE(params_.downsample_scale, 1.0f);
      tracker_width_ = std::max(
          1, static_cast<int>(params_.width * params_.downsample_scale));
      tracker_height_ = std::max(
          1, static_cast<int>(params_.height * params_.downsample_scale));
    }

    std::shared_ptr<vilib::DetectorBaseGPU> detector =
        std::make_shared<vilib::HarrisGPU>(
            tracker_width_,
            tracker_height_,
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
                             tracker_width_,
                             tracker_height_,
                             1,
                             params_.n_pyramid_levels_,
                             vilib::IMAGE_PYRAMID_MEMORY_TYPE);

    feature_tracker_ =
        std::make_shared<vilib::FeatureTrackerGPU>(ft_options, 1);
    feature_tracker_->setDetectorGPU(detector, 0);
  }

 private:
  void logGpuMemoryUsage(const std::string& label) {
    size_t free_byte;
    size_t total_byte;
    cudaError_t cuda_status = cudaMemGetInfo(&free_byte, &total_byte);
    if (cudaSuccess != cuda_status) {
      LOG(ERROR) << "Error getting GPU memory info: "
                 << cudaGetErrorString(cuda_status);
      return;
    }
    size_t used_byte = total_byte - free_byte;
    VLOG(10) << label
              << " - GPU Memory: Used=" << used_byte / (1024.0 * 1024.0)
              << " MB, Free=" << free_byte / (1024.0 * 1024.0)
              << " MB, Total=" << total_byte / (1024.0 * 1024.0) << " MB";
  }

  void track(Frame* ref_frame,
             Frame* cur_frame,
             const std::vector<cv::Point2f>& prevPts,
             std::vector<cv::Point2f>* nextPts,
             std::vector<int>* prevNextIds,
             cv::OutputArray err,
             std::vector<float>* std,
             std::vector<float>* scores) override {
    logGpuMemoryUsage("Before tracking");

    // TODO: change for stereo mode accordingly
    // if (ref_frame) {
    //   CHECK_EQ(ref_frame->id_, prev_frame_id_);
    // }

    size_t num_prev_keypoints = ref_frame ? ref_frame->of_keypoints_.size() : 0;

    cv::Mat gray_image;
    cv::cvtColor(cur_frame->img_, gray_image, cv::COLOR_BGR2GRAY);
    // Verify dimensions match expected parameters
    CHECK_EQ(gray_image.cols, params_.width) << "Image width mismatch";
    CHECK_EQ(gray_image.rows, params_.height) << "Image height mismatch";
    CHECK_EQ(gray_image.type(), CV_8UC1) << "Expected CV_8UC1 image type";
    CHECK_EQ(gray_image.step[0], gray_image.cols * gray_image.elemSize())
        << "Image pitch/stride invalid for CUDA";

    // Optionally downsample the image before creating the vilib frame used for
    // tracking. The detector and pyramid pool were initialized using the
    // tracker size (tracker_width_/tracker_height_).
    cv::Mat tracker_image = gray_image;
    if (params_.downsample) {
      if (gray_image.cols != tracker_width_ ||
          gray_image.rows != tracker_height_) {
        cv::Mat small;
        cv::resize(gray_image,
                   small,
                   cv::Size(tracker_width_, tracker_height_),
                   0,
                   0,
                   cv::INTER_AREA);
        tracker_image = small;
      }
    }

    auto vilib_frame = std::make_shared<vilib::Frame>(
        tracker_image, cur_frame->timestamp_, params_.n_pyramid_levels_);
    auto vilib_frame_bundle = std::make_shared<vilib::FrameBundle>(
        std::vector<std::shared_ptr<vilib::Frame>>({vilib_frame}));
    size_t n_tracked, n_detected;
    feature_tracker_->track(vilib_frame_bundle, n_tracked, n_detected);

    logGpuMemoryUsage("After tracking");

    // feature_tracker_->showAdditionalStat(true);

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
      nextPts->at(i).x = features(0, i) / params_.downsample_scale;
      nextPts->at(i).y = features(1, i) / params_.downsample_scale;
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

    auto levels = vilib_frame->level_vec_;
    std->resize(vilib_frame->num_features_);
    for (size_t i = 0; i < vilib_frame->num_features_; ++i) {
      (*std)[i] = (levels[i] + 1) * 8;
    }
    auto feature_scores = vilib_frame->score_vec_;
    scores->resize(vilib_frame->num_features_, 0.0f);
    for (size_t i = 0; i < vilib_frame->num_features_; ++i) {
      (*scores)[i] = feature_scores[i];
    }

    // find min/max of scores
    auto [min_score, max_score] =
        std::minmax_element(scores->begin(), scores->end());

    // normalize scores to 0-1
    float score_range = *max_score - *min_score;
    if (score_range > 0) {
      for (size_t i = 0; i < scores->size(); ++i) {
        (*scores)[i] = ((*scores)[i] - *min_score) / score_range;
      }
    }

    prev_kp_id_to_feature_id_ = cur_kp_id_to_feature_id;
    prev_frame_id_ = cur_frame->id_;
  }

  std::shared_ptr<vilib::FeatureTrackerGPU> feature_tracker_;
  std::map<size_t, int> prev_kp_id_to_feature_id_;
  FrameId prev_frame_id_{0};

  // Internal tracker image size (may be downsampled from the input image).
  int tracker_width_{0};
  int tracker_height_{0};

  Params params_;
};

}  // namespace VIO