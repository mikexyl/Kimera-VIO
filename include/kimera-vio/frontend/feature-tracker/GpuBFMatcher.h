#pragma once

#include <xfeat-cpp/gpu_matcher.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {
class GpuBFMatcher : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(GpuBFMatcher);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GpuBFMatcher);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GpuBFMatcher(int nkpts,
               float min_sim = 0.4,
               bool use_ransac = false,
               std::optional<CameraParams> camera_params = std::nullopt)
      : min_sim_(min_sim),
        use_ransac_(use_ransac),
        camera_params_(camera_params) {
    gpu_matcher_ = std::make_unique<xfeat::CuMatcher>();
    gpu_matcher_->init(nkpts, nkpts, 64);  // Assuming 64 is the descriptor size

    if (use_ransac_) {
      CHECK(camera_params_)
          << "Camera parameters must be set when using RANSAC";
    }

    VLOG(1) << "GpuBFMatcher initialized with max keypoints: " << nkpts;
  }
  virtual ~GpuBFMatcher() = default;

  void track(Frame* ref_frame,
             Frame* cur_frame,
             const std::vector<cv::Point2f>& prevPts,
             std::vector<cv::Point2f>* nextPts,
             std::vector<int>*,
             cv::OutputArray err,
             std::vector<float>* stds,
             std::vector<float>* scores) override {
    throw std::runtime_error(
        "GpuBFMatcher does not support optical flow tracking. ");
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 cv::Mat homography,
                 int search_radius,
                 const std::vector<cv::Point2f>& predictedPts,
                 DMatchVec* matches) override {
    LOG(FATAL) << "GpuBFMatcher removed.";
  }

 private:
  std::unique_ptr<xfeat::CuMatcher> gpu_matcher_;
  float min_sim_ = 0.4;  // Minimum similarity threshold for matching
  bool use_ransac_ = false;

  std::optional<CameraParams> camera_params_;
};
}  // namespace VIO