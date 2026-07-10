#pragma once

#include <gtsam/geometry/Pose3.h>
#include <opencv2/core.hpp>

#include <string>

#include "kimera-vio/common/MonoDepthTypes.h"

namespace VIO {

struct MonoDepthPoseScaleEstimate {
  bool valid = false;
  double da3_camera_displacement = 0.0;
  double odometry_camera_displacement = 0.0;
  double depth_scale = 1.0;
  std::string error;
};

MonoDepthPoseScaleEstimate estimateMonoDepthPoseScale(
    const gtsam::Pose3& da3_context_cam_T_current_cam,
    const gtsam::Pose3& odometry_world_T_context_cam,
    const gtsam::Pose3& odometry_world_T_current_cam);

cv::Mat scaleMonoDepthImage(const cv::Mat& canonical_depth,
                            double depth_scale);

cv::Mat makeMonoDepthWeightImage(const cv::Mat& depth,
                                 const cv::Mat& valid_mask,
                                 const MonoDepthIntrinsics& intrinsics,
                                 const MonoDepthParams& params);

cv::Mat makeMonoDepthWeightImageAtSize(
    const cv::Mat& depth,
    const cv::Mat& valid_mask,
    const MonoDepthIntrinsics& intrinsics,
    const cv::Size& weight_size,
    const MonoDepthParams& params);

float sampleMonoDepthWeight(const cv::Mat& weight_image,
                            const cv::Size& depth_size,
                            int u,
                            int v);

}  // namespace VIO
