/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdCamera.cpp
 * @brief  Class describing a RgbdCamera.
 * @author Antoni Rosinol
 * @author Nathan Hughes
 */

#include "kimera-vio/frontend/RgbdCamera.h"

#include "kimera-vio/frontend/Camera.h"

namespace VIO {

template <typename T>
void convertToPcl(const cv::Mat& intensity_img,
                  const cv::Mat& depth_img,
                  const CameraParams::Intrinsics& intrinsics,
                  const T& depth_factor,
                  cv::Mat* cloud,
                  cv::Mat* colors) {
  throw std::runtime_error(
      "convertToPcl is deleted for removing opencv contrib dependency. ");
}

RgbdCamera::RgbdCamera(const CameraParams& cam_params) : Camera(cam_params) {}

void RgbdCamera::distortKeypoints(
    const StatusKeypointsCV& keypoints_undistorted,
    KeypointsCV* keypoints) const {
  undistorter_->distortUnrectifyKeypoints(keypoints_undistorted, keypoints);
}

StereoCalibPtr RgbdCamera::getFakeStereoCalib() const {
  return StereoCalibPtr(
      new gtsam::Cal3_S2Stereo(calibration_.fx(),
                               calibration_.fy(),
                               calibration_.skew(),
                               calibration_.px(),
                               calibration_.py(),
                               cam_params_.depth.virtual_baseline_));
}

gtsam::StereoCamera RgbdCamera::getFakeStereoCamera() const {
  return {gtsam::Pose3(), getFakeStereoCalib()};
}

void RgbdCamera::convertRgbdToPointcloud(const RgbdFrame& rgbd_frame,
                                         cv::Mat* cloud,
                                         cv::Mat* colors) {
  CHECK_NOTNULL(cloud);
  CHECK_NOTNULL(colors);
  const auto& depth_type = rgbd_frame.depth_img_.depth_img_.type();
  if (depth_type == CV_16UC1) {
    return convertToPcl<uint16_t>(rgbd_frame.intensity_img_.img_,
                                  rgbd_frame.depth_img_.depth_img_,
                                  cam_params_.intrinsics_,
                                  depth_factor_,
                                  cloud,
                                  colors);
  } else if (depth_type == CV_32FC1) {
    return convertToPcl<float>(rgbd_frame.intensity_img_.img_,
                               rgbd_frame.depth_img_.depth_img_,
                               cam_params_.intrinsics_,
                               static_cast<float>(depth_factor_),
                               cloud,
                               colors);

  } else {
    LOG(FATAL) << "Unrecognized depth image type.";
  }
}

}  // namespace VIO
