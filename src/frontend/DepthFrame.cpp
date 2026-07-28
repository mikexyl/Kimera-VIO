/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DepthFrame.cpp
 * @brief  Class describing a single Depth image
 * @author Antoni Rosinol
 */
#include "kimera-vio/frontend/DepthFrame.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <cmath>
#include <vector>

#include "kimera-vio/frontend/CameraParams.h"

namespace VIO {
namespace {

cv::Mat registerDepthImage(const CameraParams& params,
                           const cv::Mat& source_depth) {
  LOG(FATAL) << "The Jetson build does not provide OpenCV's validated rgbd "
                "depth registration module; refusing to execute the "
                "temporary fallback.";

  CHECK(!params.depth.K_.empty());
  CHECK(!params.K_.empty());
  CHECK_EQ(params.depth.T_color_depth_.rows, 4);
  CHECK_EQ(params.depth.T_color_depth_.cols, 4);
  CHECK_GT(params.depth.depth_to_meters_, 0.0f);

  cv::Mat color_T_depth;
  params.depth.T_color_depth_.convertTo(color_T_depth, CV_64F);
  const cv::Matx33d color_R_depth(
      color_T_depth.at<double>(0, 0), color_T_depth.at<double>(0, 1),
      color_T_depth.at<double>(0, 2), color_T_depth.at<double>(1, 0),
      color_T_depth.at<double>(1, 1), color_T_depth.at<double>(1, 2),
      color_T_depth.at<double>(2, 0), color_T_depth.at<double>(2, 1),
      color_T_depth.at<double>(2, 2));
  const cv::Vec3d color_t_depth(color_T_depth.at<double>(0, 3),
                                color_T_depth.at<double>(1, 3),
                                color_T_depth.at<double>(2, 3));

  cv::Mat depth_K;
  params.depth.K_.convertTo(depth_K, CV_64F);
  const double fx = depth_K.at<double>(0, 0);
  const double fy = depth_K.at<double>(1, 1);
  const double cx = depth_K.at<double>(0, 2);
  const double cy = depth_K.at<double>(1, 2);

  std::vector<cv::Point3f> color_points;
  color_points.reserve(source_depth.total());
  for (int v = 0; v < source_depth.rows; ++v) {
    for (int u = 0; u < source_depth.cols; ++u) {
      double raw_depth = 0.0;
      if (source_depth.type() == CV_32FC1) {
        raw_depth = source_depth.at<float>(v, u);
      } else {
        raw_depth = source_depth.at<uint16_t>(v, u);
      }
      const double depth_m =
          raw_depth * static_cast<double>(params.depth.depth_to_meters_);
      if (!std::isfinite(depth_m) || depth_m <= 0.0) {
        continue;
      }
      const cv::Vec3d depth_point((u - cx) / fx * depth_m,
                                  (v - cy) / fy * depth_m,
                                  depth_m);
      const cv::Vec3d color_point =
          color_R_depth * depth_point + color_t_depth;
      if (color_point[2] > 0.0 && std::isfinite(color_point[0]) &&
          std::isfinite(color_point[1]) && std::isfinite(color_point[2])) {
        color_points.emplace_back(color_point[0],
                                  color_point[1],
                                  color_point[2]);
      }
    }
  }

  cv::Mat registered = cv::Mat::zeros(params.image_size_, source_depth.type());
  if (color_points.empty()) {
    return registered;
  }

  std::vector<cv::Point2f> color_pixels;
  const cv::Vec3d zero_rotation(0.0, 0.0, 0.0);
  const cv::Vec3d zero_translation(0.0, 0.0, 0.0);
  if (params.distortion_model_ == DistortionModel::EQUIDISTANT) {
    cv::fisheye::projectPoints(color_points,
                               color_pixels,
                               zero_rotation,
                               zero_translation,
                               params.K_,
                               params.distortion_coeff_mat_);
  } else {
    CHECK(params.distortion_model_ == DistortionModel::NONE ||
          params.distortion_model_ == DistortionModel::RADTAN)
        << "Depth registration does not support omni projection.";
    cv::projectPoints(color_points,
                      zero_rotation,
                      zero_translation,
                      params.K_,
                      params.distortion_coeff_mat_,
                      color_pixels);
  }

  std::vector<float> nearest_depth(registered.total(),
                                   std::numeric_limits<float>::infinity());
  for (size_t i = 0; i < color_points.size(); ++i) {
    const int u = cvRound(color_pixels[i].x);
    const int v = cvRound(color_pixels[i].y);
    if (u < 0 || u >= registered.cols || v < 0 || v >= registered.rows) {
      continue;
    }
    const size_t output_index =
        static_cast<size_t>(v) * registered.cols + u;
    if (color_points[i].z >= nearest_depth[output_index]) {
      continue;
    }
    nearest_depth[output_index] = color_points[i].z;
    const float raw_depth =
        color_points[i].z / params.depth.depth_to_meters_;
    if (registered.type() == CV_32FC1) {
      registered.at<float>(v, u) = raw_depth;
    } else {
      registered.at<uint16_t>(v, u) =
          cv::saturate_cast<uint16_t>(raw_depth);
    }
  }
  return registered;
}

}  // namespace

DepthFrame::DepthFrame(const FrameId& id,
                       const Timestamp& timestamp,
                       const cv::Mat& depth_img)
    : PipelinePayload(timestamp),
      id_(id),
      depth_img_(depth_img),
      is_registered_(false) {
  CHECK(depth_img_.type() == CV_32FC1 || depth_img_.type() == CV_16UC1);
}

DepthFrame::DepthFrame(const DepthFrame& other)
    : PipelinePayload(other.timestamp_),
      id_(other.id_),
      depth_img_(other.depth_img_),
      is_registered_(other.is_registered_),
      registered_img_(other.registered_img_) {}

float DepthFrame::getDepthAtPoint(const CameraParams& params,
                                  const KeypointCV& point) const {
  const auto x = static_cast<int>(point.x);
  const auto y = static_cast<int>(point.y);

  float depth = std::numeric_limits<float>::quiet_NaN();
  if (x < 0 || x >= depth_img_.cols || y < 0 || y >= depth_img_.rows) {
    VLOG(10) << "Found feature (" << point.x << ", " << point.y
             << ") outside image bounds: [" << depth_img_.cols << " x "
             << depth_img_.rows << "]";
    return depth;
  }

  const cv::Mat& img = is_registered_ ? registered_img_ : depth_img_;
  switch (depth_img_.type()) {
    case CV_32FC1:
      depth = img.at<float>(y, x);
      break;
    case CV_16UC1:
      depth = img.at<uint16_t>(y, x);
      break;
    default:
      LOG(FATAL) << "Invalid depth datatype: " << depth_img_.type();
      return depth;
  }

  depth *= params.depth.depth_to_meters_;
  if (depth < params.depth.min_depth_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  // TODO(nathan) optionally filter by max depth as well

  return depth;
}

cv::Mat DepthFrame::getDetectionMask(const CameraParams& params) const {
  float min = params.depth.min_depth_ * 1.0f / params.depth.depth_to_meters_;
  float max = params.depth.max_depth_ * 1.0f / params.depth.depth_to_meters_;
  const cv::Mat& img_to_use = is_registered_ ? registered_img_ : depth_img_;
  cv::Mat mask;
  switch (depth_img_.type()) {
    case CV_32FC1:
      cv::inRange(img_to_use, min, max, mask);
      break;
    case CV_16UC1:
      cv::inRange(img_to_use,
                  static_cast<uint16_t>(min),
                  static_cast<uint16_t>(max),
                  mask);
      break;
    default:
      LOG(FATAL) << "Invalid depth datatype: " << img_to_use.type();
      break;
  }

  return mask;
}

void DepthFrame::registerDepth(const CameraParams& params) const {
  if (is_registered_) {
    return;
  }

  registered_img_ = registerDepthImage(params, depth_img_);
  is_registered_ = true;
}

}  // namespace VIO
