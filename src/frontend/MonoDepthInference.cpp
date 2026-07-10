#include "kimera-vio/frontend/MonoDepthInference.h"

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#ifdef HAVE_TENSORRT
#include <xfeat-cpp/mono_depth/depth_anything_v3_trt.h>
#endif

namespace VIO {

MonoDepthInference::MonoDepthInference(const MonoDepthParams& params)
    : params_(params), mono_depth_(nullptr) {
  if (!params_.enabled) {
    return;
  }

  if (params_.mode == MonoDepthMode::kMultiView) {
    LOG(FATAL) << "mono_depth.mode=multi_view is not implemented yet.";
  }

#ifdef HAVE_TENSORRT
  if (params_.engine_path.empty()) {
    LOG(FATAL) << "Mono depth is enabled but mono_depth.engine_path is empty.";
  }
  if (!std::filesystem::exists(params_.engine_path)) {
    LOG(FATAL) << "Mono depth TensorRT engine does not exist: "
               << params_.engine_path;
  }

  try {
    xfeat::DepthAnythingV3TRT::Params trt_params;
    trt_params.engine_path = params_.engine_path;
    trt_params.verbose = params_.verbose;
    mono_depth_ = std::make_unique<xfeat::DepthAnythingV3TRT>(trt_params);
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to initialize DA3 mono depth: " << e.what();
  }

  CHECK(mono_depth_);
  LOG(INFO) << "Initialized DA3 mono depth inference with engine: "
            << params_.engine_path;
#else
  LOG(FATAL) << "Mono depth inference requires xfeat-cpp TensorRT support, "
                "but HAVE_TENSORRT is not enabled.";
#endif
}

MonoDepthRawPacket::ConstPtr MonoDepthInference::inferKeyframe(
    const Frame& frame) const {
  if (!params_.enabled) {
    return nullptr;
  }
  CHECK(mono_depth_);
  CHECK(frame.isKeyframe_);
  CHECK(frame.keyframe_id_.has_value());

  const int keyframe_skip = params_.keyframe_skip > 0
                                ? params_.keyframe_skip
                                : 0;
  const FrameId keyframe_interval =
      static_cast<FrameId>(keyframe_skip + 1);
  if ((*frame.keyframe_id_ % keyframe_interval) != 0u) {
    VLOG(1) << "Skipping DA3 mono depth for keyframe " << *frame.keyframe_id_
            << " because mono_depth.keyframe_skip=" << keyframe_skip;
    return nullptr;
  }

  if (params_.mode == MonoDepthMode::kMultiView) {
    LOG(FATAL) << "mono_depth.mode=multi_view is not implemented yet.";
  }

  const cv::Mat bgr_image = toBgrImage(frame.img_);
  if (bgr_image.empty()) {
    LOG(ERROR) << "Skipping mono depth inference for an empty keyframe image.";
    return nullptr;
  }

  const auto& intrinsics = frame.cam_param_.intrinsics_;
  xfeat::CameraIntrinsics xfeat_intrinsics;
  xfeat_intrinsics.fx = intrinsics[0];
  xfeat_intrinsics.fy = intrinsics[1];
  xfeat_intrinsics.cx = intrinsics[2];
  xfeat_intrinsics.cy = intrinsics[3];
  xfeat_intrinsics.width = bgr_image.cols;
  xfeat_intrinsics.height = bgr_image.rows;
  if (xfeat_intrinsics.fx <= 0.0 || xfeat_intrinsics.fy <= 0.0) {
    LOG(ERROR) << "Skipping mono depth inference because camera intrinsics are "
                  "invalid.";
    return nullptr;
  }

  xfeat::MonoDepthResult depth_result;
  try {
    depth_result = mono_depth_->infer(
        bgr_image, std::optional<xfeat::CameraIntrinsics>(xfeat_intrinsics));
  } catch (const std::exception& e) {
    LOG(ERROR) << "DA3 mono depth inference failed: " << e.what();
    return nullptr;
  }

  if (depth_result.depth.empty() || depth_result.depth.type() != CV_32FC1) {
    LOG(ERROR) << "DA3 mono depth returned an empty or non-CV_32FC1 depth map.";
    return nullptr;
  }

  auto packet = std::make_shared<MonoDepthRawPacket>();
  packet->keyframe_id = *frame.keyframe_id_;
  packet->timestamp = frame.timestamp_;
  packet->source_image_bgr = bgr_image.clone();
  packet->depth = depth_result.depth.clone();
  packet->valid_mask = makeValidMask(packet->depth, depth_result.sky_mask);
  packet->intrinsics.fx = xfeat_intrinsics.fx;
  packet->intrinsics.fy = xfeat_intrinsics.fy;
  packet->intrinsics.cx = xfeat_intrinsics.cx;
  packet->intrinsics.cy = xfeat_intrinsics.cy;
  packet->intrinsics.width = xfeat_intrinsics.width;
  packet->intrinsics.height = xfeat_intrinsics.height;
  packet->weight_image =
      makeWeightImage(packet->depth, packet->valid_mask, packet->intrinsics);
  packet->body_T_cam = frame.cam_param_.body_Pose_cam_;
  packet->keypoints = frame.keypoints_;
  packet->landmark_ids = frame.landmarks_;
  packet->metadata = depth_result.metadata;
  return packet;
}

cv::Mat MonoDepthInference::toBgrImage(const cv::Mat& image) {
  cv::Mat bgr_image;
  if (image.empty()) {
    return bgr_image;
  }
  if (image.type() == CV_8UC3) {
    bgr_image = image;
  } else if (image.type() == CV_8UC1) {
    cv::cvtColor(image, bgr_image, cv::COLOR_GRAY2BGR);
  } else if (image.type() == CV_8UC4) {
    cv::cvtColor(image, bgr_image, cv::COLOR_BGRA2BGR);
  } else {
    LOG_EVERY_N(WARNING, 30)
        << "Unsupported mono depth image type: " << image.type();
  }
  return bgr_image;
}

cv::Mat MonoDepthInference::makeValidMask(const cv::Mat& depth,
                                          const cv::Mat& sky_mask) {
  CHECK(!depth.empty());
  CHECK_EQ(depth.type(), CV_32FC1);

  cv::Mat valid_mask(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  const bool has_sky_mask =
      !sky_mask.empty() && sky_mask.type() == CV_8UC1 &&
      sky_mask.rows >= depth.rows && sky_mask.cols >= depth.cols;

  for (int v = 0; v < depth.rows; ++v) {
    const float* depth_row = depth.ptr<float>(v);
    const uint8_t* sky_row = has_sky_mask ? sky_mask.ptr<uint8_t>(v) : nullptr;
    uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
    for (int u = 0; u < depth.cols; ++u) {
      const float z = depth_row[u];
      const bool sky = sky_row != nullptr && sky_row[u] != 0u;
      if (std::isfinite(z) && z > 0.0f && !sky) {
        valid_row[u] = 255u;
      }
    }
  }
  return valid_mask;
}

Eigen::Vector3d MonoDepthInference::backprojectDepthPixel(
    const int u,
    const int v,
    const float z,
    const MonoDepthIntrinsics& intrinsics) {
  return Eigen::Vector3d(
      (static_cast<double>(u) - intrinsics.cx) * static_cast<double>(z) /
          intrinsics.fx,
      (static_cast<double>(v) - intrinsics.cy) * static_cast<double>(z) /
          intrinsics.fy,
      static_cast<double>(z));
}

cv::Mat MonoDepthInference::makeWeightImage(
    const cv::Mat& depth,
    const cv::Mat& valid_mask,
    const MonoDepthIntrinsics& intrinsics) const {
  CHECK(!depth.empty());
  CHECK_EQ(depth.type(), CV_32FC1);

  cv::Mat weights(depth.rows, depth.cols, CV_32FC1, cv::Scalar(0.0f));
  if (valid_mask.empty() || valid_mask.type() != CV_8UC1 ||
      intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    return weights;
  }

  if (!params_.depth_weighting_enabled) {
    for (int v = 0; v < std::min(depth.rows, valid_mask.rows); ++v) {
      const uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
      float* weight_row = weights.ptr<float>(v);
      for (int u = 0; u < std::min(depth.cols, valid_mask.cols); ++u) {
        weight_row[u] = valid_row[u] == 0u ? 0.0f : 1.0f;
      }
    }
    return weights;
  }

  const int radius = std::max(1, params_.depth_weight_normal_radius);
  const bool use_range_weight = params_.depth_weight_range_ref > 0.0;
  const int rows = std::min(depth.rows, valid_mask.rows);
  const int cols = std::min(depth.cols, valid_mask.cols);
  for (int v = radius; v < rows - radius; ++v) {
    const float* depth_row = depth.ptr<float>(v);
    const uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
    float* weight_row = weights.ptr<float>(v);
    for (int u = radius; u < cols - radius; ++u) {
      if (valid_row[u] == 0u ||
          valid_mask.at<uint8_t>(v, u - radius) == 0u ||
          valid_mask.at<uint8_t>(v, u + radius) == 0u ||
          valid_mask.at<uint8_t>(v - radius, u) == 0u ||
          valid_mask.at<uint8_t>(v + radius, u) == 0u) {
        continue;
      }

      const float z = depth_row[u];
      const float z_l = depth.at<float>(v, u - radius);
      const float z_r = depth.at<float>(v, u + radius);
      const float z_u = depth.at<float>(v - radius, u);
      const float z_d = depth.at<float>(v + radius, u);
      const auto depth_is_valid = [this](const float depth_value) {
        return std::isfinite(depth_value) &&
               depth_value >= params_.min_depth_m &&
               depth_value <= params_.max_depth_m;
      };
      if (!depth_is_valid(z) || !depth_is_valid(z_l) ||
          !depth_is_valid(z_r) || !depth_is_valid(z_u) ||
          !depth_is_valid(z_d)) {
        continue;
      }

      const Eigen::Vector3d p = backprojectDepthPixel(u, v, z, intrinsics);
      const Eigen::Vector3d p_l =
          backprojectDepthPixel(u - radius, v, z_l, intrinsics);
      const Eigen::Vector3d p_r =
          backprojectDepthPixel(u + radius, v, z_r, intrinsics);
      const Eigen::Vector3d p_u =
          backprojectDepthPixel(u, v - radius, z_u, intrinsics);
      const Eigen::Vector3d p_d =
          backprojectDepthPixel(u, v + radius, z_d, intrinsics);

      Eigen::Vector3d normal = (p_r - p_l).cross(p_d - p_u);
      const double normal_norm = normal.norm();
      const double point_norm = p.norm();
      if (normal_norm < 1e-9 || point_norm < 1e-9) {
        continue;
      }

      normal /= normal_norm;
      const Eigen::Vector3d view_to_camera = -p / point_norm;
      const double cos_theta =
          std::clamp(std::abs(normal.dot(view_to_camera)), 0.0, 1.0);
      const double grazing_confidence =
          std::pow(cos_theta, params_.depth_weight_grazing_power);
      const double grazing_weight =
          params_.depth_weight_min +
          (1.0 - params_.depth_weight_min) * grazing_confidence;
      const double range_weight =
          use_range_weight
              ? std::clamp(
                    std::pow(params_.depth_weight_range_ref / point_norm,
                             params_.depth_weight_range_power),
                    params_.depth_weight_range_min,
                    1.0)
              : 1.0;
      weight_row[u] =
          static_cast<float>(std::clamp(grazing_weight * range_weight,
                                        0.0,
                                        1.0));
    }
  }
  return weights;
}

}  // namespace VIO
