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
    : params_(params), mono_depth_(nullptr), buffered_keyframe_(std::nullopt) {
  if (!std::isfinite(params_.min_confidence) ||
      params_.min_confidence < 0.0) {
    LOG(FATAL) << "mono_depth.min_confidence must be finite and non-negative, "
               << "but got " << params_.min_confidence;
  }
  if (!params_.enabled) {
    return;
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
    auto da3 = std::make_unique<xfeat::DepthAnythingV3TRT>(trt_params);
    if (da3->has_camera_inputs()) {
      throw std::invalid_argument(
          "Mono-depth integration requires an image-only DA3 engine; camera "
          "input bindings are not supported");
    }
    mono_depth_ = std::move(da3);
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to initialize DA3 mono depth: " << e.what();
  }

  CHECK(mono_depth_);
  LOG(INFO) << "Initialized DA3 mono depth inference with engine: "
            << params_.engine_path << ", mode="
            << monoDepthModeToString(params_.mode)
            << ", min_confidence=" << params_.min_confidence;
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

  const std::optional<BufferedKeyframe> current = bufferKeyframe(frame);
  if (!current.has_value()) {
    return nullptr;
  }

  if (params_.mode == MonoDepthMode::kSingleView) {
    xfeat::MonoDepthResult depth_result;
    try {
      depth_result = mono_depth_->infer(
          current->image_bgr,
          std::optional<xfeat::CameraIntrinsics>(
              toXfeatIntrinsics(current->intrinsics)));
    } catch (const std::exception& e) {
      LOG(ERROR) << "DA3 single-view mono depth inference failed for keyframe "
                 << current->keyframe_id << ": " << e.what();
      return nullptr;
    }
    return buildPacket(*current, depth_result, false);
  }

  if (!buffered_keyframe_.has_value()) {
    buffered_keyframe_ = current;
    LOG(INFO) << "Primed DA3 two-view buffer with keyframe "
              << current->keyframe_id;
    return nullptr;
  }

  BufferedKeyframe previous = std::move(*buffered_keyframe_);
  // Advance before inference so a failed pair does not get retried and the
  // next sampled keyframe still uses the newest available context view.
  buffered_keyframe_ = current;

  LOG(INFO) << "Running pose-free DA3 two-view pair ["
            << previous.keyframe_id << ", " << current->keyframe_id << "]";
  std::vector<xfeat::MonoDepthResult> depth_results;
  try {
    const std::vector<cv::Mat> images{previous.image_bgr, current->image_bgr};
    const std::vector<xfeat::CameraIntrinsics> intrinsics{
        toXfeatIntrinsics(previous.intrinsics),
        toXfeatIntrinsics(current->intrinsics)};
    depth_results = mono_depth_->infer_multi_view(images, intrinsics);
  } catch (const std::exception& e) {
    LOG(ERROR) << "DA3 two-view mono depth inference failed for pair ["
               << previous.keyframe_id << ", " << current->keyframe_id
               << "]: " << e.what();
    return nullptr;
  }

  if (depth_results.size() != 2u) {
    LOG(ERROR) << "DA3 two-view inference returned " << depth_results.size()
               << " results for pair [" << previous.keyframe_id << ", "
               << current->keyframe_id << "]; expected exactly 2.";
    return nullptr;
  }

  MonoDepthRawPacket::ConstPtr packet =
      buildPacket(*current, depth_results[1], true);
  if (packet) {
    LOG(INFO) << "DA3 pair [" << previous.keyframe_id << ", "
              << current->keyframe_id << "] emitted keyframe "
              << packet->keyframe_id << " (view_index="
              << packet->metadata.view_index << ", view_count="
              << packet->metadata.view_count << ", confidence_threshold="
              << packet->confidence_threshold << ", accepted="
              << packet->confidence_accepted_pixels << ", rejected="
              << packet->confidence_rejected_pixels << ", retained="
              << packet->confidence_retained_fraction << ")";
  }
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

std::optional<MonoDepthInference::BufferedKeyframe>
MonoDepthInference::bufferKeyframe(const Frame& frame) {
  const cv::Mat bgr_image = toBgrImage(frame.img_);
  if (bgr_image.empty()) {
    LOG(ERROR) << "Skipping mono depth inference for an empty or unsupported "
                  "keyframe image.";
    return std::nullopt;
  }

  BufferedKeyframe buffered;
  buffered.keyframe_id = *frame.keyframe_id_;
  buffered.timestamp = frame.timestamp_;
  buffered.image_bgr = bgr_image.clone();
  buffered.intrinsics.fx = frame.cam_param_.intrinsics_[0];
  buffered.intrinsics.fy = frame.cam_param_.intrinsics_[1];
  buffered.intrinsics.cx = frame.cam_param_.intrinsics_[2];
  buffered.intrinsics.cy = frame.cam_param_.intrinsics_[3];
  buffered.intrinsics.width = bgr_image.cols;
  buffered.intrinsics.height = bgr_image.rows;
  if (!std::isfinite(buffered.intrinsics.fx) ||
      !std::isfinite(buffered.intrinsics.fy) ||
      buffered.intrinsics.fx <= 0.0 || buffered.intrinsics.fy <= 0.0) {
    LOG(ERROR) << "Skipping mono depth inference because camera intrinsics are "
                  "invalid.";
    return std::nullopt;
  }
  buffered.body_T_cam = frame.cam_param_.body_Pose_cam_;
  buffered.keypoints = frame.keypoints_;
  buffered.landmark_ids = frame.landmarks_;
  return buffered;
}

xfeat::CameraIntrinsics MonoDepthInference::toXfeatIntrinsics(
    const MonoDepthIntrinsics& intrinsics) {
  xfeat::CameraIntrinsics converted;
  converted.fx = intrinsics.fx;
  converted.fy = intrinsics.fy;
  converted.cx = intrinsics.cx;
  converted.cy = intrinsics.cy;
  converted.width = intrinsics.width;
  converted.height = intrinsics.height;
  return converted;
}

MonoDepthConfidenceFilterResult makeMonoDepthConfidenceFilter(
    const cv::Size& expected_size,
    const cv::Mat& confidence,
    const double min_confidence) {
  if (!std::isfinite(min_confidence) || min_confidence < 0.0) {
    throw std::invalid_argument(
        "Mono-depth confidence threshold must be finite and non-negative");
  }

  MonoDepthConfidenceFilterResult result;
  result.filtering_enabled = min_confidence > 0.0;
  if (expected_size.width <= 0 || expected_size.height <= 0) {
    result.retained_fraction = 0.0;
    result.error = "expected confidence size is empty";
    return result;
  }

  const std::size_t pixel_count =
      static_cast<std::size_t>(expected_size.area());
  result.mask = cv::Mat(expected_size, CV_8UC1, cv::Scalar(0));
  result.confidence_valid =
      !confidence.empty() && confidence.type() == CV_32FC1 &&
      confidence.size() == expected_size;

  if (!result.filtering_enabled) {
    result.mask.setTo(255u);
    result.accepted_pixels = pixel_count;
    result.retained_fraction = 1.0;
    return result;
  }

  if (!result.confidence_valid) {
    result.rejected_pixels = pixel_count;
    result.retained_fraction = 0.0;
    if (confidence.empty()) {
      result.error = "confidence map is missing";
    } else if (confidence.type() != CV_32FC1) {
      result.error = "confidence map is not CV_32FC1";
    } else {
      result.error = "confidence map shape does not match depth";
    }
    return result;
  }

  const float threshold = static_cast<float>(min_confidence);
  for (int v = 0; v < confidence.rows; ++v) {
    const float* confidence_row = confidence.ptr<float>(v);
    uint8_t* mask_row = result.mask.ptr<uint8_t>(v);
    for (int u = 0; u < confidence.cols; ++u) {
      const float value = confidence_row[u];
      if (std::isfinite(value) && value >= threshold) {
        mask_row[u] = 255u;
        ++result.accepted_pixels;
      }
    }
  }
  result.rejected_pixels = pixel_count - result.accepted_pixels;
  result.retained_fraction =
      static_cast<double>(result.accepted_pixels) /
      static_cast<double>(pixel_count);
  return result;
}

cv::Mat makeMonoDepthValidMask(const cv::Mat& depth,
                               const cv::Mat& sky_mask,
                               const cv::Mat& confidence_mask) {
  CHECK(!depth.empty());
  CHECK_EQ(depth.type(), CV_32FC1);

  cv::Mat valid_mask(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  const bool has_sky_mask =
      !sky_mask.empty() && sky_mask.type() == CV_8UC1 &&
      sky_mask.rows >= depth.rows && sky_mask.cols >= depth.cols;
  const bool use_confidence_mask = !confidence_mask.empty();
  const bool has_confidence_mask =
      use_confidence_mask && confidence_mask.type() == CV_8UC1 &&
      confidence_mask.size() == depth.size();
  if (use_confidence_mask && !has_confidence_mask) {
    LOG(ERROR) << "Rejecting all mono-depth pixels because the confidence "
                  "mask is malformed or shape-incompatible.";
    return valid_mask;
  }

  for (int v = 0; v < depth.rows; ++v) {
    const float* depth_row = depth.ptr<float>(v);
    const uint8_t* sky_row = has_sky_mask ? sky_mask.ptr<uint8_t>(v) : nullptr;
    const uint8_t* confidence_row =
        has_confidence_mask ? confidence_mask.ptr<uint8_t>(v) : nullptr;
    uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
    for (int u = 0; u < depth.cols; ++u) {
      const float z = depth_row[u];
      const bool sky = sky_row != nullptr && sky_row[u] != 0u;
      const bool confidence_valid =
          confidence_row == nullptr || confidence_row[u] != 0u;
      if (std::isfinite(z) && z > 0.0f && !sky && confidence_valid) {
        valid_row[u] = 255u;
      }
    }
  }
  return valid_mask;
}

MonoDepthRawPacket::ConstPtr MonoDepthInference::buildPacket(
    const BufferedKeyframe& frame,
    const xfeat::MonoDepthResult& depth_result,
    const bool apply_confidence_filter) const {
  if (depth_result.depth.empty() || depth_result.depth.type() != CV_32FC1) {
    LOG(ERROR) << "DA3 mono depth returned an empty or non-CV_32FC1 depth map "
               << "for keyframe " << frame.keyframe_id;
    return nullptr;
  }

  auto packet = std::make_shared<MonoDepthRawPacket>();
  packet->keyframe_id = frame.keyframe_id;
  packet->timestamp = frame.timestamp;
  packet->source_image_bgr = frame.image_bgr.clone();
  packet->depth = depth_result.depth.clone();
  packet->intrinsics = frame.intrinsics;
  packet->body_T_cam = frame.body_T_cam;
  packet->keypoints = frame.keypoints;
  packet->landmark_ids = frame.landmark_ids;
  packet->metadata = depth_result.metadata;
  if (apply_confidence_filter) {
    packet->metadata.view_index = 1;
    packet->metadata.view_count = 2;
  }

  const double threshold =
      apply_confidence_filter ? params_.min_confidence : 0.0;
  const MonoDepthConfidenceFilterResult confidence_filter =
      makeMonoDepthConfidenceFilter(
          packet->depth.size(), depth_result.confidence, threshold);
  packet->confidence_filtering_enabled =
      confidence_filter.filtering_enabled;
  packet->confidence_valid = confidence_filter.confidence_valid;
  packet->confidence_threshold = threshold;
  packet->confidence_accepted_pixels = confidence_filter.accepted_pixels;
  packet->confidence_rejected_pixels = confidence_filter.rejected_pixels;
  packet->confidence_retained_fraction = confidence_filter.retained_fraction;
  packet->confidence_error = confidence_filter.error;

  if (confidence_filter.filtering_enabled &&
      !confidence_filter.confidence_valid) {
    LOG(ERROR) << "Rejecting all mono-depth pixels for keyframe "
               << frame.keyframe_id << ": " << confidence_filter.error;
  }

  packet->valid_mask = makeMonoDepthValidMask(
      packet->depth, depth_result.sky_mask, confidence_filter.mask);
  packet->weight_image =
      makeWeightImage(packet->depth, packet->valid_mask, packet->intrinsics);

  packet->confidence_visualization_enabled =
      apply_confidence_filter && params_.visualize_confidence;
  if (packet->confidence_visualization_enabled) {
    if (confidence_filter.confidence_valid) {
      packet->confidence = depth_result.confidence.clone();
    }
    packet->confidence_mask = confidence_filter.mask.clone();
  }
  return packet;
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
