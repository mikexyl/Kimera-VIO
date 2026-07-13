#include "kimera-vio/frontend/MonoDepthInference.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "kimera-vio/common/MonoDepthUtils.h"

#ifdef HAVE_TENSORRT
#include <xfeat-cpp/mono_depth/depth_anything_v3_trt.h>
#endif

namespace VIO {

MonoDepthPairDistanceGateResult evaluateMonoDepthPairDistanceGate(
    const gtsam::Pose3& odometry_world_T_context_cam,
    const gtsam::Pose3& odometry_world_T_current_cam,
    const double min_keyframe_distance_m) {
  MonoDepthPairDistanceGateResult result;
  if (!std::isfinite(min_keyframe_distance_m) ||
      min_keyframe_distance_m < 0.0) {
    result.error = "minimum keyframe distance must be finite and non-negative";
    return result;
  }

  result.camera_displacement_m =
      odometry_world_T_context_cam.between(odometry_world_T_current_cam)
          .translation()
          .norm();
  if (!std::isfinite(result.camera_displacement_m)) {
    result.error = "odometry camera-center displacement is not finite";
    return result;
  }
  result.valid = true;
  const double comparison_tolerance =
      1e-9 * std::max(1.0, min_keyframe_distance_m);
  result.passes = result.camera_displacement_m + comparison_tolerance >=
                  min_keyframe_distance_m;
  return result;
}

MonoDepthRectifiedFeatures makeMonoDepthRectifiedFeatures(
    const StatusKeypointsCV& undistorted_keypoints,
    const LandmarkIds& landmark_ids) {
  MonoDepthRectifiedFeatures result;
  if (undistorted_keypoints.size() != landmark_ids.size()) {
    result.error = "undistorted keypoint and landmark-id counts differ";
    return result;
  }

  result.keypoints.reserve(undistorted_keypoints.size());
  result.landmark_ids = landmark_ids;
  for (std::size_t i = 0u; i < undistorted_keypoints.size(); ++i) {
    const StatusKeypointCV& status_keypoint = undistorted_keypoints[i];
    result.keypoints.push_back(status_keypoint.second);
    if (status_keypoint.first != KeypointStatus::VALID) {
      if (result.landmark_ids[i] != -1) {
        ++result.rejected_keypoints;
      }
      result.landmark_ids[i] = -1;
    }
  }
  result.valid = true;
  return result;
}

MonoDepthInference::MonoDepthInference(const MonoDepthParams& params,
                                       const CameraParams& camera_params)
    : params_(params),
      camera_params_(camera_params),
      image_undistorter_(nullptr),
      mono_depth_(nullptr),
      buffered_keyframe_(std::nullopt) {
  validateMonoDepthScaleAlignmentConfiguration(params_.mode,
                                               params_.scale_alignment_method);
  if (!std::isfinite(params_.min_confidence) || params_.min_confidence < 0.0) {
    LOG(FATAL) << "mono_depth.min_confidence must be finite and non-negative, "
               << "but got " << params_.min_confidence;
  }
  if (!std::isfinite(params_.min_keyframe_distance_m) ||
      params_.min_keyframe_distance_m < 0.0) {
    LOG(FATAL) << "mono_depth.min_keyframe_distance_m must be finite and "
                  "non-negative, but got "
               << params_.min_keyframe_distance_m;
  }
  if (!params_.enabled) {
    return;
  }

  if (camera_params_.camera_model_ != CameraModel::PINHOLE) {
    LOG(FATAL) << "Mono-depth image undistortion currently requires a "
                  "pinhole camera model.";
  }
  if (camera_params_.K_.empty() || camera_params_.image_size_.width <= 0 ||
      camera_params_.image_size_.height <= 0) {
    LOG(FATAL) << "Mono-depth image undistortion requires a valid camera "
                  "matrix and image size.";
  }
  const cv::Mat rectification_rotation =
      cv::Mat::eye(3, 3, camera_params_.K_.type());
  image_undistorter_ = std::make_unique<UndistorterRectifier>(
      camera_params_.K_, camera_params_, rectification_rotation);
  CHECK(image_undistorter_);

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
            << params_.engine_path
            << ", mode=" << monoDepthModeToString(params_.mode)
            << ", scale_alignment_method="
            << monoDepthScaleAlignmentMethodToString(
                   params_.scale_alignment_method)
            << ", min_confidence=" << params_.min_confidence
            << ", min_keyframe_distance_m=" << params_.min_keyframe_distance_m
            << ", input_geometry=undistorted_pinhole";
#else
  LOG(FATAL) << "Mono depth inference requires xfeat-cpp TensorRT support, "
                "but HAVE_TENSORRT is not enabled.";
#endif
}

MonoDepthRawPacket::ConstPtr MonoDepthInference::inferKeyframe(
    const Frame& frame,
    const std::optional<gtsam::Pose3>& odometry_world_T_body) const {
  if (!params_.enabled) {
    return nullptr;
  }
  CHECK(mono_depth_);
  CHECK(frame.isKeyframe_);
  CHECK(frame.keyframe_id_.has_value());

  if (params_.mode == MonoDepthMode::kMultiView &&
      !odometry_world_T_body.has_value()) {
    LOG_EVERY_N(WARNING, 30)
        << "Holding DA3 two-view inference at keyframe " << *frame.keyframe_id_
        << " because a metric odometry pose is unavailable.";
    return nullptr;
  }

  const std::optional<BufferedKeyframe> current =
      bufferKeyframe(frame, odometry_world_T_body);
  if (!current.has_value()) {
    return nullptr;
  }

  if (params_.mode == MonoDepthMode::kSingleView) {
    xfeat::MonoDepthResult depth_result;
    try {
      depth_result =
          mono_depth_->infer(current->image_bgr,
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

  CHECK(buffered_keyframe_->odometry_world_T_body.has_value());
  CHECK(current->odometry_world_T_body.has_value());
  const gtsam::Pose3 odometry_world_T_context_cam =
      buffered_keyframe_->odometry_world_T_body->compose(
          buffered_keyframe_->body_T_cam);
  const gtsam::Pose3 odometry_world_T_current_cam =
      current->odometry_world_T_body->compose(current->body_T_cam);
  const MonoDepthPairDistanceGateResult distance_gate =
      evaluateMonoDepthPairDistanceGate(odometry_world_T_context_cam,
                                        odometry_world_T_current_cam,
                                        params_.min_keyframe_distance_m);
  if (!distance_gate.valid) {
    LOG(ERROR) << "Cannot evaluate DA3 two-view distance gate for pair ["
               << buffered_keyframe_->keyframe_id << ", "
               << current->keyframe_id << "]: " << distance_gate.error;
    return nullptr;
  }
  if (!distance_gate.passes) {
    VLOG(1) << "Holding DA3 two-view context keyframe "
            << buffered_keyframe_->keyframe_id << "; candidate keyframe "
            << current->keyframe_id << " has endpoint camera displacement "
            << distance_gate.camera_displacement_m << " m < "
            << params_.min_keyframe_distance_m << " m.";
    return nullptr;
  }

  BufferedKeyframe previous = std::move(*buffered_keyframe_);
  // Advance before inference so a failed pair does not get retried and the
  // next distance-gated pair uses the newest accepted context view.
  buffered_keyframe_ = current;

  LOG(INFO) << "Running pose-free DA3 two-view pair [" << previous.keyframe_id
            << ", " << current->keyframe_id
            << "] with odometry endpoint camera displacement "
            << distance_gate.camera_displacement_m << " m (threshold "
            << params_.min_keyframe_distance_m << " m)";
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

  Da3PairInfo pair_info;
  pair_info.context_keyframe_id = previous.keyframe_id;
  pair_info.context_body_T_cam = previous.body_T_cam;
  pair_info.context_cam_T_current_cam =
      makeDa3ContextToCurrentPose(depth_results[0], depth_results[1]);
  if (!pair_info.context_cam_T_current_cam.has_value()) {
    LOG(ERROR) << "DA3 pair [" << previous.keyframe_id << ", "
               << current->keyframe_id
               << "] did not return usable predicted camera poses; backend "
                  "depth scaling will fail closed.";
  }

  MonoDepthRawPacket::ConstPtr packet =
      buildPacket(*current, depth_results[1], true, pair_info);
  if (packet) {
    LOG(INFO) << "DA3 pair [" << previous.keyframe_id << ", "
              << current->keyframe_id << "] emitted keyframe "
              << packet->keyframe_id
              << " (view_index=" << packet->metadata.view_index
              << ", view_count=" << packet->metadata.view_count
              << ", confidence_threshold=" << packet->confidence_threshold
              << ", accepted=" << packet->confidence_accepted_pixels
              << ", rejected=" << packet->confidence_rejected_pixels
              << ", retained=" << packet->confidence_retained_fraction
              << ", undistortion_valid="
              << packet->image_geometry_valid_pixels
              << ", undistortion_rejected="
              << packet->image_geometry_rejected_pixels << ")";
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
MonoDepthInference::bufferKeyframe(
    const Frame& frame,
    const std::optional<gtsam::Pose3>& odometry_world_T_body) const {
  const cv::Mat bgr_image = toBgrImage(frame.img_);
  if (bgr_image.empty()) {
    LOG(ERROR) << "Skipping mono depth inference for an empty or unsupported "
                  "keyframe image.";
    return std::nullopt;
  }
  if (bgr_image.size() != camera_params_.image_size_) {
    LOG(ERROR) << "Skipping mono depth inference because keyframe image size "
               << bgr_image.cols << "x" << bgr_image.rows
               << " differs from the calibrated size "
               << camera_params_.image_size_.width << "x"
               << camera_params_.image_size_.height << ".";
    return std::nullopt;
  }
  CHECK(image_undistorter_);

  BufferedKeyframe buffered;
  buffered.keyframe_id = *frame.keyframe_id_;
  buffered.timestamp = frame.timestamp_;
  image_undistorter_->undistortRectifyImage(
      bgr_image, &buffered.image_bgr, &buffered.image_geometry_mask);
  if (buffered.image_bgr.empty() || buffered.image_geometry_mask.empty()) {
    LOG(ERROR) << "Skipping mono depth inference because image undistortion "
                  "failed.";
    return std::nullopt;
  }
  buffered.intrinsics.fx = camera_params_.intrinsics_[0];
  buffered.intrinsics.fy = camera_params_.intrinsics_[1];
  buffered.intrinsics.cx = camera_params_.intrinsics_[2];
  buffered.intrinsics.cy = camera_params_.intrinsics_[3];
  buffered.intrinsics.width = bgr_image.cols;
  buffered.intrinsics.height = bgr_image.rows;
  if (!std::isfinite(buffered.intrinsics.fx) ||
      !std::isfinite(buffered.intrinsics.fy) || buffered.intrinsics.fx <= 0.0 ||
      buffered.intrinsics.fy <= 0.0) {
    LOG(ERROR) << "Skipping mono depth inference because camera intrinsics are "
                  "invalid.";
    return std::nullopt;
  }
  buffered.body_T_cam = frame.cam_param_.body_Pose_cam_;
  buffered.odometry_world_T_body = odometry_world_T_body;
  const MonoDepthRectifiedFeatures rectified_features =
      makeMonoDepthRectifiedFeatures(frame.keypoints_undistorted_,
                                     frame.landmarks_);
  if (!rectified_features.valid) {
    LOG(ERROR) << "Skipping mono depth inference for keyframe "
               << buffered.keyframe_id << ": " << rectified_features.error;
    return std::nullopt;
  }
  buffered.keypoints = rectified_features.keypoints;
  buffered.landmark_ids = rectified_features.landmark_ids;
  VLOG(1) << "Prepared undistorted mono-depth keyframe " << buffered.keyframe_id
          << ": geometric_valid_pixels="
          << cv::countNonZero(buffered.image_geometry_mask) << "/"
          << buffered.image_geometry_mask.total()
          << ", rejected_rectified_landmarks="
          << rectified_features.rejected_keypoints;
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

std::optional<gtsam::Pose3> MonoDepthInference::makeDa3ContextToCurrentPose(
    const xfeat::MonoDepthResult& context_result,
    const xfeat::MonoDepthResult& current_result) {
  const auto to_pose = [](const std::optional<cv::Matx44f>& extrinsic)
      -> std::optional<gtsam::Pose3> {
    if (!extrinsic.has_value()) {
      return std::nullopt;
    }
    gtsam::Matrix3 rotation;
    gtsam::Point3 translation;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const double value = static_cast<double>((*extrinsic)(row, col));
        if (!std::isfinite(value)) {
          return std::nullopt;
        }
        rotation(row, col) = value;
      }
      const double value = static_cast<double>((*extrinsic)(row, 3));
      if (!std::isfinite(value)) {
        return std::nullopt;
      }
      translation(row) = value;
    }
    return gtsam::Pose3(gtsam::Rot3::ClosestTo(rotation), translation);
  };

  const std::optional<gtsam::Pose3> context_cam_T_world =
      to_pose(context_result.predicted_world_to_camera);
  const std::optional<gtsam::Pose3> current_cam_T_world =
      to_pose(current_result.predicted_world_to_camera);
  if (!context_cam_T_world.has_value() || !current_cam_T_world.has_value()) {
    return std::nullopt;
  }
  return context_cam_T_world->compose(current_cam_T_world->inverse());
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
  result.confidence_valid = !confidence.empty() &&
                            confidence.type() == CV_32FC1 &&
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
  result.retained_fraction = static_cast<double>(result.accepted_pixels) /
                             static_cast<double>(pixel_count);
  return result;
}

cv::Mat makeMonoDepthValidMask(const cv::Mat& depth,
                               const cv::Mat& sky_mask,
                               const cv::Mat& confidence_mask,
                               const cv::Mat& image_geometry_mask) {
  CHECK(!depth.empty());
  CHECK_EQ(depth.type(), CV_32FC1);

  cv::Mat valid_mask(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  const bool has_sky_mask = !sky_mask.empty() && sky_mask.type() == CV_8UC1 &&
                            sky_mask.rows >= depth.rows &&
                            sky_mask.cols >= depth.cols;
  const bool use_confidence_mask = !confidence_mask.empty();
  const bool has_confidence_mask = use_confidence_mask &&
                                   confidence_mask.type() == CV_8UC1 &&
                                   confidence_mask.size() == depth.size();
  if (use_confidence_mask && !has_confidence_mask) {
    LOG(ERROR) << "Rejecting all mono-depth pixels because the confidence "
                  "mask is malformed or shape-incompatible.";
    return valid_mask;
  }
  const bool use_image_geometry_mask = !image_geometry_mask.empty();
  const bool has_image_geometry_mask =
      use_image_geometry_mask && image_geometry_mask.type() == CV_8UC1 &&
      image_geometry_mask.size() == depth.size();
  if (use_image_geometry_mask && !has_image_geometry_mask) {
    LOG(ERROR) << "Rejecting all mono-depth pixels because the undistorted "
                  "image geometry mask is malformed or shape-incompatible.";
    return valid_mask;
  }

  for (int v = 0; v < depth.rows; ++v) {
    const float* depth_row = depth.ptr<float>(v);
    const uint8_t* sky_row = has_sky_mask ? sky_mask.ptr<uint8_t>(v) : nullptr;
    const uint8_t* confidence_row =
        has_confidence_mask ? confidence_mask.ptr<uint8_t>(v) : nullptr;
    const uint8_t* geometry_row =
        has_image_geometry_mask ? image_geometry_mask.ptr<uint8_t>(v) : nullptr;
    uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
    for (int u = 0; u < depth.cols; ++u) {
      const float z = depth_row[u];
      const bool sky = sky_row != nullptr && sky_row[u] != 0u;
      const bool confidence_valid =
          confidence_row == nullptr || confidence_row[u] != 0u;
      const bool geometry_valid =
          geometry_row == nullptr || geometry_row[u] != 0u;
      if (std::isfinite(z) && z > 0.0f && !sky && confidence_valid &&
          geometry_valid) {
        valid_row[u] = 255u;
      }
    }
  }
  return valid_mask;
}

MonoDepthRawPacket::ConstPtr MonoDepthInference::buildPacket(
    const BufferedKeyframe& frame,
    const xfeat::MonoDepthResult& depth_result,
    const bool apply_confidence_filter,
    const std::optional<Da3PairInfo>& pair_info) const {
  if (depth_result.depth.empty() || depth_result.depth.type() != CV_32FC1) {
    LOG(ERROR) << "DA3 mono depth returned an empty or non-CV_32FC1 depth map "
               << "for keyframe " << frame.keyframe_id;
    return nullptr;
  }
  if (depth_result.depth.size() != frame.image_bgr.size() ||
      frame.image_geometry_mask.type() != CV_8UC1 ||
      frame.image_geometry_mask.size() != frame.image_bgr.size()) {
    LOG(ERROR) << "DA3 mono depth geometry mismatch for keyframe "
               << frame.keyframe_id << ": image=" << frame.image_bgr.cols << "x"
               << frame.image_bgr.rows << ", depth=" << depth_result.depth.cols
               << "x" << depth_result.depth.rows
               << ", geometry_mask=" << frame.image_geometry_mask.cols << "x"
               << frame.image_geometry_mask.rows;
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
    if (pair_info.has_value()) {
      packet->da3_context_keyframe_id = pair_info->context_keyframe_id;
      packet->da3_context_body_T_cam = pair_info->context_body_T_cam;
      packet->da3_context_cam_T_current_cam =
          pair_info->context_cam_T_current_cam;
    }
  }

  const double threshold =
      apply_confidence_filter ? params_.min_confidence : 0.0;
  const MonoDepthConfidenceFilterResult confidence_filter =
      makeMonoDepthConfidenceFilter(
          packet->depth.size(), depth_result.confidence, threshold);
  packet->confidence_filtering_enabled = confidence_filter.filtering_enabled;
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

  packet->source_image_is_undistorted = true;
  packet->image_geometry_valid_pixels =
      static_cast<std::size_t>(cv::countNonZero(frame.image_geometry_mask));
  packet->image_geometry_rejected_pixels =
      frame.image_geometry_mask.total() - packet->image_geometry_valid_pixels;
  packet->valid_mask = makeMonoDepthValidMask(packet->depth,
                                              depth_result.sky_mask,
                                              confidence_filter.mask,
                                              frame.image_geometry_mask);
  const cv::Size weight_size =
      depth_result.metadata.model_size.width > 0 &&
              depth_result.metadata.model_size.height > 0
          ? depth_result.metadata.model_size
          : packet->depth.size();
  packet->weight_image = makeMonoDepthWeightImageAtSize(packet->depth,
                                                        packet->valid_mask,
                                                        packet->intrinsics,
                                                        weight_size,
                                                        params_);
  VLOG(1) << "Computed mono-depth weights once for keyframe "
          << packet->keyframe_id << " on " << packet->weight_image.cols << "x"
          << packet->weight_image.rows << " grid (depth " << packet->depth.cols
          << "x" << packet->depth.rows << ").";

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

}  // namespace VIO
