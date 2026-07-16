#pragma once

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <xfeat-cpp/mono_depth/mono_depth.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <map>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class MonoDepthMode { kSingleView = 0, kMultiView = 1 };

enum class Da3KeyframeSelectionMethod {
  kDistance = 0,
  kFixedSkip = 1,
  kCovisibility = 2,
};

inline std::string da3KeyframeSelectionMethodToString(
    const Da3KeyframeSelectionMethod method) {
  switch (method) {
    case Da3KeyframeSelectionMethod::kDistance:
      return "distance";
    case Da3KeyframeSelectionMethod::kFixedSkip:
      return "fixed_skip";
    case Da3KeyframeSelectionMethod::kCovisibility:
      return "covisibility";
  }
  throw std::invalid_argument("Unknown DA3 keyframe selection method: " +
                              std::to_string(static_cast<int>(method)));
}

inline Da3KeyframeSelectionMethod da3KeyframeSelectionMethodFromString(
    std::string method) {
  std::transform(
      method.begin(), method.end(), method.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
  std::replace(method.begin(), method.end(), '-', '_');
  if (method == "distance") {
    return Da3KeyframeSelectionMethod::kDistance;
  }
  if (method == "fixed_skip" || method == "skip") {
    return Da3KeyframeSelectionMethod::kFixedSkip;
  }
  if (method == "covisibility" || method == "covis") {
    return Da3KeyframeSelectionMethod::kCovisibility;
  }
  throw std::invalid_argument(
      "Unsupported mono_depth.da3_keyframe_selection_method: " + method +
      ". Expected one of: distance, fixed_skip, covisibility.");
}

enum class MonoDepthScaleAlignmentMethod {
  kNone = 0,
  kRelativePose = 1,
  kLandmarks = 2,
};

inline std::string monoDepthScaleAlignmentMethodToString(
    const MonoDepthScaleAlignmentMethod method) {
  switch (method) {
    case MonoDepthScaleAlignmentMethod::kNone:
      return "none";
    case MonoDepthScaleAlignmentMethod::kRelativePose:
      return "relative_pose";
    case MonoDepthScaleAlignmentMethod::kLandmarks:
      return "landmarks";
  }
  throw std::invalid_argument("Unknown mono-depth scale alignment method: " +
                              std::to_string(static_cast<int>(method)));
}

inline MonoDepthScaleAlignmentMethod monoDepthScaleAlignmentMethodFromString(
    const std::string& method) {
  if (method == "none") {
    return MonoDepthScaleAlignmentMethod::kNone;
  }
  if (method == "relative_pose") {
    return MonoDepthScaleAlignmentMethod::kRelativePose;
  }
  if (method == "landmarks") {
    return MonoDepthScaleAlignmentMethod::kLandmarks;
  }
  throw std::invalid_argument(
      "Unsupported mono_depth.scale_alignment_method: " + method +
      ". Expected exactly one of: none, relative_pose, landmarks.");
}

inline void validateMonoDepthScaleAlignmentConfiguration(
    const MonoDepthMode mode,
    const MonoDepthScaleAlignmentMethod method) {
  if (mode == MonoDepthMode::kSingleView &&
      method == MonoDepthScaleAlignmentMethod::kRelativePose) {
    throw std::invalid_argument(
        "mono_depth.scale_alignment_method=relative_pose requires "
        "mono_depth.mode=multi_view");
  }
}

enum class MonoDepthLandmarkScaleSampleStatus {
  kFlatInlier = 0,
  kDepthEdgeRejected = 1,
  kFlatnessSupportRejected = 2,
  kLogRatioOutlier = 3,
};

struct MonoDepthLandmarkScaleSample {
  LandmarkId landmark_id = -1;
  cv::Point2f keypoint;
  double relative_depth_variation = -1.0;
  double flatness_weight = 0.0;
  double log_depth_ratio = 0.0;
  MonoDepthLandmarkScaleSampleStatus status =
      MonoDepthLandmarkScaleSampleStatus::kFlatInlier;

  bool operator==(const MonoDepthLandmarkScaleSample& rhs) const {
    return landmark_id == rhs.landmark_id && keypoint.x == rhs.keypoint.x &&
           keypoint.y == rhs.keypoint.y &&
           relative_depth_variation == rhs.relative_depth_variation &&
           flatness_weight == rhs.flatness_weight &&
           log_depth_ratio == rhs.log_depth_ratio && status == rhs.status;
  }
};

struct MonoDepthScaleAlignmentResult {
  MonoDepthScaleAlignmentMethod method = MonoDepthScaleAlignmentMethod::kNone;
  bool valid = false;
  double absolute_scale = 1.0;
  std::string failure_reason;
  std::size_t candidate_count = 0u;
  std::size_t inlier_count = 0u;
  double log_rmse = 0.0;
  std::map<std::string, double> metrics;
  std::vector<MonoDepthLandmarkScaleSample> landmark_samples;

  bool operator==(const MonoDepthScaleAlignmentResult& rhs) const {
    return method == rhs.method && valid == rhs.valid &&
           absolute_scale == rhs.absolute_scale &&
           failure_reason == rhs.failure_reason &&
           candidate_count == rhs.candidate_count &&
           inlier_count == rhs.inlier_count && log_rmse == rhs.log_rmse &&
           metrics == rhs.metrics && landmark_samples == rhs.landmark_samples;
  }

  bool operator!=(const MonoDepthScaleAlignmentResult& rhs) const {
    return !(*this == rhs);
  }
};

inline std::string monoDepthModeToString(const MonoDepthMode mode) {
  switch (mode) {
    case MonoDepthMode::kSingleView:
      return "single_view";
    case MonoDepthMode::kMultiView:
      return "multi_view";
    default:
      LOG(FATAL) << "Unknown mono depth mode: " << static_cast<int>(mode);
  }
  return "single_view";
}

inline MonoDepthMode monoDepthModeFromString(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode == "single_view" || mode == "single-view" || mode == "single") {
    return MonoDepthMode::kSingleView;
  }
  if (mode == "multi_view" || mode == "multi-view" || mode == "multi") {
    return MonoDepthMode::kMultiView;
  }
  LOG(FATAL) << "Unsupported mono_depth.mode: " << mode
             << ". Expected single_view or multi_view.";
  return MonoDepthMode::kSingleView;
}

struct MonoDepthParams {
  bool enabled = false;
  std::string engine_path;
  Da3KeyframeSelectionMethod da3_keyframe_selection_method =
      Da3KeyframeSelectionMethod::kDistance;
  // Number of incoming VIO keyframes to hold between selected DA3 endpoints.
  // Zero selects consecutive VIO keyframes.
  int da3_keyframe_skip = 0;
  double min_keyframe_distance_m = 1.0;
  // Run two-view DA3 when the fraction of the buffered reference frame's
  // valid feature tracks still observed by the candidate falls below this.
  double da3_keyframe_covisibility_threshold = 0.5;
  int point_stride = 4;
  int max_points_per_keyframe = 5000;
  double min_depth_m = 0.1;
  double max_depth_m = 30.0;
  bool depth_weighting_enabled = true;
  int depth_weight_normal_radius = 2;
  double depth_weight_min = 0.05;
  double depth_weight_grazing_power = 1.0;
  double depth_weight_range_ref = 0.0;
  double depth_weight_range_power = 2.0;
  double depth_weight_range_min = 0.05;
  // Experimental five-DoF DA3 camera-motion constraint in the VIO smoother.
  // This consumes only the canonical two-view pose metadata, independently of
  // depth scaling, confidence filtering, and ICP.
  bool da3_essential_factors_enabled = false;
  // Experimental scale-free constraint over consecutive DA3 pairs.  The
  // measured baseline ratio comes only from canonical shared-image depth and
  // DA3 translations; it supplies no absolute distance.
  bool da3_baseline_ratio_factors_enabled = false;
  double da3_baseline_ratio_log_sigma = 0.25;
  double min_confidence = 1.1;
  bool visualize_confidence = false;
  bool verbose = false;
  MonoDepthScaleAlignmentMethod scale_alignment_method =
      MonoDepthScaleAlignmentMethod::kNone;
  // Landmark-scale samples are weighted by local depth flatness. The
  // variation is scale-invariant: max(neighbor, center) /
  // min(neighbor, center) - 1. Samples at or above the cutoff are rejected.
  int landmark_scale_flatness_radius = 4;
  double landmark_scale_max_relative_depth_variation = 0.15;
  MonoDepthMode mode = MonoDepthMode::kSingleView;

  bool operator==(const MonoDepthParams& rhs) const {
    return enabled == rhs.enabled && engine_path == rhs.engine_path &&
           da3_keyframe_selection_method == rhs.da3_keyframe_selection_method &&
           da3_keyframe_skip == rhs.da3_keyframe_skip &&
           min_keyframe_distance_m == rhs.min_keyframe_distance_m &&
           da3_keyframe_covisibility_threshold ==
               rhs.da3_keyframe_covisibility_threshold &&
           point_stride == rhs.point_stride &&
           max_points_per_keyframe == rhs.max_points_per_keyframe &&
           min_depth_m == rhs.min_depth_m && max_depth_m == rhs.max_depth_m &&
           depth_weighting_enabled == rhs.depth_weighting_enabled &&
           depth_weight_normal_radius == rhs.depth_weight_normal_radius &&
           depth_weight_min == rhs.depth_weight_min &&
           depth_weight_grazing_power == rhs.depth_weight_grazing_power &&
           depth_weight_range_ref == rhs.depth_weight_range_ref &&
           depth_weight_range_power == rhs.depth_weight_range_power &&
           depth_weight_range_min == rhs.depth_weight_range_min &&
           da3_essential_factors_enabled == rhs.da3_essential_factors_enabled &&
           da3_baseline_ratio_factors_enabled ==
               rhs.da3_baseline_ratio_factors_enabled &&
           da3_baseline_ratio_log_sigma == rhs.da3_baseline_ratio_log_sigma &&
           min_confidence == rhs.min_confidence &&
           visualize_confidence == rhs.visualize_confidence &&
           verbose == rhs.verbose &&
           scale_alignment_method == rhs.scale_alignment_method &&
           landmark_scale_flatness_radius ==
               rhs.landmark_scale_flatness_radius &&
           landmark_scale_max_relative_depth_variation ==
               rhs.landmark_scale_max_relative_depth_variation &&
           mode == rhs.mode;
  }
};

struct MonoDepthIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  int width = 0;
  int height = 0;
};

struct MonoDepthRawPacket {
  KIMERA_POINTER_TYPEDEFS(MonoDepthRawPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameId keyframe_id = 0u;
  Timestamp timestamp = 0;
  cv::Mat source_image_bgr;
  cv::Mat depth;
  // Sky and undistortion filtering only.  Unlike valid_mask, this mask does
  // not apply the per-run confidence threshold, so two predictions of the
  // same DA3 view can still be compared when their confidence differs.
  cv::Mat depth_support_mask;
  cv::Mat valid_mask;
  cv::Mat weight_image;
  cv::Mat confidence;
  cv::Mat confidence_mask;
  bool source_image_is_undistorted = false;
  std::size_t image_geometry_valid_pixels = 0u;
  std::size_t image_geometry_rejected_pixels = 0u;
  bool confidence_visualization_enabled = false;
  bool confidence_filtering_enabled = false;
  bool confidence_valid = false;
  double confidence_threshold = 0.0;
  std::size_t confidence_accepted_pixels = 0u;
  std::size_t confidence_rejected_pixels = 0u;
  double confidence_retained_fraction = 1.0;
  std::string confidence_error;
  std::optional<FrameId> da3_context_keyframe_id;
  std::optional<gtsam::Pose3> da3_context_body_T_cam;
  std::optional<gtsam::Pose3> da3_context_cam_T_current_cam;
  // The first view returned by the same two-view DA3 invocation.  The packet
  // itself is the second view.  Keeping both views exposes the duplicate
  // middle image shared by consecutive invocations.
  MonoDepthRawPacket::ConstPtr da3_context_packet;
  MonoDepthScaleAlignmentResult scale_alignment;
  MonoDepthIntrinsics intrinsics;
  gtsam::Pose3 body_T_cam;
  KeypointsCV keypoints;
  LandmarkIds landmark_ids;
  xfeat::MonoDepthMetadata metadata;
};

}  // namespace VIO
