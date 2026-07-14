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
  double min_keyframe_distance_m = 1.0;
  int point_stride = 4;
  int max_points_per_keyframe = 5000;
  int visualization_point_stride = 4;
  int visualization_max_points_per_keyframe = 5000;
  double min_depth_m = 0.1;
  double max_depth_m = 30.0;
  bool depth_weighting_enabled = true;
  int depth_weight_normal_radius = 2;
  double depth_weight_min = 0.05;
  double depth_weight_grazing_power = 1.0;
  double depth_weight_range_ref = 0.0;
  double depth_weight_range_power = 2.0;
  double depth_weight_range_min = 0.05;
  bool visualize_weights = false;
  bool visualize_landmark_scale_alignment = false;
  // Experimental five-DoF DA3 camera-motion constraint in the VIO smoother.
  // This consumes only the canonical two-view pose metadata, independently of
  // depth scaling, confidence filtering, dense mapping, and ICP.
  bool da3_essential_factors_enabled = false;
  // Experimental isolated diagnostic: run DA3 on consecutive keyframes and
  // chain pair reconstructions through their duplicate middle image. No
  // odometry pose is used by DA3 overlap alignment or fusion; metric odometry
  // is consulted only to apply min_keyframe_distance_m consistently.
  bool icp_only_da3_overlap_fusion = false;
  double min_confidence = 1.1;
  bool visualize_confidence = false;
  float point_radius = 0.005f;
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
           min_keyframe_distance_m == rhs.min_keyframe_distance_m &&
           point_stride == rhs.point_stride &&
           max_points_per_keyframe == rhs.max_points_per_keyframe &&
           visualization_point_stride == rhs.visualization_point_stride &&
           visualization_max_points_per_keyframe ==
               rhs.visualization_max_points_per_keyframe &&
           min_depth_m == rhs.min_depth_m && max_depth_m == rhs.max_depth_m &&
           depth_weighting_enabled == rhs.depth_weighting_enabled &&
           depth_weight_normal_radius == rhs.depth_weight_normal_radius &&
           depth_weight_min == rhs.depth_weight_min &&
           depth_weight_grazing_power == rhs.depth_weight_grazing_power &&
           depth_weight_range_ref == rhs.depth_weight_range_ref &&
           depth_weight_range_power == rhs.depth_weight_range_power &&
           depth_weight_range_min == rhs.depth_weight_range_min &&
           visualize_weights == rhs.visualize_weights &&
           visualize_landmark_scale_alignment ==
               rhs.visualize_landmark_scale_alignment &&
           da3_essential_factors_enabled == rhs.da3_essential_factors_enabled &&
           icp_only_da3_overlap_fusion == rhs.icp_only_da3_overlap_fusion &&
           min_confidence == rhs.min_confidence &&
           visualize_confidence == rhs.visualize_confidence &&
           point_radius == rhs.point_radius && verbose == rhs.verbose &&
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

// Result of the optional diagnostic pose-only optimization.  This graph is
// initialized from the VIO smoother poses and contains only mono-depth ICP
// factors plus one gauge-fixing pose prior per connected component.  Its poses
// never feed back into the VIO smoother, landmarks, or depth scale alignment.
struct MonoDepthICPOnlyResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool enabled = false;
  bool solution_available = false;
  bool valid = false;
  std::string failure_reason;
  std::size_t factor_count = 0u;
  std::size_t pose_count = 0u;
  std::size_t anchor_count = 0u;
  std::size_t iterations = 0u;
  double initial_error = 0.0;
  double final_error = 0.0;
  double error_ratio = 1.0;
  double max_translation_delta_m = 0.0;
  double max_rotation_delta_deg = 0.0;
  double optimization_ms = 0.0;
  std::map<FrameId, gtsam::Pose3> body_poses;

  // Experimental DA3-only alternative carried through the existing isolated
  // ICP diagnostic path.  It chains consecutive two-view predictions using
  // their duplicate middle image and never consumes VIO/odometry poses.
  bool da3_overlap_fusion = false;
  std::size_t da3_pair_count = 0u;
  std::size_t da3_fused_view_count = 0u;
  std::size_t da3_retained_view_count = 0u;
  std::size_t da3_retained_keyframe_count = 0u;
  std::size_t da3_component_reset_count = 0u;
  std::size_t da3_overlap_candidate_count = 0u;
  std::size_t da3_overlap_inlier_count = 0u;
  double da3_overlap_log_rmse = 0.0;
  double da3_last_pair_scale = 1.0;
  Point3Vector da3_overlap_cloud;
  RgbaColorVector da3_overlap_colors;
};

struct MonoDepthMapOutput {
  KIMERA_POINTER_TYPEDEFS(MonoDepthMapOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameId target_frame_id = 0u;
  Timestamp target_timestamp = 0;
  FrameId selected_scale_alignment_frame_id = 0u;
  MonoDepthScaleAlignmentResult selected_scale_alignment;
  cv::Mat landmark_scale_alignment_visualization_bgr;
  std::map<FrameId, MonoDepthScaleAlignmentResult> scale_alignments;
  std::size_t valid_scale_alignment_packets = 0u;
  std::size_t rejected_scale_alignment_packets = 0u;
  Point3Vector keyframe_cloud;
  RgbaColorVector keyframe_colors;
  Point3Vector window_cloud;
  RgbaColorVector window_colors;
  RgbaColorVector window_weight_colors;
  std::size_t window_keyframes = 0u;
  MonoDepthICPOnlyResult icp_only;
  Point3Vector icp_only_window_cloud;
  RgbaColorVector icp_only_window_colors;
  std::size_t icp_only_window_keyframes = 0u;
  float point_radius = 0.005f;
};

}  // namespace VIO
