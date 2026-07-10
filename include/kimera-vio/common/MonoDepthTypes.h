#pragma once

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <xfeat-cpp/mono_depth/mono_depth.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class MonoDepthMode { kSingleView = 0, kMultiView = 1 };

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
  int keyframe_skip = 0;
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
  double min_confidence = 1.1;
  bool visualize_confidence = false;
  float point_radius = 0.005f;
  bool verbose = false;
  bool align_scale_with_landmarks = false;
  MonoDepthMode mode = MonoDepthMode::kSingleView;

  bool operator==(const MonoDepthParams& rhs) const {
    return enabled == rhs.enabled && engine_path == rhs.engine_path &&
           keyframe_skip == rhs.keyframe_skip &&
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
           min_confidence == rhs.min_confidence &&
           visualize_confidence == rhs.visualize_confidence &&
           point_radius == rhs.point_radius && verbose == rhs.verbose &&
           align_scale_with_landmarks == rhs.align_scale_with_landmarks &&
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
  cv::Mat valid_mask;
  cv::Mat weight_image;
  cv::Mat confidence;
  cv::Mat confidence_mask;
  bool confidence_visualization_enabled = false;
  bool confidence_filtering_enabled = false;
  bool confidence_valid = false;
  double confidence_threshold = 0.0;
  std::size_t confidence_accepted_pixels = 0u;
  std::size_t confidence_rejected_pixels = 0u;
  double confidence_retained_fraction = 1.0;
  std::string confidence_error;
  MonoDepthIntrinsics intrinsics;
  gtsam::Pose3 body_T_cam;
  KeypointsCV keypoints;
  LandmarkIds landmark_ids;
  xfeat::MonoDepthMetadata metadata;
};

struct MonoDepthMapOutput {
  KIMERA_POINTER_TYPEDEFS(MonoDepthMapOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameId target_frame_id = 0u;
  Timestamp target_timestamp = 0;
  double scale = 1.0;
  double scale_log_rmse = 0.0;
  std::size_t scale_candidate_pairs = 0u;
  std::size_t scale_inlier_pairs = 0u;
  Point3Vector keyframe_cloud;
  RgbaColorVector keyframe_colors;
  Point3Vector window_cloud;
  RgbaColorVector window_colors;
  RgbaColorVector window_weight_colors;
  std::size_t window_keyframes = 0u;
  float point_radius = 0.005f;
};

}  // namespace VIO
