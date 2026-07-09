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
  int point_stride = 4;
  int max_points_per_keyframe = 5000;
  double min_depth_m = 0.1;
  double max_depth_m = 30.0;
  float point_radius = 0.005f;
  bool verbose = false;
  MonoDepthMode mode = MonoDepthMode::kSingleView;
  int view_count = 1;
  int view_stride = 1;

  bool operator==(const MonoDepthParams& rhs) const {
    return enabled == rhs.enabled && engine_path == rhs.engine_path &&
           point_stride == rhs.point_stride &&
           max_points_per_keyframe == rhs.max_points_per_keyframe &&
           min_depth_m == rhs.min_depth_m && max_depth_m == rhs.max_depth_m &&
           point_radius == rhs.point_radius && verbose == rhs.verbose &&
           mode == rhs.mode && view_count == rhs.view_count &&
           view_stride == rhs.view_stride;
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
  Point3Vector accumulated_map;
  RgbaColorVector accumulated_colors;
  float point_radius = 0.005f;
};

}  // namespace VIO
