#pragma once

#include <xfeat-cpp/mono_depth/mono_depth.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct MonoDepthConfidenceFilterResult {
  cv::Mat mask;
  bool filtering_enabled = false;
  bool confidence_valid = false;
  std::size_t accepted_pixels = 0u;
  std::size_t rejected_pixels = 0u;
  double retained_fraction = 1.0;
  std::string error;
};

struct MonoDepthPairDistanceGateResult {
  bool valid = false;
  bool passes = false;
  double camera_displacement_m = 0.0;
  std::string error;
};

MonoDepthConfidenceFilterResult makeMonoDepthConfidenceFilter(
    const cv::Size& expected_size,
    const cv::Mat& confidence,
    double min_confidence);

cv::Mat makeMonoDepthValidMask(const cv::Mat& depth,
                               const cv::Mat& sky_mask,
                               const cv::Mat& confidence_mask = cv::Mat());

MonoDepthPairDistanceGateResult evaluateMonoDepthPairDistanceGate(
    const gtsam::Pose3& odometry_world_T_context_cam,
    const gtsam::Pose3& odometry_world_T_current_cam,
    double min_keyframe_distance_m);

class MonoDepthInference {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDepthInference);
  KIMERA_POINTER_TYPEDEFS(MonoDepthInference);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MonoDepthInference(const MonoDepthParams& params);
  ~MonoDepthInference() = default;

  MonoDepthRawPacket::ConstPtr inferKeyframe(
      const Frame& frame,
      const std::optional<gtsam::Pose3>& odometry_world_T_body =
          std::nullopt) const;

 private:
  struct BufferedKeyframe {
    FrameId keyframe_id = 0u;
    Timestamp timestamp = 0;
    cv::Mat image_bgr;
    MonoDepthIntrinsics intrinsics;
    gtsam::Pose3 body_T_cam;
    std::optional<gtsam::Pose3> odometry_world_T_body;
    KeypointsCV keypoints;
    LandmarkIds landmark_ids;
  };

  struct Da3PairInfo {
    FrameId context_keyframe_id = 0u;
    gtsam::Pose3 context_body_T_cam;
    std::optional<gtsam::Pose3> context_cam_T_current_cam;
  };

  static cv::Mat toBgrImage(const cv::Mat& image);
  static std::optional<BufferedKeyframe> bufferKeyframe(
      const Frame& frame,
      const std::optional<gtsam::Pose3>& odometry_world_T_body);
  static xfeat::CameraIntrinsics toXfeatIntrinsics(
      const MonoDepthIntrinsics& intrinsics);
  static std::optional<gtsam::Pose3> makeDa3ContextToCurrentPose(
      const xfeat::MonoDepthResult& context_result,
      const xfeat::MonoDepthResult& current_result);
  MonoDepthRawPacket::ConstPtr buildPacket(
      const BufferedKeyframe& frame,
      const xfeat::MonoDepthResult& depth_result,
      bool apply_confidence_filter,
      const std::optional<Da3PairInfo>& pair_info = std::nullopt) const;

 private:
  MonoDepthParams params_;
  std::unique_ptr<xfeat::MonoDepth> mono_depth_;
  mutable std::optional<BufferedKeyframe> buffered_keyframe_;
};

}  // namespace VIO
