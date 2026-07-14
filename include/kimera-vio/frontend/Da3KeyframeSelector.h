#pragma once

#include <gtsam/geometry/Pose3.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/common/vio_types.h"

namespace VIO {

struct MonoDepthPairDistanceGateResult {
  bool valid = false;
  bool passes = false;
  double camera_displacement_m = 0.0;
  std::string error;
};

MonoDepthPairDistanceGateResult evaluateMonoDepthPairDistanceGate(
    const gtsam::Pose3& odometry_world_T_context_cam,
    const gtsam::Pose3& odometry_world_T_current_cam,
    double min_keyframe_distance_m);

struct Da3KeyframeSelectionInput {
  FrameId context_keyframe_id = 0u;
  FrameId candidate_keyframe_id = 0u;
  std::optional<gtsam::Pose3> odometry_world_T_context_cam;
  std::optional<gtsam::Pose3> odometry_world_T_candidate_cam;
  const LandmarkIds* context_feature_track_ids = nullptr;
  const LandmarkIds* candidate_feature_track_ids = nullptr;
};

struct Da3KeyframeSelectionResult {
  bool valid = false;
  bool selected = false;
  std::size_t held_candidates = 0u;
  std::optional<double> camera_displacement_m;
  std::optional<double> covisibility_score;
  std::size_t reference_tracks = 0u;
  std::size_t shared_tracks = 0u;
  std::string diagnostic;
};

/**
 * Selects endpoint keyframes for pose-free two-view DA3 inference.
 *
 * Implementations may be stateful. evaluate() is called exactly once for each
 * incoming VIO keyframe after the current DA3 context keyframe. A selected
 * candidate becomes the next context even if inference subsequently fails.
 */
class Da3KeyframeSelector {
 public:
  virtual ~Da3KeyframeSelector() = default;

  virtual Da3KeyframeSelectionResult evaluate(
      const Da3KeyframeSelectionInput& input) = 0;
  virtual Da3KeyframeSelectionMethod method() const noexcept = 0;
};

std::unique_ptr<Da3KeyframeSelector> makeDa3KeyframeSelector(
    const MonoDepthParams& params);

}  // namespace VIO
