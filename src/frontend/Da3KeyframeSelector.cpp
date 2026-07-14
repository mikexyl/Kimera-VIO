#include "kimera-vio/frontend/Da3KeyframeSelector.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace VIO {
namespace {

class DistanceDa3KeyframeSelector final : public Da3KeyframeSelector {
 public:
  explicit DistanceDa3KeyframeSelector(const double min_distance_m)
      : min_distance_m_(min_distance_m) {
    if (!std::isfinite(min_distance_m_) || min_distance_m_ < 0.0) {
      throw std::invalid_argument(
          "DA3 minimum keyframe distance must be finite and non-negative");
    }
  }

  Da3KeyframeSelectionResult evaluate(
      const Da3KeyframeSelectionInput& input) override {
    Da3KeyframeSelectionResult result;
    if (!input.odometry_world_T_context_cam.has_value() ||
        !input.odometry_world_T_candidate_cam.has_value()) {
      result.diagnostic =
          "distance selection requires odometry poses for both endpoints";
      return result;
    }

    const MonoDepthPairDistanceGateResult distance =
        evaluateMonoDepthPairDistanceGate(*input.odometry_world_T_context_cam,
                                          *input.odometry_world_T_candidate_cam,
                                          min_distance_m_);
    if (!distance.valid) {
      result.diagnostic = distance.error;
      return result;
    }

    result.valid = true;
    result.selected = distance.passes;
    result.camera_displacement_m = distance.camera_displacement_m;
    std::ostringstream diagnostic;
    diagnostic << "camera displacement " << distance.camera_displacement_m
               << " m " << (distance.passes ? ">= " : "< ") << min_distance_m_
               << " m";
    result.diagnostic = diagnostic.str();
    return result;
  }

  Da3KeyframeSelectionMethod method() const noexcept override {
    return Da3KeyframeSelectionMethod::kDistance;
  }

 private:
  double min_distance_m_;
};

class FixedSkipDa3KeyframeSelector final : public Da3KeyframeSelector {
 public:
  explicit FixedSkipDa3KeyframeSelector(const std::size_t keyframes_to_skip)
      : keyframes_to_skip_(keyframes_to_skip) {}

  Da3KeyframeSelectionResult evaluate(
      const Da3KeyframeSelectionInput& input) override {
    Da3KeyframeSelectionResult result;
    if (input.candidate_keyframe_id == input.context_keyframe_id) {
      result.diagnostic = "context and candidate keyframe IDs are identical";
      return result;
    }

    result.valid = true;
    result.held_candidates = held_candidates_;
    if (held_candidates_ < keyframes_to_skip_) {
      ++held_candidates_;
      result.held_candidates = held_candidates_;
      std::ostringstream diagnostic;
      diagnostic << "held " << held_candidates_ << "/" << keyframes_to_skip_
                 << " intermediate keyframes";
      result.diagnostic = diagnostic.str();
      return result;
    }

    result.selected = true;
    std::ostringstream diagnostic;
    diagnostic << "selected after " << held_candidates_
               << " intermediate keyframes";
    result.diagnostic = diagnostic.str();
    held_candidates_ = 0u;
    return result;
  }

  Da3KeyframeSelectionMethod method() const noexcept override {
    return Da3KeyframeSelectionMethod::kFixedSkip;
  }

 private:
  std::size_t keyframes_to_skip_;
  std::size_t held_candidates_ = 0u;
};

}  // namespace

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

std::unique_ptr<Da3KeyframeSelector> makeDa3KeyframeSelector(
    const MonoDepthParams& params) {
  switch (params.da3_keyframe_selection_method) {
    case Da3KeyframeSelectionMethod::kDistance:
      return std::make_unique<DistanceDa3KeyframeSelector>(
          params.min_keyframe_distance_m);
    case Da3KeyframeSelectionMethod::kFixedSkip:
      if (params.da3_keyframe_skip < 0) {
        throw std::invalid_argument(
            "mono_depth.da3_keyframe_skip must be non-negative");
      }
      return std::make_unique<FixedSkipDa3KeyframeSelector>(
          static_cast<std::size_t>(params.da3_keyframe_skip));
  }
  throw std::invalid_argument("Unknown DA3 keyframe selection method");
}

}  // namespace VIO
