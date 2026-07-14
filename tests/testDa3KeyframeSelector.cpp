#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "kimera-vio/frontend/Da3KeyframeSelector.h"

namespace {

#define EXPECT_TRUE(condition, message)                                       \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::cerr << "FAILED: " << (message) << " (line " << __LINE__ << ")\n"; \
      return EXIT_FAILURE;                                                    \
    }                                                                         \
  } while (false)

int testMethodParsing() {
  EXPECT_TRUE(VIO::da3KeyframeSelectionMethodFromString("distance") ==
                  VIO::Da3KeyframeSelectionMethod::kDistance,
              "distance method parses");
  EXPECT_TRUE(VIO::da3KeyframeSelectionMethodFromString("FIXED-SKIP") ==
                  VIO::Da3KeyframeSelectionMethod::kFixedSkip,
              "fixed-skip method parses case-insensitively");
  EXPECT_TRUE(VIO::da3KeyframeSelectionMethodToString(
                  VIO::Da3KeyframeSelectionMethod::kFixedSkip) == "fixed_skip",
              "fixed-skip method has a stable parameter spelling");
  bool threw = false;
  try {
    static_cast<void>(
        VIO::da3KeyframeSelectionMethodFromString("not_a_method"));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  EXPECT_TRUE(threw, "unknown methods are rejected");
  return EXIT_SUCCESS;
}

int testDistanceSelector() {
  VIO::MonoDepthParams params;
  params.da3_keyframe_selection_method =
      VIO::Da3KeyframeSelectionMethod::kDistance;
  params.min_keyframe_distance_m = 2.0;
  auto selector = VIO::makeDa3KeyframeSelector(params);

  VIO::Da3KeyframeSelectionInput input;
  input.context_keyframe_id = 10u;
  input.candidate_keyframe_id = 11u;
  input.odometry_world_T_context_cam = gtsam::Pose3();
  input.odometry_world_T_candidate_cam =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  auto result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && !result.selected,
              "distance selector holds a short baseline");
  EXPECT_TRUE(result.camera_displacement_m.has_value() &&
                  std::abs(*result.camera_displacement_m - 1.0) < 1e-12,
              "distance selector reports the endpoint chord");

  input.candidate_keyframe_id = 12u;
  input.odometry_world_T_candidate_cam =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0));
  result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && result.selected,
              "distance selector accepts the threshold boundary");

  input.odometry_world_T_candidate_cam = std::nullopt;
  result = selector->evaluate(input);
  EXPECT_TRUE(!result.valid && !result.selected,
              "distance selector requires endpoint odometry");
  return EXIT_SUCCESS;
}

int testFixedSkipSelector() {
  VIO::MonoDepthParams params;
  params.da3_keyframe_selection_method =
      VIO::Da3KeyframeSelectionMethod::kFixedSkip;
  params.da3_keyframe_skip = 2;
  auto selector = VIO::makeDa3KeyframeSelector(params);

  VIO::Da3KeyframeSelectionInput input;
  input.context_keyframe_id = 20u;
  input.candidate_keyframe_id = 21u;
  auto result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && !result.selected && result.held_candidates == 1u,
              "fixed skip holds the first intermediate keyframe");

  input.candidate_keyframe_id = 22u;
  result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && !result.selected && result.held_candidates == 2u,
              "fixed skip holds the configured number of keyframes");

  input.candidate_keyframe_id = 23u;
  result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && result.selected && result.held_candidates == 2u,
              "fixed skip selects the next keyframe");

  input.context_keyframe_id = 23u;
  input.candidate_keyframe_id = 24u;
  result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && !result.selected && result.held_candidates == 1u,
              "fixed skip resets after selection");
  EXPECT_TRUE(!result.camera_displacement_m.has_value(),
              "fixed skip does not require or report odometry");
  return EXIT_SUCCESS;
}

int testConsecutiveAndInvalidFixedSkip() {
  VIO::MonoDepthParams params;
  params.da3_keyframe_selection_method =
      VIO::Da3KeyframeSelectionMethod::kFixedSkip;
  params.da3_keyframe_skip = 0;
  auto selector = VIO::makeDa3KeyframeSelector(params);
  VIO::Da3KeyframeSelectionInput input;
  input.context_keyframe_id = 30u;
  input.candidate_keyframe_id = 31u;
  const auto result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && result.selected,
              "zero fixed skip selects consecutive VIO keyframes");

  params.da3_keyframe_skip = -1;
  bool threw = false;
  try {
    static_cast<void>(VIO::makeDa3KeyframeSelector(params));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  EXPECT_TRUE(threw, "negative fixed skip is rejected");
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  int status = EXIT_SUCCESS;
  status |= testMethodParsing();
  status |= testDistanceSelector();
  status |= testFixedSkipSelector();
  status |= testConsecutiveAndInvalidFixedSkip();
  if (status == EXIT_SUCCESS) {
    std::cout << "All DA3 keyframe selector tests PASSED.\n";
  }
  return status;
}
