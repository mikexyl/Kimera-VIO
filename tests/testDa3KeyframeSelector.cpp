#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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
  EXPECT_TRUE(VIO::da3KeyframeSelectionMethodFromString("COVISIBILITY") ==
                  VIO::Da3KeyframeSelectionMethod::kCovisibility,
              "covisibility method parses case-insensitively");
  EXPECT_TRUE(VIO::da3KeyframeSelectionMethodFromString("covis") ==
                  VIO::Da3KeyframeSelectionMethod::kCovisibility,
              "covisibility method has a short alias");
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

int testCovisibilitySelector() {
  VIO::MonoDepthParams params;
  params.da3_keyframe_selection_method =
      VIO::Da3KeyframeSelectionMethod::kCovisibility;
  params.da3_keyframe_covisibility_threshold = 0.5;
  auto selector = VIO::makeDa3KeyframeSelector(params);

  const VIO::LandmarkIds reference_tracks{1, 2, 3, 4, -1, 4};
  VIO::LandmarkIds candidate_tracks{1, 2, 3, 99, 3};
  VIO::Da3KeyframeSelectionInput input;
  input.context_keyframe_id = 40u;
  input.candidate_keyframe_id = 41u;
  input.context_feature_track_ids = &reference_tracks;
  input.candidate_feature_track_ids = &candidate_tracks;

  auto result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && !result.selected,
              "high covisibility keeps the buffered reference");
  EXPECT_TRUE(result.reference_tracks == 4u && result.shared_tracks == 3u,
              "covisibility counts unique valid reference tracks");
  EXPECT_TRUE(result.covisibility_score.has_value() &&
                  std::abs(*result.covisibility_score - 0.75) < 1e-12,
              "covisibility is normalized by reference tracks");
  EXPECT_TRUE(!result.camera_displacement_m.has_value(),
              "covisibility selection does not require odometry");

  input.candidate_keyframe_id = 42u;
  candidate_tracks = {1, 4, 100};
  result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && !result.selected &&
                  std::abs(*result.covisibility_score - 0.5) < 1e-12,
              "threshold equality continues buffering");

  input.candidate_keyframe_id = 43u;
  candidate_tracks = {1, 100};
  result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && result.selected && result.shared_tracks == 1u,
              "covisibility below the threshold selects the candidate");

  input.candidate_feature_track_ids = nullptr;
  result = selector->evaluate(input);
  EXPECT_TRUE(!result.valid && !result.selected,
              "missing feature-track metadata is rejected");
  return EXIT_SUCCESS;
}

int testCovisibilityEdgeCases() {
  VIO::MonoDepthParams params;
  params.da3_keyframe_selection_method =
      VIO::Da3KeyframeSelectionMethod::kCovisibility;
  params.da3_keyframe_covisibility_threshold = 0.5;
  auto selector = VIO::makeDa3KeyframeSelector(params);

  const VIO::LandmarkIds no_reference_tracks{-1, -1};
  const VIO::LandmarkIds candidate_tracks{1, 2};
  VIO::Da3KeyframeSelectionInput input;
  input.context_keyframe_id = 50u;
  input.candidate_keyframe_id = 51u;
  input.context_feature_track_ids = &no_reference_tracks;
  input.candidate_feature_track_ids = &candidate_tracks;
  const auto result = selector->evaluate(input);
  EXPECT_TRUE(result.valid && result.selected &&
                  result.covisibility_score.has_value() &&
                  *result.covisibility_score == 0.0,
              "an empty valid reference track set has zero covisibility");

  for (const double invalid_threshold :
       {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    params.da3_keyframe_covisibility_threshold = invalid_threshold;
    bool threw = false;
    try {
      static_cast<void>(VIO::makeDa3KeyframeSelector(params));
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    EXPECT_TRUE(threw, "invalid covisibility thresholds are rejected");
  }
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
  status |= testCovisibilitySelector();
  status |= testCovisibilityEdgeCases();
  if (status == EXIT_SUCCESS) {
    std::cout << "All DA3 keyframe selector tests PASSED.\n";
  }
  return status;
}
