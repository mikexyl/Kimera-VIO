#include <gtsam/inference/Symbol.h>

#include <cstdlib>
#include <iostream>
#include <memory>

#include "kimera-vio/backend/Da3EssentialMatrixFactors.h"
#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/factors/CameraAwareEssentialMatrixFactor.h"

#define EXPECT_TRUE(condition, message)            \
  do {                                             \
    if (!(condition)) {                            \
      std::cerr << "[FAIL] " << (message) << "\n"; \
      return EXIT_FAILURE;                         \
    }                                              \
  } while (false)

namespace {

VIO::MonoDepthRawPacket::ConstPtr makePair(
    const VIO::FrameId context_id,
    const VIO::FrameId current_id,
    const gtsam::Pose3& context_body_T_cam,
    const gtsam::Pose3& current_body_T_cam,
    const gtsam::Pose3& context_cam_T_current_cam) {
  auto packet = std::make_shared<VIO::MonoDepthRawPacket>();
  packet->keyframe_id = current_id;
  packet->da3_context_keyframe_id = context_id;
  packet->da3_context_body_T_cam = context_body_T_cam;
  packet->da3_context_cam_T_current_cam = context_cam_T_current_cam;
  packet->body_T_cam = current_body_T_cam;
  return packet;
}

struct Fixture {
  VIO::FrameId context_id = 7u;
  VIO::FrameId current_id = 11u;
  gtsam::Pose3 context_body_T_cam{gtsam::Rot3::RzRyRx(0.03, -0.04, 0.02),
                                  gtsam::Point3(0.1, 0.0, 0.03)};
  gtsam::Pose3 current_body_T_cam{gtsam::Rot3::RzRyRx(-0.02, 0.01, 0.05),
                                  gtsam::Point3(-0.04, 0.02, 0.07)};
  gtsam::Pose3 measurement{gtsam::Rot3::RzRyRx(0.1, -0.07, 0.04),
                           gtsam::Point3(0.7, 0.2, -0.1)};
  gtsam::Pose3 context_body{gtsam::Rot3::RzRyRx(-0.1, 0.05, 0.08),
                            gtsam::Point3(1.0, -2.0, 0.5)};
  gtsam::Pose3 current_body = context_body.compose(context_body_T_cam)
                                  .compose(measurement)
                                  .compose(current_body_T_cam.inverse());

  VIO::MonoDepthRawPacket::ConstPtr packet() const {
    return makePair(context_id,
                    current_id,
                    context_body_T_cam,
                    current_body_T_cam,
                    measurement);
  }

  gtsam::Values contextState() const {
    gtsam::Values state;
    state.insert(gtsam::Symbol(VIO::kPoseSymbolChar, context_id), context_body);
    return state;
  }

  gtsam::Values currentValues() const {
    gtsam::Values values;
    values.insert(gtsam::Symbol(VIO::kPoseSymbolChar, current_id),
                  current_body);
    return values;
  }
};

int testDisabledAndValidModes() {
  const Fixture fixture;
  gtsam::NonlinearFactorGraph graph;
  VIO::Da3EssentialMatrixFactors disabled(false);
  const auto disabled_result = disabled.addFactor(fixture.packet(),
                                                  fixture.contextState(),
                                                  fixture.currentValues(),
                                                  &graph);
  EXPECT_TRUE(!disabled_result.added && graph.empty(),
              "disabled mode adds no factors");

  VIO::Da3EssentialMatrixFactors enabled(true);
  const auto result = enabled.addFactor(fixture.packet(),
                                        fixture.contextState(),
                                        fixture.currentValues(),
                                        &graph);
  EXPECT_TRUE(result.added && graph.size() == 1u,
              "one valid active DA3 pair adds exactly one factor");
  EXPECT_TRUE(std::dynamic_pointer_cast<VIO::CameraAwareEssentialMatrixFactor>(
                  graph.front()) != nullptr,
              "the backend adds the camera-aware essential factor type");
  EXPECT_TRUE(result.initial_rotation_residual_norm < 1e-8 &&
                  result.initial_direction_residual_norm < 1e-8,
              "backend reports the initial pair residual norms");
  return EXIT_SUCCESS;
}

int testMissingAndOutOfWindowMetadata() {
  const Fixture fixture;
  VIO::Da3EssentialMatrixFactors factors(true);
  gtsam::NonlinearFactorGraph graph;

  auto missing = std::make_shared<VIO::MonoDepthRawPacket>(*fixture.packet());
  missing->da3_context_body_T_cam.reset();
  const auto missing_result = factors.addFactor(
      missing, fixture.contextState(), fixture.currentValues(), &graph);
  EXPECT_TRUE(
      !missing_result.added && graph.empty() &&
          missing_result.diagnostic.find("metadata") != std::string::npos,
      "missing pair metadata is skipped");

  const auto marginalized_result = factors.addFactor(
      fixture.packet(), gtsam::Values(), fixture.currentValues(), &graph);
  EXPECT_TRUE(
      !marginalized_result.added && graph.empty() &&
          marginalized_result.diagnostic.find("fixed-lag") != std::string::npos,
      "a marginalized context is skipped as out of window");
  return EXIT_SUCCESS;
}

int testConfidenceIndependenceAndDeduplication() {
  const Fixture fixture;
  auto low_confidence =
      std::make_shared<VIO::MonoDepthRawPacket>(*fixture.packet());
  low_confidence->confidence_filtering_enabled = true;
  low_confidence->confidence_valid = false;
  low_confidence->confidence_accepted_pixels = 0u;
  low_confidence->confidence_retained_fraction = 0.0;
  low_confidence->valid_mask.release();
  low_confidence->depth.release();

  VIO::Da3EssentialMatrixFactors factors(true);
  gtsam::NonlinearFactorGraph graph;
  const auto first = factors.addFactor(
      low_confidence, fixture.contextState(), fixture.currentValues(), &graph);
  EXPECT_TRUE(first.added && graph.size() == 1u,
              "low depth confidence does not suppress a valid pose factor");

  const auto pending_duplicate = factors.addFactor(
      low_confidence, fixture.contextState(), fixture.currentValues(), &graph);
  EXPECT_TRUE(!pending_duplicate.added && graph.size() == 1u,
              "a pending pair is not duplicated in one smoother update");
  factors.notifySmootherUpdateResult(true);

  gtsam::NonlinearFactorGraph next_graph;
  const auto committed_duplicate = factors.addFactor(low_confidence,
                                                     fixture.contextState(),
                                                     fixture.currentValues(),
                                                     &next_graph);
  EXPECT_TRUE(!committed_duplicate.added && next_graph.empty() &&
                  factors.committedPairCount() == 1u,
              "no duplicate is added on later or extra smoother iterations");
  return EXIT_SUCCESS;
}

int testFailedUpdateCanRetryAndZeroBaselineSkips() {
  const Fixture fixture;
  VIO::Da3EssentialMatrixFactors factors(true);
  gtsam::NonlinearFactorGraph graph;
  EXPECT_TRUE(factors
                  .addFactor(fixture.packet(),
                             fixture.contextState(),
                             fixture.currentValues(),
                             &graph)
                  .added,
              "valid pair is pending before smoother notification");
  factors.notifySmootherUpdateResult(false);
  gtsam::NonlinearFactorGraph retry_graph;
  EXPECT_TRUE(factors
                  .addFactor(fixture.packet(),
                             fixture.contextState(),
                             fixture.currentValues(),
                             &retry_graph)
                  .added,
              "a failed smoother update permits a safe retry");

  const auto zero_pair = makePair(fixture.context_id,
                                  fixture.current_id + 1u,
                                  fixture.context_body_T_cam,
                                  fixture.current_body_T_cam,
                                  gtsam::Pose3());
  gtsam::Values zero_current;
  zero_current.insert(
      gtsam::Symbol(VIO::kPoseSymbolChar, fixture.current_id + 1u),
      fixture.current_body);
  const auto zero_result = factors.addFactor(
      zero_pair, fixture.contextState(), zero_current, &retry_graph);
  EXPECT_TRUE(!zero_result.added &&
                  zero_result.diagnostic.find("zero") != std::string::npos,
              "zero-baseline DA3 metadata is skipped explicitly");
  return EXIT_SUCCESS;
}

}  // namespace

int main() {
  int status = EXIT_SUCCESS;
  status |= testDisabledAndValidModes();
  status |= testMissingAndOutOfWindowMetadata();
  status |= testConfidenceIndependenceAndDeduplication();
  status |= testFailedUpdateCanRetryAndZeroBaselineSkips();
  if (status == EXIT_SUCCESS) {
    std::cout << "All DA3 essential-factor backend tests PASSED.\n";
  }
  return status;
}
