#include <gtsam/inference/Symbol.h>

#include <cstdlib>
#include <iostream>
#include <memory>

#include "kimera-vio/backend/Da3BaselineRatioFactors.h"
#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/factors/CameraAwareBaselineRatioFactor.h"

#define EXPECT_TRUE(condition, message)            \
  do {                                             \
    if (!(condition)) {                            \
      std::cerr << "[FAIL] " << (message) << "\n"; \
      return EXIT_FAILURE;                         \
    }                                              \
  } while (false)

namespace {

constexpr double kTolerance = 1e-8;

VIO::MonoDepthRawPacket makeView(const VIO::FrameId frame_id,
                                 const float depth,
                                 const gtsam::Pose3& body_T_cam) {
  VIO::MonoDepthRawPacket packet;
  packet.keyframe_id = frame_id;
  packet.depth = cv::Mat(8, 8, CV_32FC1, cv::Scalar(depth)).clone();
  packet.depth_support_mask = cv::Mat(8, 8, CV_8UC1, cv::Scalar(255)).clone();
  packet.valid_mask = cv::Mat(8, 8, CV_8UC1, cv::Scalar(255)).clone();
  packet.body_T_cam = body_T_cam;
  return packet;
}

VIO::MonoDepthRawPacket::ConstPtr makePair(
    const VIO::FrameId context_id,
    const VIO::FrameId current_id,
    const float context_depth,
    const float current_depth,
    const gtsam::Pose3& context_body_T_cam,
    const gtsam::Pose3& current_body_T_cam,
    const double da3_translation_norm) {
  auto context = std::make_shared<VIO::MonoDepthRawPacket>(
      makeView(context_id, context_depth, context_body_T_cam));
  auto current = std::make_shared<VIO::MonoDepthRawPacket>(
      makeView(current_id, current_depth, current_body_T_cam));
  current->da3_context_keyframe_id = context_id;
  current->da3_context_body_T_cam = context_body_T_cam;
  current->da3_context_cam_T_current_cam =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.03, -0.01, 0.02),
                   gtsam::Point3(da3_translation_norm, 0.0, 0.0));
  current->da3_context_packet = context;
  return current;
}

gtsam::Pose3 bodyPoseForCameraCenter(const gtsam::Point3& center,
                                     const gtsam::Pose3& body_T_cam) {
  const gtsam::Rot3 rotation =
      gtsam::Rot3::RzRyRx(0.04 * center.x(), -0.03, 0.02);
  return gtsam::Pose3(rotation,
                      center - rotation.rotate(body_T_cam.translation()));
}

struct Fixture {
  gtsam::Pose3 body_T_cam{gtsam::Rot3::RzRyRx(0.08, -0.04, 0.11),
                          gtsam::Point3(0.14, -0.03, 0.06)};

  VIO::MonoDepthRawPacket::ConstPtr firstPair(
      const VIO::FrameId first = 1u,
      const VIO::FrameId middle = 2u) const {
    return makePair(first, middle, 3.0f, 4.0f, body_T_cam, body_T_cam, 2.0);
  }

  VIO::MonoDepthRawPacket::ConstPtr secondPair(
      const VIO::FrameId middle = 2u,
      const VIO::FrameId last = 3u) const {
    // The shared middle image is predicted at depth 2 instead of 4, so this
    // run maps into the first run with overlap scale two.  With DA3 baseline
    // norms 2 and 3, the measured optimized ratio is 2 * 3 / 2 = 3.
    return makePair(middle, last, 2.0f, 5.0f, body_T_cam, body_T_cam, 3.0);
  }

  gtsam::Values firstMiddleState(const VIO::FrameId first = 1u,
                                 const VIO::FrameId middle = 2u) const {
    gtsam::Values state;
    state.insert(
        gtsam::Symbol(VIO::kPoseSymbolChar, first),
        bodyPoseForCameraCenter(gtsam::Point3(0.0, 0.0, 0.0), body_T_cam));
    state.insert(
        gtsam::Symbol(VIO::kPoseSymbolChar, middle),
        bodyPoseForCameraCenter(gtsam::Point3(1.0, 0.0, 0.0), body_T_cam));
    return state;
  }

  gtsam::Values lastValues(const VIO::FrameId last = 3u,
                           const double camera_x = 4.0) const {
    gtsam::Values values;
    values.insert(
        gtsam::Symbol(VIO::kPoseSymbolChar, last),
        bodyPoseForCameraCenter(gtsam::Point3(camera_x, 0.0, 0.0), body_T_cam));
    return values;
  }
};

int testDisabledFirstPairAndValidTriple() {
  const Fixture fixture;
  gtsam::NonlinearFactorGraph graph;
  VIO::Da3BaselineRatioFactors disabled(false, 0.25, 1);
  EXPECT_TRUE(!disabled
                      .addFactor(fixture.firstPair(),
                                 fixture.firstMiddleState(),
                                 fixture.lastValues(),
                                 &graph)
                      .added &&
                  graph.empty(),
              "disabled mode adds no factor");

  VIO::Da3BaselineRatioFactors factors(true, 0.25, 1);
  const auto first = factors.addFactor(
      fixture.firstPair(), fixture.firstMiddleState(), gtsam::Values(), &graph);
  EXPECT_TRUE(!first.added && graph.empty(),
              "the first DA3 pair only anchors the chain");
  const auto second = factors.addFactor(fixture.secondPair(),
                                        fixture.firstMiddleState(),
                                        fixture.lastValues(),
                                        &graph);
  EXPECT_TRUE(second.added && graph.size() == 1u,
              "two consecutive canonical DA3 pairs add exactly one factor");
  const auto factor =
      std::dynamic_pointer_cast<VIO::CameraAwareBaselineRatioFactor>(
          graph.front());
  EXPECT_TRUE(
      factor && std::abs(factor->measuredBaselineRatio() - 3.0) < kTolerance &&
          std::abs(second.overlap_scale_ratio - 2.0) < kTolerance &&
          second.overlap_candidate_count == 64u &&
          second.overlap_inlier_count == 64u &&
          second.overlap_log_rmse < kTolerance &&
          std::abs(second.initial_log_ratio_residual) < kTolerance,
      "the factor carries the expected overlap-derived ratio and "
      "diagnostics");
  return EXIT_SUCCESS;
}

int testCanonicalSupportIgnoresConfidenceAndOtherFactorModes() {
  const Fixture fixture;
  auto first = std::make_shared<VIO::MonoDepthRawPacket>(*fixture.firstPair());
  auto second =
      std::make_shared<VIO::MonoDepthRawPacket>(*fixture.secondPair());
  first->confidence_filtering_enabled = true;
  first->confidence_valid = false;
  first->confidence_accepted_pixels = 0u;
  first->valid_mask.release();
  auto low_confidence_context =
      std::make_shared<VIO::MonoDepthRawPacket>(*second->da3_context_packet);
  low_confidence_context->valid_mask.release();
  second->da3_context_packet = low_confidence_context;

  // This manager has no essential-factor, VGICP, ICP-only, scale-alignment,
  // or dense-map dependency. Canonical depth_support_mask alone gates overlap.
  VIO::Da3BaselineRatioFactors factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph graph;
  factors.addFactor(first, fixture.firstMiddleState(), gtsam::Values(), &graph);
  const auto result = factors.addFactor(
      second, fixture.firstMiddleState(), fixture.lastValues(), &graph);
  EXPECT_TRUE(result.added && graph.size() == 1u,
              "low confidence and disabled adjacent factor modes do not "
              "suppress canonical overlap");
  return EXIT_SUCCESS;
}

int testMissingOverlapAndGapReanchor() {
  const Fixture fixture;
  VIO::Da3BaselineRatioFactors factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph graph;
  factors.addFactor(
      fixture.firstPair(), gtsam::Values(), gtsam::Values(), &graph);

  auto missing_overlap =
      std::make_shared<VIO::MonoDepthRawPacket>(*fixture.secondPair());
  missing_overlap->da3_context_packet.reset();
  const auto missing = factors.addFactor(
      missing_overlap, gtsam::Values(), gtsam::Values(), &graph);
  EXPECT_TRUE(
      !missing.added && missing.diagnostic.find("missing") != std::string::npos,
      "a missing shared-image packet is skipped explicitly");

  const auto third =
      makePair(3u, 4u, 5.0f, 6.0f, fixture.body_T_cam, fixture.body_T_cam, 4.0);
  gtsam::Values state = fixture.firstMiddleState(2u, 3u);
  gtsam::Values values = fixture.lastValues(4u, 8.0);
  const auto recovered = factors.addFactor(third, state, values, &graph);
  EXPECT_TRUE(recovered.added,
              "the missing-overlap pair becomes the new anchor for later "
              "consecutive pairs");

  VIO::Da3BaselineRatioFactors gap_factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph gap_graph;
  gap_factors.addFactor(
      fixture.firstPair(), gtsam::Values(), gtsam::Values(), &gap_graph);
  const auto gap_pair =
      makePair(7u, 8u, 2.0f, 3.0f, fixture.body_T_cam, fixture.body_T_cam, 1.0);
  const auto gap = gap_factors.addFactor(
      gap_pair, gtsam::Values(), gtsam::Values(), &gap_graph);
  EXPECT_TRUE(
      !gap.added && gap.diagnostic.find("not consecutive") != std::string::npos,
      "a broken pair chain is skipped explicitly");
  const auto after_gap =
      makePair(8u, 9u, 3.0f, 4.0f, fixture.body_T_cam, fixture.body_T_cam, 2.0);
  gtsam::Values gap_state = fixture.firstMiddleState(7u, 8u);
  gtsam::Values gap_values = fixture.lastValues(9u, 2.0);
  EXPECT_TRUE(
      gap_factors.addFactor(after_gap, gap_state, gap_values, &gap_graph).added,
      "the newest valid pair re-anchors after a chain gap");
  return EXIT_SUCCESS;
}

int testInvalidOverlapAndExtrinsicMismatchReanchor() {
  const Fixture fixture;
  VIO::Da3BaselineRatioFactors invalid_overlap_factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph graph;
  invalid_overlap_factors.addFactor(
      fixture.firstPair(), gtsam::Values(), gtsam::Values(), &graph);
  auto invalid_overlap =
      std::make_shared<VIO::MonoDepthRawPacket>(*fixture.secondPair());
  auto invalid_context = std::make_shared<VIO::MonoDepthRawPacket>(
      *invalid_overlap->da3_context_packet);
  invalid_context->depth_support_mask.setTo(0);
  invalid_overlap->da3_context_packet = invalid_context;
  const auto invalid = invalid_overlap_factors.addFactor(
      invalid_overlap, gtsam::Values(), gtsam::Values(), &graph);
  EXPECT_TRUE(!invalid.added &&
                  invalid.diagnostic.find("insufficient") != std::string::npos,
              "invalid robust overlap is skipped");
  const auto third =
      makePair(3u, 4u, 5.0f, 6.0f, fixture.body_T_cam, fixture.body_T_cam, 4.0);
  EXPECT_TRUE(invalid_overlap_factors
                  .addFactor(third,
                             fixture.firstMiddleState(2u, 3u),
                             fixture.lastValues(4u, 8.0),
                             &graph)
                  .added,
              "a failed overlap still re-anchors at the newest valid pair");

  VIO::Da3BaselineRatioFactors mismatch_factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph mismatch_graph;
  mismatch_factors.addFactor(
      fixture.firstPair(), gtsam::Values(), gtsam::Values(), &mismatch_graph);
  const gtsam::Pose3 mismatched_extrinsic(
      fixture.body_T_cam.rotation(),
      fixture.body_T_cam.translation() + gtsam::Point3(0.01, 0.0, 0.0));
  auto mismatch = std::make_shared<VIO::MonoDepthRawPacket>(*makePair(
      2u, 3u, 2.0f, 5.0f, mismatched_extrinsic, fixture.body_T_cam, 3.0));
  const auto mismatch_result = mismatch_factors.addFactor(
      mismatch, gtsam::Values(), gtsam::Values(), &mismatch_graph);
  EXPECT_TRUE(!mismatch_result.added && mismatch_result.diagnostic.find(
                                            "extrinsics") != std::string::npos,
              "shared-frame extrinsic disagreement is skipped");
  EXPECT_TRUE(mismatch_factors
                  .addFactor(third,
                             fixture.firstMiddleState(2u, 3u),
                             fixture.lastValues(4u, 8.0),
                             &mismatch_graph)
                  .added,
              "an extrinsic mismatch re-anchors at the newest pair");
  return EXIT_SUCCESS;
}

int testMarginalizedSkipDuplicateAndRetry() {
  const Fixture fixture;
  VIO::Da3BaselineRatioFactors marginalized_factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph marginalized_graph;
  marginalized_factors.addFactor(fixture.firstPair(),
                                 gtsam::Values(),
                                 gtsam::Values(),
                                 &marginalized_graph);
  const auto marginalized = marginalized_factors.addFactor(fixture.secondPair(),
                                                           gtsam::Values(),
                                                           fixture.lastValues(),
                                                           &marginalized_graph);
  EXPECT_TRUE(!marginalized.added && marginalized.diagnostic.find(
                                         "fixed-lag") != std::string::npos,
              "a marginalized endpoint is skipped explicitly");
  const auto third =
      makePair(3u, 4u, 5.0f, 6.0f, fixture.body_T_cam, fixture.body_T_cam, 4.0);
  EXPECT_TRUE(marginalized_factors
                  .addFactor(third,
                             fixture.firstMiddleState(2u, 3u),
                             fixture.lastValues(4u, 8.0),
                             &marginalized_graph)
                  .added,
              "a marginalized triple does not prevent later anchored triples");

  VIO::Da3BaselineRatioFactors retry_factors(true, 0.25, 1);
  gtsam::NonlinearFactorGraph graph;
  retry_factors.addFactor(
      fixture.firstPair(), gtsam::Values(), gtsam::Values(), &graph);
  EXPECT_TRUE(retry_factors
                      .addFactor(fixture.secondPair(),
                                 fixture.firstMiddleState(),
                                 fixture.lastValues(),
                                 &graph)
                      .added &&
                  graph.size() == 1u,
              "a valid factor is pending before smoother notification");
  const auto pending_duplicate =
      retry_factors.addFactor(fixture.secondPair(),
                              fixture.firstMiddleState(),
                              fixture.lastValues(),
                              &graph);
  EXPECT_TRUE(!pending_duplicate.added && graph.size() == 1u,
              "a duplicate call adds no second pending factor");

  retry_factors.notifySmootherUpdateResult(false);
  gtsam::NonlinearFactorGraph retry_graph;
  EXPECT_TRUE(retry_factors
                  .addFactor(fixture.secondPair(),
                             fixture.firstMiddleState(),
                             fixture.lastValues(),
                             &retry_graph)
                  .added,
              "a failed smoother update restores the predecessor for retry");
  retry_factors.notifySmootherUpdateResult(true);
  gtsam::NonlinearFactorGraph committed_graph;
  const auto committed_duplicate =
      retry_factors.addFactor(fixture.secondPair(),
                              fixture.firstMiddleState(),
                              fixture.lastValues(),
                              &committed_graph);
  EXPECT_TRUE(!committed_duplicate.added && committed_graph.empty() &&
                  retry_factors.committedTripleCount() == 1u,
              "a committed triple cannot be duplicated by extra iterations");
  return EXIT_SUCCESS;
}

}  // namespace

int main() {
  int status = EXIT_SUCCESS;
  status |= testDisabledFirstPairAndValidTriple();
  status |= testCanonicalSupportIgnoresConfidenceAndOtherFactorModes();
  status |= testMissingOverlapAndGapReanchor();
  status |= testInvalidOverlapAndExtrinsicMismatchReanchor();
  status |= testMarginalizedSkipDuplicateAndRetry();
  if (status == EXIT_SUCCESS) {
    std::cout << "All DA3 baseline-ratio backend tests PASSED.\n";
  }
  return status;
}
