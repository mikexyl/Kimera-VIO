#include <glog/logging.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <stdexcept>

#include "kimera-vio/common/MonoDepthUtils.h"
#include "kimera-vio/frontend/MonoDepthInference.h"

#define EXPECT_TRUE(condition, message)            \
  do {                                             \
    if (!(condition)) {                            \
      std::cerr << "[FAIL] " << (message) << "\n"; \
      return EXIT_FAILURE;                         \
    }                                              \
  } while (false)

namespace {

int testThresholdBoundaryAndNonFiniteValues() {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const cv::Mat confidence =
      (cv::Mat_<float>(1, 6) << 1.099f, 1.1f, 1.2f, nan, inf, -inf);

  const auto filter =
      VIO::makeMonoDepthConfidenceFilter(confidence.size(), confidence, 1.1);
  EXPECT_TRUE(filter.filtering_enabled, "positive threshold enables filtering");
  EXPECT_TRUE(filter.confidence_valid, "well-formed confidence is accepted");
  EXPECT_TRUE(filter.accepted_pixels == 2u,
              "the 1.1 boundary and larger finite value pass");
  EXPECT_TRUE(filter.rejected_pixels == 4u,
              "sub-threshold and non-finite values are rejected");
  EXPECT_TRUE(filter.mask.at<uint8_t>(0, 0) == 0u,
              "value below threshold is masked");
  EXPECT_TRUE(filter.mask.at<uint8_t>(0, 1) == 255u,
              "value exactly at threshold is retained");
  EXPECT_TRUE(std::abs(filter.retained_fraction - (2.0 / 6.0)) < 1e-12,
              "retained fraction matches accepted pixels");
  return EXIT_SUCCESS;
}

int testMissingAndMalformedConfidenceFailClosed() {
  const cv::Size size(3, 2);
  const auto missing = VIO::makeMonoDepthConfidenceFilter(size, cv::Mat(), 1.1);
  EXPECT_TRUE(!missing.confidence_valid, "missing confidence is diagnosed");
  EXPECT_TRUE(missing.accepted_pixels == 0u && missing.rejected_pixels == 6u,
              "missing confidence rejects every pixel");
  EXPECT_TRUE(cv::countNonZero(missing.mask) == 0,
              "missing confidence produces an all-zero mask");

  const cv::Mat wrong_type(size, CV_8UC1, cv::Scalar(2));
  const auto malformed =
      VIO::makeMonoDepthConfidenceFilter(size, wrong_type, 1.1);
  EXPECT_TRUE(!malformed.confidence_valid, "non-float confidence is diagnosed");
  EXPECT_TRUE(cv::countNonZero(malformed.mask) == 0,
              "malformed confidence fails closed");

  const cv::Mat wrong_shape(1, 3, CV_32FC1, cv::Scalar(2.0f));
  const auto mismatched =
      VIO::makeMonoDepthConfidenceFilter(size, wrong_shape, 1.1);
  EXPECT_TRUE(!mismatched.confidence_valid,
              "shape-incompatible confidence is diagnosed");
  EXPECT_TRUE(cv::countNonZero(mismatched.mask) == 0,
              "shape mismatch fails closed");

  const cv::Mat depth(size, CV_32FC1, cv::Scalar(2.0f));
  const cv::Mat final_mask =
      VIO::makeMonoDepthValidMask(depth, cv::Mat(), mismatched.mask);
  EXPECT_TRUE(cv::countNonZero(final_mask) == 0,
              "failed confidence cannot leak into the final valid mask");
  return EXIT_SUCCESS;
}

int testDisabledFilteringAndSingleViewBehavior() {
  const cv::Mat depth = (cv::Mat_<float>(1, 5) << 2.0f,
                         0.0f,
                         std::numeric_limits<float>::quiet_NaN(),
                         4.0f,
                         5.0f);
  const cv::Mat sky = (cv::Mat_<uint8_t>(1, 5) << 0u, 0u, 0u, 255u, 0u);
  const cv::Mat baseline = VIO::makeMonoDepthValidMask(depth, sky);

  const auto disabled =
      VIO::makeMonoDepthConfidenceFilter(depth.size(), cv::Mat(), 0.0);
  EXPECT_TRUE(!disabled.filtering_enabled,
              "zero threshold disables confidence filtering");
  EXPECT_TRUE(disabled.accepted_pixels == depth.total() &&
                  disabled.rejected_pixels == 0u,
              "disabled filtering retains every confidence position");
  const cv::Mat with_disabled_filter =
      VIO::makeMonoDepthValidMask(depth, sky, disabled.mask);
  EXPECT_TRUE(
      cv::countNonZero(baseline != with_disabled_filter) == 0,
      "disabled filtering leaves single-view depth/sky validity unchanged");
  EXPECT_TRUE(cv::countNonZero(baseline) == 2,
              "baseline still rejects zero, NaN, and sky pixels");
  return EXIT_SUCCESS;
}

int testInvalidThresholdsAreRejected() {
  const cv::Size size(1, 1);
  bool negative_threw = false;
  try {
    VIO::makeMonoDepthConfidenceFilter(size, cv::Mat(), -0.1);
  } catch (const std::invalid_argument&) {
    negative_threw = true;
  }
  EXPECT_TRUE(negative_threw, "negative threshold is rejected");

  bool non_finite_threw = false;
  try {
    VIO::makeMonoDepthConfidenceFilter(
        size, cv::Mat(), std::numeric_limits<double>::infinity());
  } catch (const std::invalid_argument&) {
    non_finite_threw = true;
  }
  EXPECT_TRUE(non_finite_threw, "non-finite threshold is rejected");
  return EXIT_SUCCESS;
}

int testPoseScaleAlwaysStartsFromCanonicalDepth() {
  const cv::Mat canonical_depth(1, 2, CV_32FC1, cv::Scalar(2.0f));
  const cv::Mat first_refresh = VIO::scaleMonoDepthImage(canonical_depth, 3.0);
  const cv::Mat second_refresh = VIO::scaleMonoDepthImage(canonical_depth, 4.0);

  EXPECT_TRUE(!first_refresh.empty() && !second_refresh.empty(),
              "valid scales produce depth images");
  EXPECT_TRUE(std::abs(canonical_depth.at<float>(0, 0) - 2.0f) < 1e-6f,
              "refresh does not mutate canonical DA3 depth");
  EXPECT_TRUE(std::abs(first_refresh.at<float>(0, 0) - 6.0f) < 1e-6f,
              "first refresh applies its scale to canonical depth");
  EXPECT_TRUE(std::abs(second_refresh.at<float>(0, 0) - 8.0f) < 1e-6f,
              "later refresh replaces rather than compounds scale");
  EXPECT_TRUE(first_refresh.data != canonical_depth.data &&
                  second_refresh.data != canonical_depth.data,
              "scaled depth images own distinct storage");
  return EXIT_SUCCESS;
}

int testWeightImageUsesDa3Resolution() {
  cv::Mat depth(6, 8, CV_32FC1, cv::Scalar(4.0f));
  cv::Mat valid_mask(6, 8, CV_8UC1, cv::Scalar(255u));
  VIO::MonoDepthIntrinsics intrinsics;
  intrinsics.fx = 8.0;
  intrinsics.fy = 8.0;
  intrinsics.cx = 3.5;
  intrinsics.cy = 2.5;
  intrinsics.width = depth.cols;
  intrinsics.height = depth.rows;
  VIO::MonoDepthParams params;
  params.depth_weight_normal_radius = 1;
  params.min_depth_m = 0.1;
  params.max_depth_m = 20.0;

  const cv::Mat weights = VIO::makeMonoDepthWeightImageAtSize(
      depth, valid_mask, intrinsics, cv::Size(4, 3), params);
  EXPECT_TRUE(weights.type() == CV_32FC1,
              "downsampled weights retain float type");
  EXPECT_TRUE(weights.size() == cv::Size(4, 3),
              "weights are computed on the requested DA3 grid");
  EXPECT_TRUE(weights.at<float>(1, 1) > 0.0f,
              "valid planar depth produces a positive interior weight");

  valid_mask.setTo(cv::Scalar(0u));
  const cv::Mat rejected_weights = VIO::makeMonoDepthWeightImageAtSize(
      depth, valid_mask, intrinsics, cv::Size(4, 3), params);
  EXPECT_TRUE(cv::countNonZero(rejected_weights) == 0,
              "rejected full-resolution pixels remain zero on the weight grid");
  return EXIT_SUCCESS;
}

int testLowResolutionWeightSampling() {
  const cv::Mat weights = (cv::Mat_<float>(2, 2) << 0.1f, 0.2f, 0.3f, 0.4f);
  const cv::Size depth_size(4, 4);
  EXPECT_TRUE(std::abs(VIO::sampleMonoDepthWeight(weights, depth_size, 0, 0) -
                       0.1f) < 1e-6f,
              "top-left depth pixels map to the top-left weight");
  EXPECT_TRUE(std::abs(VIO::sampleMonoDepthWeight(weights, depth_size, 3, 0) -
                       0.2f) < 1e-6f,
              "top-right depth pixels map to the top-right weight");
  EXPECT_TRUE(std::abs(VIO::sampleMonoDepthWeight(weights, depth_size, 0, 3) -
                       0.3f) < 1e-6f,
              "bottom-left depth pixels map to the bottom-left weight");
  EXPECT_TRUE(std::abs(VIO::sampleMonoDepthWeight(weights, depth_size, 3, 3) -
                       0.4f) < 1e-6f,
              "bottom-right depth pixels map to the bottom-right weight");
  EXPECT_TRUE(std::abs(VIO::sampleMonoDepthWeight(cv::Mat(), depth_size, 2, 2) -
                       1.0f) < 1e-6f,
              "missing weights retain the legacy unit-weight fallback");
  return EXIT_SUCCESS;
}

int testPairDistanceGateUsesEndpointCameraChord() {
  const double pi = std::acos(-1.0);
  const gtsam::Pose3 world_T_context_cam(gtsam::Rot3::Rz(pi / 2.0),
                                         gtsam::Point3(1.0, 0.0, 0.0));
  const gtsam::Pose3 world_T_current_cam(gtsam::Rot3::Rz(pi),
                                         gtsam::Point3(0.0, 1.0, 0.0));
  const double chord = std::sqrt(2.0);

  const auto boundary = VIO::evaluateMonoDepthPairDistanceGate(
      world_T_context_cam, world_T_current_cam, chord);
  EXPECT_TRUE(boundary.valid && boundary.passes,
              "distance equal to the minimum threshold passes");
  EXPECT_TRUE(std::abs(boundary.camera_displacement_m - chord) < 1e-12,
              "distance gate uses endpoint camera-center chord");

  const auto too_close = VIO::evaluateMonoDepthPairDistanceGate(
      world_T_context_cam, world_T_current_cam, 1.5);
  EXPECT_TRUE(too_close.valid && !too_close.passes,
              "pair below the minimum camera displacement is held");

  const auto disabled = VIO::evaluateMonoDepthPairDistanceGate(
      gtsam::Pose3(), gtsam::Pose3(), 0.0);
  EXPECT_TRUE(disabled.valid && disabled.passes,
              "zero threshold disables distance filtering");

  const auto negative = VIO::evaluateMonoDepthPairDistanceGate(
      world_T_context_cam, world_T_current_cam, -1.0);
  EXPECT_TRUE(!negative.valid, "negative distance threshold is rejected");
  const auto non_finite = VIO::evaluateMonoDepthPairDistanceGate(
      world_T_context_cam,
      world_T_current_cam,
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(!non_finite.valid, "non-finite distance threshold is rejected");
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  int status = EXIT_SUCCESS;
  status |= testThresholdBoundaryAndNonFiniteValues();
  status |= testMissingAndMalformedConfidenceFailClosed();
  status |= testDisabledFilteringAndSingleViewBehavior();
  status |= testInvalidThresholdsAreRejected();
  status |= testPoseScaleAlwaysStartsFromCanonicalDepth();
  status |= testWeightImageUsesDa3Resolution();
  status |= testLowResolutionWeightSampling();
  status |= testPairDistanceGateUsesEndpointCameraChord();
  if (status == EXIT_SUCCESS) {
    std::cout << "All mono-depth confidence tests PASSED.\n";
  }
  return status;
}
