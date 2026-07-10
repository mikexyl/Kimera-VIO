#include <glog/logging.h>
#include <opencv2/core.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "kimera-vio/frontend/MonoDepthInference.h"

#define EXPECT_TRUE(condition, message)                                 \
  do {                                                                  \
    if (!(condition)) {                                                 \
      std::cerr << "[FAIL] " << (message) << "\n";                    \
      return EXIT_FAILURE;                                              \
    }                                                                   \
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
  const auto missing =
      VIO::makeMonoDepthConfidenceFilter(size, cv::Mat(), 1.1);
  EXPECT_TRUE(!missing.confidence_valid, "missing confidence is diagnosed");
  EXPECT_TRUE(missing.accepted_pixels == 0u &&
                  missing.rejected_pixels == 6u,
              "missing confidence rejects every pixel");
  EXPECT_TRUE(cv::countNonZero(missing.mask) == 0,
              "missing confidence produces an all-zero mask");

  const cv::Mat wrong_type(size, CV_8UC1, cv::Scalar(2));
  const auto malformed =
      VIO::makeMonoDepthConfidenceFilter(size, wrong_type, 1.1);
  EXPECT_TRUE(!malformed.confidence_valid,
              "non-float confidence is diagnosed");
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
  const cv::Mat depth =
      (cv::Mat_<float>(1, 5) << 2.0f,
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
  EXPECT_TRUE(cv::countNonZero(baseline != with_disabled_filter) == 0,
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

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  int status = EXIT_SUCCESS;
  status |= testThresholdBoundaryAndNonFiniteValues();
  status |= testMissingAndMalformedConfidenceFailClosed();
  status |= testDisabledFilteringAndSingleViewBehavior();
  status |= testInvalidThresholdsAreRejected();
  if (status == EXIT_SUCCESS) {
    std::cout << "All mono-depth confidence tests PASSED.\n";
  }
  return status;
}
