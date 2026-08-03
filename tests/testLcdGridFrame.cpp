#include <cstdlib>
#include <iostream>

#include "kimera-vio/loopclosure/LcdGridFrame.h"

namespace {

#define EXPECT_TRUE(condition, message)                                       \
  do {                                                                        \
    if (!(condition)) {                                                       \
      std::cerr << "FAILED: " << (message) << " (line " << __LINE__ << ")\n"; \
      return EXIT_FAILURE;                                                    \
    }                                                                         \
  } while (false)

VIO::LcdGridCell makeCell(float x, float y) {
  VIO::LcdGridCell cell;
  cell.descriptor = (cv::Mat_<float>(1, 2) << x, y);
  return cell;
}

int testEmptyFrame() {
  const VIO::LcdGridFrame frame(2, 2, 20, 20);
  EXPECT_TRUE(frame.computeDescriptorDiversityScore() == 0.0,
              "an empty frame has zero descriptor diversity");
  return EXIT_SUCCESS;
}

int testRepetitiveFrame() {
  VIO::LcdGridFrame frame(2, 2, 20, 20);
  frame.cell(0, 0) = makeCell(1.0f, 0.0f);
  frame.cell(0, 1) = makeCell(1.0f, 0.0f);
  frame.cell(1, 0) = makeCell(1.0f, 0.0f);
  frame.cell(1, 1) = makeCell(1.0f, 0.0f);
  EXPECT_TRUE(frame.computeDescriptorDiversityScore() == 0.0,
              "identical descriptors are rejected as self-similar");
  return EXIT_SUCCESS;
}

int testDiverseFrame() {
  VIO::LcdGridFrame frame(2, 2, 20, 20);
  frame.cell(0, 0) = makeCell(1.0f, 0.0f);
  frame.cell(0, 1) = makeCell(-1.0f, 0.0f);
  frame.cell(1, 0) = makeCell(0.0f, 1.0f);
  frame.cell(1, 1) = makeCell(0.0f, -1.0f);
  EXPECT_TRUE(frame.computeDescriptorDiversityScore() == 1.0,
              "well-separated descriptors saturate the diversity score");
  return EXIT_SUCCESS;
}

int testRawDescriptors() {
  const cv::Mat repetitive =
      (cv::Mat_<float>(4, 2) << 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f,
       0.0f);
  EXPECT_TRUE(
      VIO::LcdGridFrame::computeDescriptorDiversityScore(repetitive) == 0.0,
      "raw identical descriptors are rejected as self-similar");

  const cv::Mat diverse =
      (cv::Mat_<float>(4, 2) << 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
       -1.0f);
  EXPECT_TRUE(
      VIO::LcdGridFrame::computeDescriptorDiversityScore(diverse) == 1.0,
      "raw diverse descriptors saturate the diversity score");
  return EXIT_SUCCESS;
}

}  // namespace

int main() {
  if (testEmptyFrame() != EXIT_SUCCESS) return EXIT_FAILURE;
  if (testRepetitiveFrame() != EXIT_SUCCESS) return EXIT_FAILURE;
  if (testDiverseFrame() != EXIT_SUCCESS) return EXIT_FAILURE;
  if (testRawDescriptors() != EXIT_SUCCESS) return EXIT_FAILURE;
  std::cout << "All LcdGridFrame tests passed.\n";
  return EXIT_SUCCESS;
}
