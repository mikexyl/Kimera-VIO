#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "kimera-vio/frontend/StereoMatchingParams.h"

int main() {
  const VIO::DenseStereoParams params;
  if (params.stereo_depth_method_ != VIO::StereoDepthMethod::OPENCV_SGBM) {
    std::cerr << "OpenCV SGBM is not the default stereo backend\n";
    return EXIT_FAILURE;
  }

  if (params.vpi_min_disparity_ != 0 || params.vpi_min_valid_disparity_ != 0 ||
      params.vpi_max_disparity_ != 128 || params.vpi_p1_ != 3 ||
      params.vpi_p2_ != 48 || params.vpi_confidence_threshold_ != 55535) {
    std::cerr << "VPI CUDA defaults do not match NVIDIA's robust profile\n";
    return EXIT_FAILURE;
  }

  if (VIO::stereoDepthMethodFromString("OpenCV_SGBM") !=
      VIO::StereoDepthMethod::OPENCV_SGBM) {
    std::cerr << "OpenCV SGBM parser round trip failed\n";
    return EXIT_FAILURE;
  }

  if (VIO::stereoDepthMethodFromString("VPI_CUDA") !=
          VIO::StereoDepthMethod::VPI_CUDA ||
      VIO::stereoDepthMethodFromString("VPI") !=
          VIO::StereoDepthMethod::VPI_CUDA ||
      std::string(VIO::stereoDepthMethodToString(
          VIO::StereoDepthMethod::VPI_CUDA)) != "VPI_CUDA") {
    std::cerr << "VPI CUDA parser round trip failed\n";
    return EXIT_FAILURE;
  }

  try {
    static_cast<void>(VIO::stereoDepthMethodFromString("LibSGM"));
  } catch (const std::runtime_error&) {
    return EXIT_SUCCESS;
  }

  std::cerr << "Removed LibSGM backend was unexpectedly accepted\n";
  return EXIT_FAILURE;
}
