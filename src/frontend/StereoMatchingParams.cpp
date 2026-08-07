/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatchingParams.cpp
 * @brief  Parameters for stereo matching.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/StereoMatchingParams.h"

#include <glog/logging.h>

#include <filesystem>

#include "kimera-vio/frontend/StereoFrame-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"
#include "kimera-vio/utils/YamlParser.h"

namespace VIO {

StereoMatchingParams::StereoMatchingParams()
    : PipelineParams("Stereo Matching Parameters") {
  checkParams();
}

void StereoMatchingParams::checkParams() {
  CHECK(!(templ_cols_ % 2 != 1 ||
          templ_rows_ % 2 != 1))  // check that they are odd
      << "StereoMatchingParams: template size must be odd!";
  CHECK(!(stripe_extra_rows_ % 2 != 0))  // check that they are even
      << "StereoMatchingParams: stripe_extra_rows size must be even!";
}

bool StereoMatchingParams::equals(const StereoMatchingParams& tp2,
                                  double tol) const {
  return (fabs(nominal_baseline_ - tp2.nominal_baseline_) <= tol) &&
         (equalize_image_ == tp2.equalize_image_) &&
         (fabs(tolerance_template_matching_ -
               tp2.tolerance_template_matching_) <= tol) &&
         (templ_cols_ == tp2.templ_cols_) && (templ_rows_ == tp2.templ_rows_) &&
         (stripe_extra_rows_ == tp2.stripe_extra_rows_) &&
         (fabs(min_point_dist_ - tp2.min_point_dist_) <= tol) &&
         (fabs(max_point_dist_ - tp2.max_point_dist_) <= tol) &&
         (bidirectional_matching_ == tp2.bidirectional_matching_) &&
         (subpixel_refinement_ == tp2.subpixel_refinement_);
}

void StereoMatchingParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        "equalize_image_: ",
                        equalize_image_,
                        "nominalBaseline_: ",
                        nominal_baseline_,
                        "toleranceTemplateMatching_: ",
                        tolerance_template_matching_,
                        "templ_cols_: ",
                        templ_cols_,
                        "templ_rows_: ",
                        templ_rows_,
                        "stripe_extra_rows_: ",
                        stripe_extra_rows_,
                        "minPointDist_: ",
                        min_point_dist_,
                        "maxPointDist_: ",
                        max_point_dist_,
                        "bidirectionalMatching_: ",
                        bidirectional_matching_,
                        "subpixelRefinementStereo_: ",
                        subpixel_refinement_);
  LOG(INFO) << out.str();
}

bool StereoMatchingParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  yaml_parser.getYamlParam("equalizeImage", &equalize_image_);
  yaml_parser.getYamlParam("nominalBaseline", &nominal_baseline_);
  yaml_parser.getYamlParam("toleranceTemplateMatching",
                           &tolerance_template_matching_);
  yaml_parser.getYamlParam("templ_cols", &templ_cols_);
  yaml_parser.getYamlParam("templ_rows", &templ_rows_);
  yaml_parser.getYamlParam("stripe_extra_rows", &stripe_extra_rows_);
  yaml_parser.getYamlParam("minPointDist", &min_point_dist_);
  yaml_parser.getYamlParam("maxPointDist", &max_point_dist_);
  yaml_parser.getYamlParam("bidirectionalMatching", &bidirectional_matching_);
  yaml_parser.getYamlParam("subpixelRefinementStereo", &subpixel_refinement_);
  yaml_parser.getYamlParam("rectifiedInputs", &rectified_inputs);

  dense_stereo_params_.parseYAML(filepath);
  return true;
}

bool DenseStereoParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);

  // Parse stereo depth method
  if (yaml_parser.hasParam("stereoDepthMethod")) {
    std::string method_str;
    yaml_parser.getYamlParam("stereoDepthMethod", &method_str);
    stereo_depth_method_ = stereoDepthMethodFromString(method_str);
  }

  // Parse other dense stereo parameters
  if (yaml_parser.hasParam("useSGBM")) {
    yaml_parser.getYamlParam("useSGBM", &use_sgbm_);
  }
  if (yaml_parser.hasParam("postFilterDisparity")) {
    yaml_parser.getYamlParam("postFilterDisparity", &post_filter_disparity_);
  }
  if (yaml_parser.hasParam("medianBlurDisparity")) {
    yaml_parser.getYamlParam("medianBlurDisparity", &median_blur_disparity_);
  }
  if (yaml_parser.hasParam("preFilterCap")) {
    yaml_parser.getYamlParam("preFilterCap", &pre_filter_cap_);
  }
  if (yaml_parser.hasParam("sadWindowSize")) {
    yaml_parser.getYamlParam("sadWindowSize", &sad_window_size_);
  }
  if (yaml_parser.hasParam("minDisparity")) {
    yaml_parser.getYamlParam("minDisparity", &min_disparity_);
  }
  if (yaml_parser.hasParam("numDisparities")) {
    yaml_parser.getYamlParam("numDisparities", &num_disparities_);
  }
  if (yaml_parser.hasParam("uniquenessRatio")) {
    yaml_parser.getYamlParam("uniquenessRatio", &uniqueness_ratio_);
  }
  if (yaml_parser.hasParam("speckleRange")) {
    yaml_parser.getYamlParam("speckleRange", &speckle_range_);
  }
  if (yaml_parser.hasParam("speckleWindowSize")) {
    yaml_parser.getYamlParam("speckleWindowSize", &speckle_window_size_);
  }
  if (yaml_parser.hasParam("textureThreshold")) {
    yaml_parser.getYamlParam("textureThreshold", &texture_threshold_);
  }
  if (yaml_parser.hasParam("preFilterType")) {
    yaml_parser.getYamlParam("preFilterType", &pre_filter_type_);
  }
  if (yaml_parser.hasParam("preFilterSize")) {
    yaml_parser.getYamlParam("preFilterSize", &pre_filter_size_);
  }
  if (yaml_parser.hasParam("P1")) {
    yaml_parser.getYamlParam("P1", &p1_);
  }
  if (yaml_parser.hasParam("P2")) {
    yaml_parser.getYamlParam("P2", &p2_);
  }
  if (yaml_parser.hasParam("disp12MaxDiff")) {
    yaml_parser.getYamlParam("disp12MaxDiff", &disp_12_max_diff_);
  }
  if (yaml_parser.hasParam("useModeHH")) {
    yaml_parser.getYamlParam("useModeHH", &use_mode_HH_);
  }
  if (yaml_parser.hasParam("vpiMinDisparity")) {
    yaml_parser.getYamlParam("vpiMinDisparity", &vpi_min_disparity_);
  }
  if (yaml_parser.hasParam("vpiMinValidDisparity")) {
    yaml_parser.getYamlParam("vpiMinValidDisparity", &vpi_min_valid_disparity_);
  }
  if (yaml_parser.hasParam("vpiMaxDisparity")) {
    yaml_parser.getYamlParam("vpiMaxDisparity", &vpi_max_disparity_);
  }
  if (yaml_parser.hasParam("vpiP1")) {
    yaml_parser.getYamlParam("vpiP1", &vpi_p1_);
  }
  if (yaml_parser.hasParam("vpiP2")) {
    yaml_parser.getYamlParam("vpiP2", &vpi_p2_);
  }
  if (yaml_parser.hasParam("vpiConfidenceThreshold")) {
    yaml_parser.getYamlParam("vpiConfidenceThreshold",
                             &vpi_confidence_threshold_);
  }
  if (yaml_parser.hasParam("vpiUniqueness")) {
    yaml_parser.getYamlParam("vpiUniqueness", &vpi_uniqueness_);
  }
  if (yaml_parser.hasParam("vpiIncludeDiagonals")) {
    yaml_parser.getYamlParam("vpiIncludeDiagonals", &vpi_include_diagonals_);
  }
  if (yaml_parser.hasParam("enginePath")) {
    yaml_parser.getYamlParam("enginePath", &engine_path_);
  }
  if (yaml_parser.hasParam("ffsMaxDisparity")) {
    yaml_parser.getYamlParam("ffsMaxDisparity", &ffs_max_disparity_);
  }
  if (yaml_parser.hasParam("stereoWarmupIterations")) {
    yaml_parser.getYamlParam("stereoWarmupIterations",
                             &stereo_warmup_iterations_);
  }
  if (yaml_parser.hasParam("dispHeight")) {
    yaml_parser.getYamlParam("dispHeight", &disp_height_);
  }
  if (yaml_parser.hasParam("dispWidth")) {
    yaml_parser.getYamlParam("dispWidth", &disp_width_);
  }
  CHECK_GT(ffs_max_disparity_, 0);
  CHECK_GE(stereo_warmup_iterations_, 0);
  CHECK_GE(vpi_min_disparity_, 0);
  CHECK_GE(vpi_min_valid_disparity_, vpi_min_disparity_);
  CHECK_GT(vpi_max_disparity_, vpi_min_disparity_);
  CHECK_GT(vpi_max_disparity_, vpi_min_valid_disparity_);
  CHECK_LE(vpi_max_disparity_, 256);
  CHECK_GT(vpi_p1_, 0);
  CHECK_GE(vpi_p2_, vpi_p1_);
  CHECK_LT(vpi_p2_, 256);
  CHECK_GE(vpi_confidence_threshold_, 0);
  CHECK_LE(vpi_confidence_threshold_, 65535);
  CHECK(vpi_uniqueness_ == -1.0 ||
        (vpi_uniqueness_ >= 0.0 && vpi_uniqueness_ <= 1.0));
  if (stereo_depth_method_ == StereoDepthMethod::FAST_FOUNDATION_STEREO) {
    CHECK(!engine_path_.empty())
        << stereoDepthMethodToString(stereo_depth_method_)
        << " requires enginePath";
    CHECK_EQ(std::filesystem::path(engine_path_).extension(), ".engine")
        << stereoDepthMethodToString(stereo_depth_method_)
        << " requires a TensorRT .engine file: " << engine_path_;
  }
  return true;
}

}  // namespace VIO
