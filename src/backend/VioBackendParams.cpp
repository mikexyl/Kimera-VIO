/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackendParams.cpp
 * @brief  Class parsing the parameters for the VIO's Backend from a YAML file.
 * @author Antoni Rosinol, Luca Carlone
 */

#include "kimera-vio/backend/VioBackendParams.h"

#include <utility>

namespace VIO {
namespace {

template <class T>
void getYamlParamIfPresent(const YamlParser& yaml_parser,
                           const std::string& id,
                           T* output) {
  if (yaml_parser.hasParam(id)) {
    yaml_parser.getYamlParam(id, output);
  }
}

}  // namespace

BackendParams::BackendParams() : PipelineParams("Backend Parameters") {
  // Trivial sanity checks.
  CHECK_GE(nr_states_, 0);
  CHECK_GE(numOptimize_, 0);
}

void BackendParams::setIsam2Params(const BackendParams& vio_params,
                                   gtsam::ISAM2Params* isam_param) {
  CHECK_NOTNULL(isam_param);
  // iSAM2 SETTINGS
  if (vio_params.useDogLeg_) {
    gtsam::ISAM2DoglegParams dogleg_params;
    dogleg_params.wildfireThreshold = vio_params.wildfire_threshold_;
    // dogleg_params.adaptationMode;
    // dogleg_params.initialDelta;
    // dogleg_params.setVerbose(false); // only for debugging.
    isam_param->optimizationParams = dogleg_params;
  } else {
    gtsam::ISAM2GaussNewtonParams gauss_newton_params;
    gauss_newton_params.wildfireThreshold = vio_params.wildfire_threshold_;
    isam_param->optimizationParams = gauss_newton_params;
  }

  // TODO (Toni): remove hardcoded
  // Cache Linearized Factors seems to improve performance.
  isam_param->cacheLinearizedFactors = true;
  isam_param->relinearizeThreshold = vio_params.relinearizeThreshold_;
  isam_param->relinearizeSkip = vio_params.relinearizeSkip_;
  isam_param->findUnusedFactorSlots = true;
  // isam_param->enablePartialRelinearizationCheck = true;
  isam_param->evaluateNonlinearError = false;  // only for debugging
  isam_param->enableDetailedResults = false;   // only for debugging.
  isam_param->factorization = gtsam::ISAM2Params::CHOLESKY;  // QR
}

bool BackendParams::equals(const BackendParams& vp2, double tol) const {
  return equalsVioBackendParams(vp2, tol);
}

void BackendParams::print() const { printVioBackendParams(); }

bool BackendParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  return parseYAMLVioBackendParams(yaml_parser);
}

bool BackendParams::parseYAMLVioBackendParams(const YamlParser& yaml_parser) {
  // INITIALIZATION
  yaml_parser.getYamlParam("autoInitialize", &autoInitialize_);
  yaml_parser.getYamlParam("roundOnAutoInitialize", &roundOnAutoInitialize_);
  yaml_parser.getYamlParam("initialPositionSigma", &initialPositionSigma_);
  yaml_parser.getYamlParam("initialRollPitchSigma", &initialRollPitchSigma_);
  yaml_parser.getYamlParam("initialYawSigma", &initialYawSigma_);
  yaml_parser.getYamlParam("initialVelocitySigma", &initialVelocitySigma_);
  yaml_parser.getYamlParam("initialAccBiasSigma", &initialAccBiasSigma_);
  yaml_parser.getYamlParam("initialGyroBiasSigma", &initialGyroBiasSigma_);

  // VISION PARAMS
  int linearization_mode_id;
  yaml_parser.getYamlParam("linearizationMode", &linearization_mode_id);
  switch (linearization_mode_id) {
    case 0:
      linearizationMode_ = gtsam::HESSIAN;
      break;
    case 1:
      linearizationMode_ = gtsam::IMPLICIT_SCHUR;
      break;
    case 2:
      linearizationMode_ = gtsam::JACOBIAN_Q;
      break;
    case 3:
      linearizationMode_ = gtsam::JACOBIAN_SVD;
      break;
    default:
      LOG(FATAL) << "Wrong linearizationMode in VIO Backend parameters.";
  }

  int degeneracy_mode_id;
  yaml_parser.getYamlParam("degeneracyMode", &degeneracy_mode_id);
  switch (degeneracy_mode_id) {
    case 0:
      degeneracyMode_ = gtsam::IGNORE_DEGENERACY;
      break;
    case 1:
      degeneracyMode_ = gtsam::ZERO_ON_DEGENERACY;
      break;
    case 2:
      degeneracyMode_ = gtsam::HANDLE_INFINITY;
      break;
    default:
      LOG(FATAL) << "Wrong degeneracyMode in VIO Backend parameters.";
  }

  yaml_parser.getYamlParam("smartNoiseSigma", &smartNoiseSigma_);
  yaml_parser.getYamlParam("rankTolerance", &rankTolerance_);
  yaml_parser.getYamlParam("landmarkDistanceThreshold",
                           &landmarkDistanceThreshold_);
  yaml_parser.getYamlParam("outlierRejection", &outlierRejection_);
  yaml_parser.getYamlParam("retriangulationThreshold",
                           &retriangulationThreshold_);
  yaml_parser.getYamlParam("addBetweenStereoFactors",
                           &addBetweenStereoFactors_);
  yaml_parser.getYamlParam("betweenRotationPrecision",
                           &betweenRotationPrecision_);
  yaml_parser.getYamlParam("betweenTranslationPrecision",
                           &betweenTranslationPrecision_);

  // OPTIMIZATION PARAMS
  yaml_parser.getYamlParam("relinearizeThreshold", &relinearizeThreshold_);
  yaml_parser.getYamlParam("relinearizeSkip", &relinearizeSkip_);
  yaml_parser.getYamlParam("zero_velocity_precision",
                           &zero_velocity_precision_);
  yaml_parser.getYamlParam("no_motion_position_precision",
                           &no_motion_position_precision_);
  yaml_parser.getYamlParam("no_motion_rotation_precision",
                           &no_motion_rotation_precision_);
  yaml_parser.getYamlParam("constant_vel_precision", &constant_vel_precision_);
  yaml_parser.getYamlParam("numOptimize", &numOptimize_);
  yaml_parser.getYamlParam("nr_states", &nr_states_);
  yaml_parser.getYamlParam("wildfire_threshold", &wildfire_threshold_);
  yaml_parser.getYamlParam("useDogLeg", &useDogLeg_);

  int pose_guess_source = 0;
  yaml_parser.getYamlParam("pose_guess_source", &pose_guess_source);
  switch (pose_guess_source) {
    case VIO::to_underlying(PoseGuessSource::IMU): {
      pose_guess_source_ = PoseGuessSource::IMU;
      break;
    }
    case VIO::to_underlying(PoseGuessSource::MONO): {
      pose_guess_source_ = PoseGuessSource::MONO;
      break;
    }
    case VIO::to_underlying(PoseGuessSource::STEREO): {
      pose_guess_source_ = PoseGuessSource::STEREO;
      break;
    }
    case VIO::to_underlying(PoseGuessSource::PNP): {
      pose_guess_source_ = PoseGuessSource::PNP;
      break;
    }
    case VIO::to_underlying(PoseGuessSource::EXTERNAL_ODOM): {
      pose_guess_source_ = PoseGuessSource::EXTERNAL_ODOM;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown PoseGuessSource : " << pose_guess_source;
      break;
    }
  }

  yaml_parser.getYamlParam("mono_translation_scale_factor",
                           &mono_translation_scale_factor_);

  yaml_parser.getYamlParam("min_num_obs_per_landmark_to_keep",
                           &min_num_obs_per_landmark_to_keep_);

  yaml_parser.getYamlParam("max_lmk_reproj_error_to_keep",
                           &max_lmk_reproj_error_to_keep_);

  getYamlParamIfPresent(
      yaml_parser, "vgicp_factors_enabled", &vgicp_factors_enabled_);
  getYamlParamIfPresent(
      yaml_parser, "vgicp_icp_only_enabled", &vgicp_icp_only_enabled_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_icp_only_da3_overlap_fusion",
                        &vgicp_icp_only_da3_overlap_fusion_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_icp_only_max_iterations",
                        &vgicp_icp_only_max_iterations_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_use_weighted_icp_factor",
                        &vgicp_use_weighted_icp_factor_);
  getYamlParamIfPresent(
      yaml_parser, "vgicp_min_shared_tracks", &vgicp_min_shared_tracks_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_max_edges_per_keyframe",
                        &vgicp_max_edges_per_keyframe_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_downsample_resolution",
                        &vgicp_downsample_resolution_);
  getYamlParamIfPresent(
      yaml_parser, "vgicp_voxel_resolution", &vgicp_voxel_resolution_);
  getYamlParamIfPresent(
      yaml_parser, "vgicp_covariance_neighbors", &vgicp_covariance_neighbors_);
  getYamlParamIfPresent(yaml_parser, "vgicp_num_threads", &vgicp_num_threads_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_min_points_per_keyframe",
                        &vgicp_min_points_per_keyframe_);
  getYamlParamIfPresent(yaml_parser,
                        "vgicp_max_correspondence_distance",
                        &vgicp_max_correspondence_distance_);
  getYamlParamIfPresent(
      yaml_parser, "vgicp_factor_weight", &vgicp_factor_weight_);

  CHECK(!vgicp_icp_only_enabled_ || vgicp_factors_enabled_)
      << "vgicp_icp_only_enabled requires vgicp_factors_enabled";
  CHECK(!vgicp_icp_only_da3_overlap_fusion_ || vgicp_icp_only_enabled_)
      << "vgicp_icp_only_da3_overlap_fusion requires "
         "vgicp_icp_only_enabled";
  CHECK_GT(vgicp_icp_only_max_iterations_, 0);
  CHECK_GE(vgicp_min_shared_tracks_, 0);
  CHECK_GE(vgicp_max_edges_per_keyframe_, 0);
  CHECK_GT(vgicp_downsample_resolution_, 0.0);
  CHECK_GT(vgicp_voxel_resolution_, 0.0);
  CHECK_GT(vgicp_covariance_neighbors_, 0);
  CHECK_GT(vgicp_num_threads_, 0);
  CHECK_GE(vgicp_min_points_per_keyframe_, 0);
  CHECK_GT(vgicp_factor_weight_, 0.0);

  return true;
}

bool BackendParams::equalsVioBackendParams(const BackendParams& vp2,
                                           double tol) const {
  return
      // INITIALIZATION
      (autoInitialize_ == vp2.autoInitialize_) &&
      initial_ground_truth_state_.equals(vp2.initial_ground_truth_state_) &&
      (roundOnAutoInitialize_ == vp2.roundOnAutoInitialize_) &&
      (fabs(initialPositionSigma_ - vp2.initialPositionSigma_) <= tol) &&
      (fabs(initialRollPitchSigma_ - vp2.initialRollPitchSigma_) <= tol) &&
      (fabs(initialYawSigma_ - vp2.initialYawSigma_) <= tol) &&
      (fabs(initialVelocitySigma_ - vp2.initialVelocitySigma_) <= tol) &&
      (fabs(initialAccBiasSigma_ - vp2.initialAccBiasSigma_) <= tol) &&
      (fabs(initialGyroBiasSigma_ - vp2.initialGyroBiasSigma_) <= tol) &&
      // VISION PARAMS
      (linearizationMode_ == vp2.linearizationMode_) &&
      (degeneracyMode_ == vp2.degeneracyMode_) &&
      (fabs(smartNoiseSigma_ - vp2.smartNoiseSigma_) <= tol) &&
      (fabs(rankTolerance_ - vp2.rankTolerance_) <= tol) &&
      (fabs(landmarkDistanceThreshold_ - vp2.landmarkDistanceThreshold_) <=
       tol) &&
      (fabs(outlierRejection_ - vp2.outlierRejection_) <= tol) &&
      (fabs(retriangulationThreshold_ - vp2.retriangulationThreshold_) <=
       tol) &&
      (addBetweenStereoFactors_ == vp2.addBetweenStereoFactors_) &&
      (fabs(betweenRotationPrecision_ - vp2.betweenRotationPrecision_) <=
       tol) &&
      (fabs(betweenTranslationPrecision_ - vp2.betweenTranslationPrecision_) <=
       tol) &&
      // OPTIMIZATION PARAMS
      (fabs(relinearizeThreshold_ - vp2.relinearizeThreshold_) <= tol) &&
      (relinearizeSkip_ == vp2.relinearizeSkip_) &&
      (fabs(zero_velocity_precision_ - vp2.zero_velocity_precision_) <= tol) &&
      (fabs(no_motion_position_precision_ -
            vp2.no_motion_position_precision_) <= tol) &&
      (fabs(no_motion_rotation_precision_ -
            vp2.no_motion_rotation_precision_) <= tol) &&
      (fabs(constant_vel_precision_ - vp2.constant_vel_precision_) <= tol) &&
      (numOptimize_ == vp2.numOptimize_) && (nr_states_ == vp2.nr_states_) &&
      (wildfire_threshold_ == vp2.wildfire_threshold_) &&
      (useDogLeg_ == vp2.useDogLeg_) &&
      (pose_guess_source_ == vp2.pose_guess_source_) &&
      (fabs(mono_translation_scale_factor_ -
            vp2.mono_translation_scale_factor_) <= tol) &&
      (min_num_obs_per_landmark_to_keep_ ==
       vp2.min_num_obs_per_landmark_to_keep_) &&
      (max_lmk_reproj_error_to_keep_ == vp2.max_lmk_reproj_error_to_keep_) &&
      (vgicp_factors_enabled_ == vp2.vgicp_factors_enabled_) &&
      (vgicp_icp_only_enabled_ == vp2.vgicp_icp_only_enabled_) &&
      (vgicp_icp_only_da3_overlap_fusion_ ==
       vp2.vgicp_icp_only_da3_overlap_fusion_) &&
      (vgicp_icp_only_max_iterations_ == vp2.vgicp_icp_only_max_iterations_) &&
      (vgicp_use_weighted_icp_factor_ == vp2.vgicp_use_weighted_icp_factor_) &&
      (vgicp_min_shared_tracks_ == vp2.vgicp_min_shared_tracks_) &&
      (vgicp_max_edges_per_keyframe_ == vp2.vgicp_max_edges_per_keyframe_) &&
      (fabs(vgicp_downsample_resolution_ - vp2.vgicp_downsample_resolution_) <=
       tol) &&
      (fabs(vgicp_voxel_resolution_ - vp2.vgicp_voxel_resolution_) <= tol) &&
      (vgicp_covariance_neighbors_ == vp2.vgicp_covariance_neighbors_) &&
      (vgicp_num_threads_ == vp2.vgicp_num_threads_) &&
      (vgicp_min_points_per_keyframe_ == vp2.vgicp_min_points_per_keyframe_) &&
      (fabs(vgicp_max_correspondence_distance_ -
            vp2.vgicp_max_correspondence_distance_) <= tol) &&
      (fabs(vgicp_factor_weight_ - vp2.vgicp_factor_weight_) <= tol);
}

void BackendParams::printVioBackendParams() const {
  static const int kCenter = PipelineParams::kTotalWidth / 2;
  std::stringstream out;
  PipelineParams::print(
      out,
      std::string(kCenter, '.') + "** Initialization parameters **",
      "",
      "Automatic Initialization",
      autoInitialize_,
      "Round on autoinit",
      roundOnAutoInitialize_,
      "Sigmas:",
      "",
      "Initial Position Sigma",
      initialPositionSigma_,
      "Initiail Roll Pitch Sigma",
      initialRollPitchSigma_,
      "Initial Yaw Sigma",
      initialYawSigma_,
      "Initial Velocity Sigma",
      initialVelocitySigma_,
      "Initial Acc Bias Sigma",
      initialAccBiasSigma_,
      "Initial Gyro Bias Sigma",
      initialGyroBiasSigma_,
      std::string(kCenter, '.') + "** Vision parameters **",
      "",
      "Linearization Mode: hessian, implicit_schur, jacobian_q, jacobian_svd",
      linearizationMode_,
      "Degeneracy Mode: ignore_degeneracy, zero_on_degeneracy, handle_infinity",
      degeneracyMode_,
      "Rank Tolerance",
      rankTolerance_,
      "Landmark Distance Threshold",
      landmarkDistanceThreshold_,
      "Outlier Rejection",
      outlierRejection_,
      "Retriangulation Threshold",
      retriangulationThreshold_,
      "Add Btw Stereo Factors",
      addBetweenStereoFactors_,
      "Btw Rotation Precision",
      betweenRotationPrecision_,
      "Btw Translation Precision",
      betweenTranslationPrecision_,
      std::string(kCenter, '.') + "** Optimization parameters **",
      "",
      "Relinearize Threshold",
      relinearizeThreshold_,
      "Relinearize Skip",
      relinearizeSkip_,
      "Zero Velocity Precision",
      zero_velocity_precision_,
      "No Motion Position Precision",
      no_motion_position_precision_,
      "No Motion Rotation Precision",
      no_motion_rotation_precision_,
      "Constant Velocity Precision",
      constant_vel_precision_,
      "Optimization Iterations",
      numOptimize_,
      "nr_states",
      nr_states_,
      "Isam Wildfire Threshold",
      wildfire_threshold_,
      "Use Dog Leg",
      useDogLeg_,
      "Pose Guess Source",
      VIO::to_underlying(pose_guess_source_),
      "Mono Translation Scale Factor",
      mono_translation_scale_factor_,
      std::string(kCenter, '.') + "** VGICP factor parameters **",
      "",
      "VGICP Factors Enabled",
      vgicp_factors_enabled_,
      "VGICP ICP-only Diagnostic Enabled",
      vgicp_icp_only_enabled_,
      "ICP-only DA3 Overlap Fusion",
      vgicp_icp_only_da3_overlap_fusion_,
      "VGICP ICP-only Max Iterations",
      vgicp_icp_only_max_iterations_,
      "VGICP Use Weighted ICP Factor",
      vgicp_use_weighted_icp_factor_,
      "VGICP Min Shared Tracks",
      vgicp_min_shared_tracks_,
      "VGICP Max Edges Per Keyframe",
      vgicp_max_edges_per_keyframe_,
      "VGICP Downsample Resolution",
      vgicp_downsample_resolution_,
      "VGICP Voxel Resolution",
      vgicp_voxel_resolution_,
      "VGICP Covariance Neighbors",
      vgicp_covariance_neighbors_,
      "VGICP Num Threads",
      vgicp_num_threads_,
      "VGICP Min Points Per Keyframe",
      vgicp_min_points_per_keyframe_,
      "VGICP Max Correspondence Distance",
      vgicp_max_correspondence_distance_,
      "VGICP Total Factor Weight",
      vgicp_factor_weight_);
  LOG(INFO) << out.str();
  LOG(INFO) << "** Backend Iinitialization Parameters **\n"
            << "initial_ground_truth_state_: ";
  initial_ground_truth_state_.print();
}

}  // namespace VIO
