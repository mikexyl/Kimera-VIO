#pragma once

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_bool(lcd_no_optimize);
DECLARE_bool(lcd_no_detection);

namespace VIO {

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    LoopClosureDetector(
        const LoopClosureDetectorParams& lcd_params,
        const CameraParams& tracker_cam_params,
        const gtsam::Pose3& B_Pose_Cam,
        const std::optional<VIO::StereoCamera::ConstPtr>& stereo_camera,
        const std::optional<StereoMatchingParams>& stereo_matching_params,
        const std::optional<VIO::RgbdCamera::ConstPtr>& rgbd_camera,
        bool log_output)
    : lcd_state_(LcdState::Bootstrap),
      lcd_params_(lcd_params),
      feature_detector_(nullptr),
      feature_matcher_(nullptr),
      stereo_camera_(stereo_camera ? stereo_camera.value() : nullptr),
      stereo_matcher_(nullptr),
      rgbd_camera_(rgbd_camera ? rgbd_camera.value() : nullptr),
      db_(nullptr),
      cache_(lcd_params.frame_cache),
      pgo_(nullptr),
      W_Pose_B_kf_vio_(),
      B_Pose_Cam_(B_Pose_Cam),
      latest_global_vec_(nullptr),
      tracker_(nullptr),
      num_lc_unoptimized_(0),
      lcd_tp_wrapper_(nullptr),
      logger_(nullptr),
      log_output_(log_output) {
  // Shared noise model initialization
  gtsam::Vector6 precisions;
  precisions.head<3>().setConstant(lcd_params_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(lcd_params_.betweenTranslationPrecision_);
  shared_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  // Outlier rejection initialization (inside of tracker)
  tracker_ = std::make_unique<Tracker>(
      lcd_params.tracker_params_,
      std::make_shared<VIO::Camera>(tracker_cam_params));

  // Initialize pgo_:
  // TODO(marcus): parametrize the verbosity of PGO params
  KimeraRPGO::RobustSolverParams pgo_params;
  // TODO(mikexyl): turn this off for debugging
  //  pgo_params.setNoRejection();
  pgo_params.setPcmSimple3DParams(lcd_params_.odom_trans_threshold_,
                                  lcd_params_.odom_rot_threshold_,
                                  lcd_params_.pcm_trans_threshold_,
                                  lcd_params_.pcm_rot_threshold_,
                                  KimeraRPGO::Verbosity::QUIET);
  if (lcd_params_.gnc_alpha_ > 0 && lcd_params_.gnc_alpha_ < 1) {
    pgo_params.setGncInlierCostThresholdsAtProbability(lcd_params_.gnc_alpha_);
  }
  pgo_ = std::make_unique<KimeraRPGO::RobustSolver>(pgo_params);

  // Initialize the thirdparty wrapper:
  lcd_tp_wrapper_ = std::make_unique<LcdThirdPartyWrapper>(lcd_params_);

  landmark_manager_ = std::make_unique<LcdLandmarkManager>();

  if (log_output) {
    logger_ = std::make_unique<LoopClosureDetectorLogger>();
  }

  if (VLOG_IS_ON(1)) {
    print();
  }
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
LcdOutput::UniquePtr
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::spinOnce(
    const LcdInput& input) {
  CHECK_GE(input.cur_kf_id_, 0);
  CHECK(feature_detector_);
  CHECK(feature_matcher_);
  CHECK(landmark_manager_);
  CHECK(db_);

  landmark_manager_->updateLandmarks(input.W_points_with_ids_);
  std::vector<LandmarkId> new_lmk_ids;
  for (const auto& [lmk_id, lmk] : input.W_points_with_ids_) {
    new_lmk_ids.push_back(lmk_id);
  }
  int culled = landmark_manager_->checkAndCullingLandmarks(
      new_lmk_ids,
      cache_,
      lcd_params_.min_lmk_obs_ratio_,
      lcd_params_.min_lmk_parallex_);
  VLOG(1) << "Culled landmarks: " << culled;

  // Update the PGO with the Backend VIO estimate.
  // TODO(marcus): only add factor if it's a set distance away from previous
  // TODO(marcus): OdometryPose vs OdometryFactor
  timestamp_map_[input.cur_kf_id_] = input.timestamp_;
  OdometryFactor odom_factor(
      input.cur_kf_id_, input.W_Pose_Blkf_, shared_noise_model_);

  switch (lcd_state_) {
    case LcdState::Bootstrap: {
      CHECK_EQ(pgo_->calculateEstimate().size(), 0);
      initializePGO(odom_factor);
      break;
    }
    case LcdState::Nominal: {
      // TODO(marcus): need a better check than this:
      CHECK_GT(pgo_->calculateEstimate().size(), 0);
      addOdometryFactorAndOptimize(odom_factor);
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized LCD state.";
    }
  }

  // Process the StereoFrame and check for a loop closure with previous
  // ones.
  FrameId lcd_frame_id;
  switch (input.frontend_output_->frontend_type_) {
    case FrontendType::kMonoImu: {
      auto mono_frontend_output =
          std::dynamic_pointer_cast<MonoFrontendOutput>(input.frontend_output_);
      CHECK(mono_frontend_output);
      if (lcd_params_.pose_recovery_type_ == PoseRecoveryType::kPnP ||
          lcd_params_.pose_recovery_type_ == PoseRecoveryType::k5ptRotOnly) {
        lcd_frame_id = processAndAddMonoFrame(mono_frontend_output->frame_lkf_,
                                              input.W_points_with_ids_,
                                              input.W_Pose_Blkf_);
      } else {
        LOG(FATAL) << "We have a mono frontend but no PnP pose recovery in LCD "
                      "module. Must be a mistake!";
      }
      break;
    }
    case FrontendType::kStereoImu: {
      auto stereo_frontend_output =
          std::dynamic_pointer_cast<StereoFrontendOutput>(
              input.frontend_output_);
      CHECK(stereo_frontend_output);
      lcd_frame_id =
          processAndAddStereoFrame(stereo_frontend_output->stereo_frame_lkf_);
      break;
    }
    case FrontendType::kRgbdImu: {
      auto rgbd_frontend_output =
          std::dynamic_pointer_cast<RgbdFrontendOutput>(input.frontend_output_);
      CHECK(rgbd_frontend_output);
      lcd_frame_id =
          processAndAddRgbdFrame(rgbd_frontend_output->rgbd_frame_lkf_);
      break;
    }
    default: {
      LOG(FATAL)
          << "LoopClosureDetector not implemented for this frontend type.";
    }
  }

  const auto curr_frame = cache_.getFrame(lcd_frame_id);
  CHECK(curr_frame) << "Invalid frame requested!";

  landmark_manager_->updateObsFrames(curr_frame->id_, curr_frame->landmark_ids);

  typename Database::GlobalDesc curr_bow_vec;
  db_->transform(curr_frame->descriptors_vec_, curr_bow_vec);

  LoopResult loop_result;
  loop_result.status_ = LCDStatus::NO_MATCHES;
  if (!FLAGS_lcd_no_detection) {
    detectLoop(lcd_frame_id, curr_bow_vec, &loop_result);
  }

  db_->add(curr_bow_vec);

  // Update latest bowvec for normalized similarity scoring (NSS).
  if (static_cast<int>(lcd_frame_id + 1) > lcd_params_.recent_frames_window_) {
    latest_global_vec_.reset(new typename Database::GlobalDesc(curr_bow_vec));
  } else {
    VLOG(3) << "LoopClosureDetector: Not enough frames processed.";
  }

  // Build and add LC factor if result is a loop closure.
  if (loop_result.isLoop()) {
    VLOG(1) << "LoopClosureDetector: LOOP CLOSURE detected from keyframe "
            << loop_result.match_id_ << " to keyframe "
            << loop_result.query_id_;

    utils::StatsCollector stat_pgo_timing(
        "PGO Update/Optimization Timing [ms]");
    auto tic = utils::Timer::tic();

    if (loop_result.status_ == LCDStatus::LOOP_DETECTED_ROT) {
      // Rotation part of the information matrix of the noise model
      // emphasized.
      gtsam::Matrix mat_info = gtsam::Matrix::Identity(6, 6);
      gtsam::Matrix mat_info_rotation_part =
          lcd_params_.betweenRotationPrecision_ * gtsam::Matrix::Identity(3, 3);
      mat_info.block<3, 3>(0, 0) = (mat_info_rotation_part);

      // Zero out the translation part of the noise model to only use the
      // 2d2d pose for the loop closure factor. mat_info.block<3, 3>(3, 3) =
      // gtsam::Matrix::Identity(3, 3) * 0.0;
      mat_info.block<3, 3>(3, 3) = gtsam::Matrix::Identity(3, 3) * 1e-12;

      // Instantiate a noise model from the rotation-only information
      // matrix.
      gtsam::SharedNoiseModel noise_model_5pt_rotation_only =
          gtsam::noiseModel::Diagonal::Information(mat_info);

      // Refresh timer because all previous stuff irrelevant to PGO timing.
      tic = utils::Timer::tic();
      addLoopClosureFactorAndOptimize(
          LoopClosureFactor(loop_result.match_id_,
                            loop_result.query_id_,
                            loop_result.relative_pose_,
                            noise_model_5pt_rotation_only));
    } else if (loop_result.status_ == LCDStatus::LOOP_DETECTED) {
      addLoopClosureFactorAndOptimize(
          LoopClosureFactor(loop_result.match_id_,
                            loop_result.query_id_,
                            loop_result.relative_pose_,
                            shared_noise_model_));
    }

    auto update_duration = utils::Timer::toc(tic).count();
    stat_pgo_timing.AddSample(update_duration);
  } else {
    VLOG(2) << "LoopClosureDetector: No loop closure detected. Reason: "
            << LoopResult::asString(loop_result.status_);
  }

  // Timestamps for PGO and for LCD should match now.
  CHECK_EQ(curr_frame->timestamp_, timestamp_map_.at(curr_frame->id_));
  CHECK_EQ(timestamp_map_.size(), cache_.size());
  CHECK_EQ(timestamp_map_.size(), W_Pose_B_kf_vio_.first + 1);

  // Construct output payload.
  CHECK(pgo_);
  const gtsam::Pose3& w_Pose_map = getWPoseMap();
  const gtsam::Pose3& map_Pose_odom = getMapPoseOdom();
  const gtsam::Values& pgo_states = pgo_->calculateEstimate();
  const gtsam::NonlinearFactorGraph& pgo_nfg = pgo_->getFactorsUnsafe();

  LcdOutput::UniquePtr output_payload = nullptr;
  if (loop_result.isLoop()) {
    output_payload =
        std::make_unique<LcdOutput>(loop_result.status_,
                                    input.timestamp_,
                                    timestamp_map_.at(loop_result.query_id_),
                                    timestamp_map_.at(loop_result.match_id_),
                                    loop_result.match_id_,
                                    loop_result.query_id_,
                                    loop_result.relative_pose_);
  } else {
    output_payload =
        std::make_unique<LcdOutput>(loop_result.status_, input.timestamp_);
  }

  CHECK(output_payload) << "Missing LCD output payload.";

  output_payload->setMapInformation(
      w_Pose_map, map_Pose_odom, pgo_states, pgo_nfg);

  output_payload->setFrameInformation(curr_frame->keypoints_3d_,
                                      curr_frame->bearing_vectors_,
                                      globalDescToMap(curr_bow_vec),
                                      curr_frame->descriptors_mat_);
  output_payload->landmarks_ = landmark_manager_->getLandmarks();
  output_payload->timestamp_map_ = timestamp_map_;
  output_payload->covis_graph_ = landmark_manager_->getCovisGraph();

  cleanFrame(lcd_frame_id);

  if (logger_) {
    debug_info_.timestamp_ = output_payload->timestamp_;
    debug_info_.loop_result_ = loop_result;
    debug_info_.pgo_size_ = pgo_->size();
    debug_info_.pgo_lc_count_ = pgo_->getNumLC();
    debug_info_.pgo_lc_inliers_ = pgo_->getNumLCInliers();

    debug_info_.mono_input_size_ = tracker_->debug_info_.nrMonoPutatives_;
    debug_info_.mono_inliers_ = tracker_->debug_info_.nrMonoInliers_;
    debug_info_.mono_iter_ = tracker_->debug_info_.monoRansacIters_;

    debug_info_.stereo_input_size_ = tracker_->debug_info_.nrStereoPutatives_;
    debug_info_.stereo_inliers_ = tracker_->debug_info_.nrStereoInliers_;
    debug_info_.stereo_iter_ = tracker_->debug_info_.stereoRansacIters_;

    logger_->logTimestampMap(timestamp_map_);
    logger_->logDebugInfo(debug_info_);
    logger_->logLCDResult(*output_payload);
  }

  return output_payload;
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
void LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    initializePGO(const OdometryFactor& factor) {
  CHECK(lcd_state_ == LcdState::Bootstrap);
  CHECK_EQ(factor.cur_key_, 0u);

  gtsam::NonlinearFactorGraph init_nfg;
  gtsam::Values init_val;

  init_val.insert(gtsam::Symbol(factor.cur_key_), factor.W_Pose_Blkf_);

  init_nfg.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol(factor.cur_key_), factor.W_Pose_Blkf_, factor.noise_));

  CHECK(pgo_);
  pgo_->update(init_nfg, init_val);

  // Update tracker for latest VIO estimate
  // NOTE: done here instead of in spinOnce() to make unit tests easier.
  W_Pose_B_kf_vio_ = std::make_pair(factor.cur_key_, factor.W_Pose_Blkf_);

  lcd_state_ = LcdState::Nominal;
}

/* ------------------------------------------------------------------------
 */
// TODO(marcus): only add nodes if they're x dist away from previous node
// TODO(marcus): consider making the keys of OdometryFactor minus one each
// so that the extra check in here isn't needed...
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
void LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    addOdometryFactorAndOptimize(const OdometryFactor& factor) {
  CHECK(lcd_state_ == LcdState::Nominal);
  CHECK_GT(factor.cur_key_, 0u);

  const gtsam::Pose3& W_Pose_Bkf = factor.W_Pose_Blkf_;

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values value;

  const gtsam::Values& optimized_values = pgo_->calculateEstimate();
  CHECK_EQ(factor.cur_key_, optimized_values.size());
  const gtsam::Pose3& estimated_last_pose =
      optimized_values.at<gtsam::Pose3>(factor.cur_key_ - 1);

  // We can get the same relative pose used in the backend after
  // smoother_->update() by getting the relative pose between the latest two
  // VIO backend output poses, as these are created by chaining smoother_
  // relative poses.
  CHECK_EQ(W_Pose_B_kf_vio_.first, factor.cur_key_ - 1);
  const gtsam::Pose3& W_Pose_Blkf = W_Pose_B_kf_vio_.second;
  const gtsam::Pose3& B_lkf_Pose_kf = W_Pose_Blkf.between(W_Pose_Bkf);
  value.insert(gtsam::Symbol(factor.cur_key_),
               estimated_last_pose.compose(B_lkf_Pose_kf));

  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(factor.cur_key_ - 1),
                                             gtsam::Symbol(factor.cur_key_),
                                             B_lkf_Pose_kf,
                                             factor.noise_));

  CHECK(pgo_);
  pgo_->update(nfg, value);

  // Update tracker for latest VIO estimate
  // NOTE: done here instead of in spinOnce() to make unit tests easier.
  W_Pose_B_kf_vio_ = std::make_pair(factor.cur_key_, W_Pose_Bkf);
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
void LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    addLoopClosureFactorAndOptimize(const LoopClosureFactor& factor) {
  CHECK(lcd_state_ == LcdState::Nominal);

  gtsam::NonlinearFactorGraph nfg;

  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol(factor.ref_key_),
                                             gtsam::Symbol(factor.cur_key_),
                                             factor.ref_Pose_cur_,
                                             factor.noise_));

  // Only optimize if we don't have other potential loop closures to
  // process.
  CHECK(is_backend_queue_filled_cb_);
  // True if backend input queue is empty or we have cached enough LCs.
  bool do_optimize =
      num_lc_unoptimized_ >= lcd_params_.max_lc_cached_before_optimize_ ||
      !is_backend_queue_filled_cb_();

  VLOG(1) << "PGO: do optimize: " << do_optimize
          << ", num_lc_unoptimized: " << num_lc_unoptimized_
          << ", max_lc_cached_before_optimize_: "
          << lcd_params_.max_lc_cached_before_optimize_;

  if (!do_optimize) {
    num_lc_unoptimized_++;
  } else {
    num_lc_unoptimized_ = 0;
  }

  CHECK(pgo_);
  pgo_->update(nfg, gtsam::Values(), do_optimize && !FLAGS_lcd_no_optimize);
  VLOG(1) << "PGO: input pg size: " << nfg.size()
          << ", updated pg size: " << pgo_->getFactorsUnsafe().size()
          << ", num_lc: " << pgo_->getNumLC()
          << ", num_lc_inliers: " << pgo_->getNumLCInliers();
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
FrameId LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    processAndAddMonoFrame(const Frame& frame,
                           const PointsWithIdMap& W_points_with_ids,
                           const gtsam::Pose3& W_Pose_Blkf) {
  switch (lcd_params_.pose_recovery_type_) {
    case PoseRecoveryType::k5ptRotOnly: {
      // Since we are using the 5-pt method only, we don't need any 3d
      // keypoints for full pose-recovery. We are only doing up to a scaling
      // factor in translation.
      std::vector<cv::KeyPoint> keypoints;
      typename Database::Desc descriptors_mat;
      typename Database::DescVector descriptors_vec;
      getNewFeaturesAndDescriptors(frame, &keypoints, &descriptors_mat);
      descriptorMatToVec(frame, descriptors_mat, &descriptors_vec);

      BearingVectors versors;
      for (const cv::KeyPoint& keypoint : keypoints) {
        versors.push_back(UndistorterRectifier::GetBearingVector(
            keypoint.pt, frame.cam_param_));
      }

      return cache_.addFrame(std::make_shared<LCDFrame>(
          frame.timestamp_,
          FrameCache::NEW_ID,
          frame.id_,
          keypoints,
          Landmarks(),  // no 3d keypoints required for the 5-pt-only method
          descriptors_vec,
          descriptors_mat,
          versors));
    } break;

    case PoseRecoveryType::kPnP: {
      // Build and store LCDFrame object.
      return cache_.addFrame(
          this->processMonoPnP(frame, W_points_with_ids, W_Pose_Blkf));
    } break;

    case PoseRecoveryType::k3d3d: {
      LOG(FATAL) << "Cannot use PoseRecoveryType::k3d3d for Monocular LCD!";
    } break;

    default: {
      LOG(FATAL) << "Unrecognized pose recovery type: "
                 << static_cast<unsigned int>(lcd_params_.pose_recovery_type_)
                 << ".";
    } break;
  }

  throw std::runtime_error("Invalid pose recovery type");
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
FrameId LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    processAndAddStereoFrame(const StereoFrame& stereo_frame) {
  std::vector<cv::KeyPoint> keypoints;
  typename Database::Desc descriptors_mat;
  typename Database::DescVector descriptors_vec;
  getNewFeaturesAndDescriptors(
      stereo_frame.left_frame_.img_, &keypoints, &descriptors_mat);
  descriptorMatToVec(descriptors_mat, &descriptors_vec);

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  StereoFrame cp_stereo_frame(stereo_frame);
  rewriteStereoFrameFeatures(keypoints, &cp_stereo_frame);

  // Build and store LCDFrame object.
  return cache_.addFrame(std::make_shared<StereoLCDFrame>(
      cp_stereo_frame.timestamp_,
      FrameCache::NEW_ID,
      cp_stereo_frame.id_,
      keypoints,
      // keypoints_3d_ are in local (camera) frame
      cp_stereo_frame.keypoints_3d_,
      descriptors_vec,
      descriptors_mat,
      cp_stereo_frame.left_frame_.versors_,
      cp_stereo_frame.left_keypoints_rectified_,
      cp_stereo_frame.right_keypoints_rectified_));
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
FrameId LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    processAndAddRgbdFrame(const RgbdFrame& rgbd_frame) {
  std::vector<cv::KeyPoint> keypoints;
  typename Database::Desc descriptors_mat;
  typename Database::DescVector descriptors_vec;
  getNewFeaturesAndDescriptors(
      rgbd_frame.intensity_img_.img_, &keypoints, &descriptors_mat);
  descriptorMatToVec(descriptors_mat, &descriptors_vec);

  // Fill StereoFrame with ORB keypoints and perform stereo matching.
  auto cp_stereo_frame = rgbd_frame.getStereoFrame();
  for (const cv::KeyPoint& keypoint : keypoints) {
    cp_stereo_frame->left_frame_.keypoints_.push_back(keypoint.pt);
    cp_stereo_frame->left_frame_.versors_.push_back(
        UndistorterRectifier::GetBearingVector(
            keypoint.pt, cp_stereo_frame->left_frame_.cam_param_));
    cp_stereo_frame->left_frame_.scores_.push_back(1.0);
  }

  CHECK(rgbd_camera_) << "RGBD camera required for RGBD LCD";
  rgbd_frame.fillStereoFrame(*rgbd_camera_, *cp_stereo_frame);

  // Build and store LCDFrame object.
  return cache_.addFrame(std::make_shared<StereoLCDFrame>(
      cp_stereo_frame->timestamp_,
      FrameCache::NEW_ID,
      cp_stereo_frame->id_,
      keypoints,
      // keypoints_3d_ are in local (camera) frame
      cp_stereo_frame->keypoints_3d_,
      descriptors_vec,
      descriptors_mat,
      cp_stereo_frame->left_frame_.versors_,
      cp_stereo_frame->left_keypoints_rectified_,
      cp_stereo_frame->right_keypoints_rectified_));
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
const gtsam::Pose3
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::getWPoseMap()
    const {
  CHECK(pgo_);
  const gtsam::Symbol& cur_id = W_Pose_B_kf_vio_.first;
  const gtsam::Pose3& w_Pose_Bkf_estim = W_Pose_B_kf_vio_.second;
  const gtsam::Pose3& w_Pose_Bkf_optimal =
      pgo_->calculateEstimate().at<gtsam::Pose3>(cur_id);

  return w_Pose_Bkf_optimal.between(w_Pose_Bkf_estim);
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
const gtsam::Pose3
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::getMapPoseOdom()
    const {
  if (cache_.size() > 1) {
    CHECK(pgo_);
    const gtsam::Pose3& w_Pose_Bkf_estim = W_Pose_B_kf_vio_.second;
    const gtsam::Pose3& w_Pose_Bkf_optimal =
        pgo_->calculateEstimate().at<gtsam::Pose3>(W_Pose_B_kf_vio_.first);
    return w_Pose_Bkf_optimal.compose(w_Pose_Bkf_estim.inverse());
  }

  return gtsam::Pose3();
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
const gtsam::Values LoopClosureDetector<Database,
                                        FeatureDetector,
                                        FeatureMatcher>::getPGOTrajectory()
    const {
  CHECK(pgo_);
  return pgo_->calculateEstimate();
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
const gtsam::NonlinearFactorGraph
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::getPGOnfg()
    const {
  CHECK(pgo_);
  return pgo_->getFactorsUnsafe();
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
void LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    rewriteStereoFrameFeatures(const std::vector<cv::KeyPoint>& keypoints,
                               StereoFrame* stereo_frame) const {
  CHECK_NOTNULL(stereo_frame);

  // Populate frame keypoints with ORB features instead of the normal
  // VIO features that came with the StereoFrame.
  Frame* left_frame_mutable = &stereo_frame->left_frame_;
  Frame* right_frame_mutable = &stereo_frame->right_frame_;
  CHECK_NOTNULL(left_frame_mutable);
  CHECK_NOTNULL(right_frame_mutable);

  // Clear all relevant fields.
  left_frame_mutable->keypoints_.clear();
  left_frame_mutable->versors_.clear();
  left_frame_mutable->scores_.clear();
  right_frame_mutable->keypoints_.clear();
  right_frame_mutable->versors_.clear();
  right_frame_mutable->scores_.clear();
  stereo_frame->keypoints_depth_.clear();
  stereo_frame->keypoints_3d_.clear();
  stereo_frame->left_keypoints_rectified_.clear();
  stereo_frame->right_keypoints_rectified_.clear();

  // Reserve space in all relevant fields
  left_frame_mutable->keypoints_.reserve(keypoints.size());
  left_frame_mutable->versors_.reserve(keypoints.size());
  left_frame_mutable->scores_.reserve(keypoints.size());
  right_frame_mutable->keypoints_.reserve(keypoints.size());
  right_frame_mutable->versors_.reserve(keypoints.size());
  right_frame_mutable->scores_.reserve(keypoints.size());
  stereo_frame->keypoints_depth_.reserve(keypoints.size());
  stereo_frame->keypoints_3d_.reserve(keypoints.size());
  stereo_frame->left_keypoints_rectified_.reserve(keypoints.size());
  stereo_frame->right_keypoints_rectified_.reserve(keypoints.size());

  // stereo_frame->setIsRectified(false);

  // Add ORB keypoints.
  for (const cv::KeyPoint& keypoint : keypoints) {
    left_frame_mutable->keypoints_.push_back(keypoint.pt);
    left_frame_mutable->versors_.push_back(
        UndistorterRectifier::GetBearingVector(keypoint.pt,
                                               left_frame_mutable->cam_param_));
    left_frame_mutable->scores_.push_back(1.0);
  }

  if (left_frame_mutable->keypoints_.size() == 0) {
    return;
  }

  // Automatically match keypoints in right image with those in left.
  stereo_matcher_->sparseStereoReconstruction(stereo_frame);
  stereo_frame->checkStereoFrame();
}

/* ------------------------------------------------------------------------ */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
LCDStatus
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::recoverPoseBody(
    const LCDFrame& ref_frame,
    const LCDFrame& cur_frame,
    const gtsam::Pose3& camMatch_T_camQuery_2d,
    const KeypointMatches& matches_match_query,
    gtsam::Pose3* bodyMatch_T_bodyQuery_3d,
    std::vector<int>* inliers) {
  CHECK_NOTNULL(bodyMatch_T_bodyQuery_3d);
  CHECK_NOTNULL(inliers);

  gtsam::Pose3 camMatch_T_camQuery_3d;
  LCDStatus status;

  const StereoLCDFrame* ref_stereo_lcd_frame = nullptr;
  const StereoLCDFrame* cur_stereo_lcd_frame = nullptr;

  switch (lcd_params_.pose_recovery_type_) {
    case PoseRecoveryType::k3d3d: {
      TrackingStatusPose result;
      const bool camera_valid = stereo_camera_ || rgbd_camera_;
      if (tracker_->tracker_params_.ransac_use_1point_stereo_ && camera_valid) {
        // For 1pt we need stereo, so cast to derived form.
        ref_stereo_lcd_frame = dynamic_cast<const StereoLCDFrame*>(&ref_frame);
        cur_stereo_lcd_frame = dynamic_cast<const StereoLCDFrame*>(&cur_frame);
        if (!ref_stereo_lcd_frame || !cur_stereo_lcd_frame) {
          LOG(FATAL) << "LoopClosureDetector: Error casting to StereoLCDFrame. "
                        "Cannot have ransac_use_1point_stereo_ enabled without "
                        "stereo frontend inputs.";
        }

        std::pair<TrackingStatusPose, gtsam::Matrix3> result_full =
            tracker_->geometricOutlierRejection3d3dGivenRotation(
                ref_stereo_lcd_frame->left_keypoints_rectified_,
                ref_stereo_lcd_frame->right_keypoints_rectified_,
                cur_stereo_lcd_frame->left_keypoints_rectified_,
                cur_stereo_lcd_frame->right_keypoints_rectified_,
                ref_stereo_lcd_frame->keypoints_3d_,
                cur_stereo_lcd_frame->keypoints_3d_,
                stereo_camera_ ? stereo_camera_->getGtsamStereoCam()
                               : rgbd_camera_->getFakeStereoCamera(),
                matches_match_query,
                camMatch_T_camQuery_2d.rotation(),
                inliers);
        result = result_full.first;
        camMatch_T_camQuery_3d = result.second;
      } else {
        result =
            tracker_->geometricOutlierRejection3d3d(ref_frame.keypoints_3d_,
                                                    cur_frame.keypoints_3d_,
                                                    matches_match_query,
                                                    inliers);
        camMatch_T_camQuery_3d = result.second;
      }
      if (result.first == TrackingStatus::VALID) {
        status = LCDStatus::LOOP_DETECTED;
      }
    } break;

    case PoseRecoveryType::kPnP: {
      gtsam::Pose3 camMatch_T_camQuery_2d_copy(
          camMatch_T_camQuery_2d);  // because original is const

      BearingVectors camQuery_bearing_vectors;
      Landmarks camMatch_points;
      // TODO(marucs): consider adding this back in for ransac from inliers
      // only. for (const int& i : *inliers) {
      //   CHECK_LT(i, matches_match_query.size());
      //   const KeypointMatch& it = matches_match_query.at(i);
      for (const KeypointMatch& it : matches_match_query) {
        const BearingVector& query_bearing =
            cur_frame.bearing_vectors_.at(it.second);
        CHECK(it.first < ref_frame.landmark_ids.size())
            << "LoopClosureDetector: Invalid landmark id index " << it.first
            << " for ref_frame with size " << ref_frame.landmark_ids.size()
            << ".";
        auto ref_lmk_id = ref_frame.landmark_ids.at(it.first);
        auto lmk = landmark_manager_->getLandmark(ref_lmk_id);
        if (!lmk) {
          continue;
        }
        Landmark camMatch_lmk =
            (ref_frame.W_Pose_Blkf_ * B_Pose_Cam_).inverse() * (*lmk);
        camQuery_bearing_vectors.push_back(query_bearing);
        camMatch_points.push_back(camMatch_lmk);
      }

      bool success = false;
      if (camMatch_points.size() > lcd_params_.min_pnp_num_landmarks_) {
        success = tracker_->pnp(camQuery_bearing_vectors,
                                camMatch_points,
                                &camMatch_T_camQuery_3d,
                                inliers,
                                &camMatch_T_camQuery_2d_copy);
        if (success and camMatch_T_camQuery_3d.translation().norm() >
                            lcd_params_.max_pose_recovery_translation_) {
          success = false;
        }
      }

      // Manually fail the result if the norm of the translation vector is above
      // a fixed maximum. This is not technically required; PCM should be able
      // to handle these cases and reject them. However, on some datasets there
      // are several of these candidates that obviously are outliers so to keep
      // the optimization clean and prevent numerical instabilities, we filter
      // here before PCM.
      if (success) {
        status = LCDStatus::LOOP_DETECTED;
        break;
      } else {
        VLOG(1) << "pnp Pose recovery failed, fall back to 5ptRot";
        status = LCDStatus::FAILED_POSE_RECOVERY;
        // [[fallthrough]];
        break;
      }
    }

    case PoseRecoveryType::k5ptRotOnly: {
      // Passthrough the 2d2d pose to 3d3d, and the translation part will be
      // zeroed out in the noise model.
      camMatch_T_camQuery_3d = camMatch_T_camQuery_2d;
      status = LCDStatus::LOOP_DETECTED_ROT;
    } break;

    default: {
      LOG(FATAL) << "Unrecognized pose recovery type: "
                 << static_cast<unsigned int>(lcd_params_.pose_recovery_type_)
                 << ".";
    }
  }

  if (lcd_params_.refine_pose_) {
    if (!ref_stereo_lcd_frame || !cur_stereo_lcd_frame) {
      LOG(FATAL) << "LoopClosureDetector: Stereo required for refinePose";
    } else {
      camMatch_T_camQuery_3d = refinePoses(*ref_stereo_lcd_frame,
                                           *cur_stereo_lcd_frame,
                                           camMatch_T_camQuery_3d,
                                           matches_match_query);
    }
  }

  if (logger_) {
    logger_->logPoseRecovery(
        cur_frame.timestamp_, ref_frame.timestamp_, camMatch_T_camQuery_3d);
  }

  transformCameraPoseToBodyPose(camMatch_T_camQuery_3d,
                                bodyMatch_T_bodyQuery_3d);

  return status;
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
gtsam::Pose3
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::refinePoses(
    const StereoLCDFrame& ref_frame,
    const StereoLCDFrame& cur_frame,
    const gtsam::Pose3& camMatch_T_camQuery_3d,
    const KeypointMatches& matches_match_query) {
  gtsam::Cal3_S2Stereo::shared_ptr stereo_calib;
  if (stereo_camera_) {
    stereo_calib = stereo_camera_->getStereoCalib();
  } else if (rgbd_camera_) {
    stereo_calib = rgbd_camera_->getFakeStereoCalib();
  } else {
    LOG(FATAL) << "refinePose requires stereo or rgbd camera";
    return camMatch_T_camQuery_3d;
  }

  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values values;

  // TODO camMatch_T_camQuery rename to camMatch_T_camQuery
  gtsam::Key key_match = gtsam::Symbol('x', ref_frame.id_);
  gtsam::Key key_query = gtsam::Symbol('x', cur_frame.id_);
  values.insert(key_match, gtsam::Pose3());
  values.insert(key_query, camMatch_T_camQuery_3d);

  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Unit::Create(6);
  nfg.add(gtsam::PriorFactor<gtsam::Pose3>(key_match, gtsam::Pose3(), noise));

  gtsam::SharedNoiseModel noise_stereo = gtsam::noiseModel::Unit::Create(3);

  gtsam::SmartStereoProjectionParams smart_factors_params;
  smart_factors_params =
      SmartFactorParams(gtsam::HESSIAN,             // JACOBIAN_SVD
                        gtsam::ZERO_ON_DEGENERACY,  // IGNORE_DEGENERACY
                        false,                      // ThrowCherality = false
                        true);                      // verboseCherality = true
  smart_factors_params.setRankTolerance(1);
  smart_factors_params.setLandmarkDistanceThreshold(10);
  smart_factors_params.setRetriangulationThreshold(0.001);
  smart_factors_params.setDynamicOutlierRejectionThreshold(3);

  for (size_t i = 0; i < matches_match_query.size(); i++) {
    KeypointCV undistorted_rectified_left_match_keypoint =
        ref_frame.left_keypoints_rectified_.at(matches_match_query[i].first)
            .second;
    KeypointCV undistorted_rectified_right_match_keypoint =
        ref_frame.right_keypoints_rectified_.at(matches_match_query[i].first)
            .second;

    gtsam::StereoPoint2 sp_match_i(undistorted_rectified_left_match_keypoint.x,
                                   undistorted_rectified_right_match_keypoint.x,
                                   undistorted_rectified_left_match_keypoint.y);

    SmartStereoFactor stereo_factor_i(noise_stereo, smart_factors_params);

    stereo_factor_i.add(sp_match_i, key_match, stereo_calib);

    KeypointCV undistorted_rectified_left_query_keypoint =
        cur_frame.left_keypoints_rectified_.at(matches_match_query[i].second)
            .second;
    KeypointCV undistorted_rectified_right_query_keypoint =
        cur_frame.right_keypoints_rectified_.at(matches_match_query[i].second)
            .second;

    gtsam::StereoPoint2 sp_query_i(undistorted_rectified_left_query_keypoint.x,
                                   undistorted_rectified_right_query_keypoint.x,
                                   undistorted_rectified_left_query_keypoint.y);

    stereo_factor_i.add(sp_query_i, key_query, stereo_calib);

    nfg.add(stereo_factor_i);
  }

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("ERROR");
  try {
    values = gtsam::LevenbergMarquardtOptimizer(nfg, values, params).optimize();
  } catch (const gtsam::CheiralityException& e) {
    LOG(ERROR) << e.what();
  } catch (const gtsam::StereoCheiralityException& e) {
    LOG(ERROR) << e.what();
  } catch (const gtsam::OutOfRangeThreadsafe& e) {
    LOG(ERROR) << e.what();
  } catch (const std::out_of_range& e) {
    LOG(ERROR) << e.what();
  } catch (const std::exception& e) {
    // Catch anything thrown within try block that derives from
    // std::exception.
    LOG(ERROR) << e.what();
  } catch (...) {
    // Catch the rest of exceptions.
    LOG(ERROR) << "Unrecognized exception.";
  }

  return values.at<gtsam::Pose3>(key_query);
}

}  // namespace VIO