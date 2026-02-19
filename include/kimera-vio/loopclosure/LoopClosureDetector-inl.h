#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <memory>

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
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
      stereo_matching_params_(stereo_matching_params
                                  ? stereo_matching_params.value()
                                  : StereoMatchingParams()),
      stereo_matcher_(nullptr),
      rgbd_camera_(rgbd_camera ? rgbd_camera.value() : nullptr),
      db_(nullptr),
      cache_(lcd_params.frame_cache),
      B_Pose_Cam_(B_Pose_Cam),
      latest_global_vec_(nullptr),
      tracker_(nullptr),
      lcd_tp_wrapper_(nullptr),
      logger_(nullptr),
      log_output_(log_output) {
  // Shared noise model initialization
  gtsam::Vector6 precisions;
  precisions.head<3>().setConstant(lcd_params_.betweenRotationPrecision_);
  precisions.tail<3>().setConstant(lcd_params_.betweenTranslationPrecision_);
  shared_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  // Outlier rejection initialization (inside of tracker)
  static constexpr bool kLCDTrackerUseOF = false;
  tracker_ = std::make_unique<Tracker>(
      lcd_params.tracker_params_,
      std::make_shared<VIO::Camera>(tracker_cam_params),
      nullptr,
      nullptr,
      kLCDTrackerUseOF);

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

  LmkMapWithStats backend_stats;
  backend_stats.num_observations = input.lmk_num_observations_;
  backend_stats.residuals = input.lmk_smart_factor_residuals_;

  landmark_manager_->updateLandmarks(
      input.landmark_in_window_, input.W_Pose_smoother_, &backend_stats);
  landmark_manager_->updateLandmarks(
      input.landmark_out_window_, input.W_Pose_smoother_, &backend_stats);
  std::set<LandmarkId> new_lmk_ids;
  for (const auto& [lmk_id, lmk] : input.landmark_out_window_) {
    new_lmk_ids.insert(lmk_id);
  }
  // for (const auto& [lmk_id, lmk] : input.landmark_in_window_) {
  //   new_lmk_ids.insert(lmk_id);
  // }
  int culled = landmark_manager_->checkAndCullingLandmarks(
      new_lmk_ids,
      cache_,
      lcd_params_.min_lmk_obs_ratio_,
      lcd_params_.min_lmk_parallax_,
      lcd_params_.max_lmk_reproj_error,
      lcd_params_.min_lmk_obs_cnt_);
  VLOG(1) << "Culled landmarks: " << culled;

  timestamp_map_[input.cur_kf_id_] = input.timestamp_;

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
                                              input.landmark_out_window_,
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
      lcd_frame_id = processAndAddStereoFrame(
          stereo_frontend_output->stereo_frame_lkf_, input.W_Pose_Blkf_);
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

  bool frame_is_valid{false};
  CHECK(input.frontend_output_);
  if (auto stereo_output = std::dynamic_pointer_cast<StereoFrontendOutput>(
          input.frontend_output_)) {
    if (not stereo_output->getTrackerStatus()) {
      frame_is_valid = false;
    } else {
      frame_is_valid =
          stereo_output->getTrackerStatus()->kfTrackingStatus_stereo_ ==
              TrackingStatus::VALID or
          stereo_output->getTrackerStatus()->kfTrackingStatus_mono_ ==
              TrackingStatus::VALID;
    }
  } else if (auto mono_output = std::dynamic_pointer_cast<MonoFrontendOutput>(
                 input.frontend_output_)) {
    if (not mono_output->getTrackerStatus()) {
      frame_is_valid = false;
    } else {
      frame_is_valid =
          mono_output->getTrackerStatus()->kfTrackingStatus_mono_ ==
          TrackingStatus::VALID;
    }
  } else {
    LOG(FATAL) << "Unknown frontend output type.";
  }

  bool frame_too_repetitive_wrt_lkf =
      landmark_manager_->computeCovisibilityScore(
          lcd_frame_id - 1, lcd_frame_id) > lcd_params_.max_covisibility_score_;
  bool frame_too_repetitive_wrt_seq_anchor =
      getCurrentAnchorFrameId()
          ? landmark_manager_->computeCovisibilityScore(
                *getCurrentAnchorFrameId(), lcd_frame_id) >
                lcd_params_.max_covisibility_score_
          : false;

  bool add_frame_to_sequence =
      frame_is_valid and (not frame_too_repetitive_wrt_lkf and
                          not frame_too_repetitive_wrt_seq_anchor);

  computeSequenceGlobalDesc(lcd_frame_id, add_frame_to_sequence);

  updatePoseGraph(input.backend_states_);

  if (lcd_frame_id < static_cast<FrameId>(lcd_params_.local_window_size_)) {
    FrameId clean_frames_until_id =
        landmark_manager_->getOldestCovisFrame(lcd_frame_id);
    cleanFrameUntil(clean_frames_until_id);
    LOG(INFO) << "VLADLCD: LG: Not enough frames for loop detection. Current "
                 "frame ID: "
              << lcd_frame_id << ", waiting until we have at least "
              << lcd_params_.local_window_size_ << " frames.";
    return nullptr;
  } else {
    FrameId output_frame_id =
        lcd_frame_id - static_cast<FrameId>(lcd_params_.local_window_size_);
    LcdOutput::UniquePtr output_payload =
        makeOutputPayload(input.timestamp_, output_frame_id);

    // only clean frame if it contains global descriptors, i.e. once per
    // sequence
    if (!output_payload->bow_vec_.empty()) {
      FrameId clean_frames_until_id =
          landmark_manager_->getOldestCovisFrame(output_frame_id);
      cleanFrameUntil(clean_frames_until_id);
    }
    if (!output_payload) {
      LOG(WARNING) << "makeOutputPayload returned nullptr.";
    }
    return output_payload;
  }
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
void LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    updatePoseGraph(const gtsam::Values& values) {
  // extract odometry measurements from values
  for (auto const& [key, value] : values) {
    gtsam::Key pose_key(key);
    if (gtsam::symbolChr(pose_key) != 'x') {
      continue;
    }
    gtsam::Symbol next_key('x', gtsam::symbolIndex(pose_key) + 1);
    if (values.exists(pose_key) && values.exists(next_key)) {
      gtsam::Pose3 pose = values.at<gtsam::Pose3>(pose_key),
                   next_pose = values.at<gtsam::Pose3>(next_key),
                   T_pose_next = pose.inverse() * next_pose;
      gtsam::Key pose_id = gtsam::symbolIndex(pose_key), next_id = pose_id + 1;
      // build between factor
      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr factor(
          new gtsam::BetweenFactor<gtsam::Pose3>(
              pose_id, next_id, T_pose_next, shared_noise_model_));
      pg_[std::make_pair(pose_id, next_id)] = factor;
    }
  }

  for (auto const& [key, value] : values) {
    if (gtsam::symbolChr(key) != 'x') continue;
    pg_values_.insert_or_assign(gtsam::symbolIndex(key), value);
  }
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
std::optional<LcdGridFrame>
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    augmentAndFilterFrameFeatures(FrameId lcd_frame_id) {
  auto curr_frame = cache_.getFrame(lcd_frame_id);
  if (!curr_frame) {
    LOG(ERROR) << "augmentAndFilterFrameFeatures: frame " << lcd_frame_id
               << " not found in cache.";
    return std::nullopt;
  }

  if (!landmark_manager_) {
    LOG(ERROR) << "Landmark manager is null for frame " << lcd_frame_id;
    return std::nullopt;
  }

  // Copy frame data to allow modifications
  std::vector<cv::KeyPoint> frame_keypoints = curr_frame->keypoints_;
  std::vector<LandmarkId> frame_landmark_ids = curr_frame->landmark_ids;
  typename Database::Desc frame_descriptors_mat =
      curr_frame->descriptors_mat_.clone();
  BearingVectors frame_bearing_vectors = curr_frame->bearing_vectors_;

  // Augment with covisibility neighbours' landmarks visible in current FOV
  auto covis_it = landmark_manager_->getCovisGraph().find(lcd_frame_id);
  if (covis_it != landmark_manager_->getCovisGraph().end()) {
    for (const auto& covis_frame_id : covis_it->second) {
      auto covis_frame = cache_.getFrame(covis_frame_id);
      if (!covis_frame) continue;

      for (size_t lmk_i = 0; lmk_i < covis_frame->landmark_ids.size();
           lmk_i++) {
        LandmarkId lmk_id = covis_frame->landmark_ids[lmk_i];
        if (std::find(frame_landmark_ids.begin(),
                      frame_landmark_ids.end(),
                      lmk_id) != frame_landmark_ids.end()) {
          continue;
        }

        CHECK_GE(static_cast<size_t>(covis_frame->descriptors_mat_.rows), lmk_i)
            << "Descriptor index out of bounds";

        auto T_w_lmk = landmark_manager_->getLandmark(lmk_id);
        if (!T_w_lmk) continue;

        Landmark cam_lmk =
            (curr_frame->W_Pose_Blkf_ * B_Pose_Cam_).inverse() * (*T_w_lmk);
        if (cam_lmk.z() <= 0.1) continue;

        cv::Point2f uv_rect;
        CHECK_EQ(curr_frame->cam_params_.K_.rows, 3);
        CHECK_EQ(curr_frame->cam_params_.K_.cols, 3);
        if (!landmark_manager_->isLandmarkInFov(
                cam_lmk, curr_frame->cam_params_, &uv_rect)) {
          continue;
        }

        frame_landmark_ids.push_back(lmk_id);

        cv::KeyPoint new_kp;
        new_kp.pt = uv_rect;
        new_kp.response = 1.0f;
        new_kp.size = 1.0f;
        frame_keypoints.push_back(new_kp);

        frame_descriptors_mat.push_back(
            covis_frame->descriptors_mat_.row(lmk_i).clone());
        frame_bearing_vectors.push_back(UndistorterRectifier::GetBearingVector(
            uv_rect, curr_frame->cam_params_));
      }
    }
  }

  // Resolve 3-D positions (camera frame) for all landmark IDs
  Landmarks frame_landmarks;
  frame_landmarks.reserve(frame_landmark_ids.size());
  const gtsam::Pose3 T_cam_world =
      (curr_frame->W_Pose_Blkf_ * B_Pose_Cam_).inverse();
  for (const auto& lmk_id : frame_landmark_ids) {
    auto T_w_lmk = landmark_manager_->getLandmark(lmk_id);
    frame_landmarks.push_back(T_w_lmk ? T_cam_world * (*T_w_lmk) : Landmark());
  }

  // Grid assignment: keep the highest-response keypoint per cell
  const int grid_cols = 40;
  const int grid_rows = 40;
  const int img_width = curr_frame->cam_params_.image_size_.width;
  const int img_height = curr_frame->cam_params_.image_size_.height;
  const float cell_w = static_cast<float>(img_width) / grid_cols;
  const float cell_h = static_cast<float>(img_height) / grid_rows;

  // grid_best[row][col] = (response, index into frame arrays)
  std::vector<std::vector<std::pair<float, int>>> grid_best(
      grid_rows, std::vector<std::pair<float, int>>(grid_cols, {-1.f, -1}));

  for (size_t i = 0; i < frame_keypoints.size(); ++i) {
    const auto& lmk = frame_landmarks[i];
    if (lmk.x() == 0.0 && lmk.y() == 0.0 && lmk.z() == 0.0) continue;

    const auto& kpt = frame_keypoints[i];
    int gx = std::max(
        0, std::min(grid_cols - 1, static_cast<int>(kpt.pt.x / cell_w)));
    int gy = std::max(
        0, std::min(grid_rows - 1, static_cast<int>(kpt.pt.y / cell_h)));

    if (kpt.response > grid_best[gy][gx].first) {
      grid_best[gy][gx] = {kpt.response, static_cast<int>(i)};
    }
  }

  // Build LcdGridFrame from the grid assignments
  LcdGridFrame grid_frame(grid_cols, grid_rows, img_width, img_height);
  for (int row = 0; row < grid_rows; ++row) {
    for (int col = 0; col < grid_cols; ++col) {
      int idx = grid_best[row][col].second;
      if (idx < 0) continue;

      LandmarkId lmk_id = frame_landmark_ids[idx];
      if (lmk_id < 0) continue;
      LcdGridCell cell_data;
      cell_data.keypoint = frame_keypoints[idx];
      cell_data.landmark = frame_landmarks[idx];
      cell_data.descriptor = frame_descriptors_mat.empty()
                                 ? cv::Mat()
                                 : frame_descriptors_mat.row(idx).clone();
      cell_data.bearing_vector = frame_bearing_vectors[idx];
      cell_data.landmark_id = lmk_id;
      cell_data.num_obs = landmark_manager_->getNumObs(lmk_id);
      cell_data.residual = landmark_manager_->getResidual(lmk_id);
      grid_frame.cell(row, col) = std::move(cell_data);
    }
  }

  VLOG(1) << "augmentAndFilterFrameFeatures: " << grid_frame.size()
          << " keypoints kept in " << grid_cols << "x" << grid_rows << " grid.";
  return grid_frame;
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
LcdOutput::UniquePtr
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    makeOutputPayload(Timestamp msg_timestamp, FrameId lcd_frame_id) {
  const auto curr_frame = cache_.getFrame(lcd_frame_id);
  if (!curr_frame) {
    LOG(ERROR) << "Invalid frame ID requested: " << lcd_frame_id;
    return nullptr;
  }

  bool is_seq_frame = curr_frame->descriptors_vec_.size() > 0;

  const auto grid_frame = augmentAndFilterFrameFeatures(lcd_frame_id);
  if (!grid_frame) {
    return nullptr;
  }

  double S_cover = grid_frame->computeCoverageScore();
  double S_struct = grid_frame->computeStructureScore();

  auto filtered_keypoints = grid_frame->getKeypoints();
  auto filtered_landmarks = grid_frame->getLandmarks();
  auto filtered_descriptors_mat = grid_frame->getDescriptors();
  auto filtered_bearing_vectors = grid_frame->getBearingVectors();

  LcdOutput::UniquePtr output_payload =
      std::make_unique<LcdOutput>(LCDStatus::NO_MATCHES, msg_timestamp);

  CHECK(output_payload) << "Missing LCD output payload.";

  gtsam::NonlinearFactorGraph pg;
  for (auto it = pg_.begin(); it != pg_.end(); it++) {
    pg.add(it->second);
  }

  pg.addPrior(gtsam::Key(0),
              gtsam::Pose3(),
              gtsam::noiseModel::Isotropic::Sigma(6, 1e-4));

  output_payload->setMapInformation(
      gtsam::Pose3(), gtsam::Pose3(), pg_values_, pg);

  KeypointsCV keypoints_2d;
  cv::KeyPoint::convert(filtered_keypoints, keypoints_2d);

  CHECK_EQ(filtered_keypoints.size(), filtered_landmarks.size());
  CHECK_EQ(filtered_landmarks.size(), filtered_bearing_vectors.size());

  std::map<int, double> bow_vec{};
  if (curr_frame->descriptors_vec_.size() and
      S_cover > lcd_params_.min_seq_coverage_score_ and
      S_struct > lcd_params_.min_seq_structure_score_) {
    bow_vec = globalDescToMap(curr_frame->descriptors_vec_[0]);
  } else {
    bow_vec = {};
  }

  cv::Mat debug_seq_frame;
  // if frame's image is empty, make a black image for visualization
  if (curr_frame->image_.empty()) {
    debug_seq_frame = cv::Mat(
        curr_frame->cam_params_.image_size_, CV_8UC3, cv::Scalar(0, 0, 0));
  } else {
    debug_seq_frame = curr_frame->image_.clone();
  }

  // draw keypoints on the image for visualization, and colored by depth
  for (size_t i = 0; i < filtered_keypoints.size(); ++i) {
    const auto& kp = filtered_keypoints[i];
    const auto& lmk = filtered_landmarks[i];
    if (lmk.z() > 0) {
      // Color by depth (closer points are red, farther points are blue)
      float depth = lmk.z();
      float depth_normalized =
          std::min(depth / 10.0f, 1.0f);  // Assuming max depth of 10m
      cv::Scalar color =
          cv::Scalar(255 * (1 - depth_normalized), 0, 255 * depth_normalized);
      cv::circle(debug_seq_frame, kp.pt, 3, color, -1);
    } else {
      // If no valid landmark, draw in white
      cv::circle(debug_seq_frame, kp.pt, 3, cv::Scalar(255, 255, 255), -1);
    }
  }

  // downsample to 320 width for visualization if larger
  if (debug_seq_frame.cols > 320) {
    int new_height =
        static_cast<int>(debug_seq_frame.rows * (320.0 / debug_seq_frame.cols));
    cv::resize(debug_seq_frame,
               debug_seq_frame,
               cv::Size(320, new_height),
               0,
               0,
               cv::INTER_AREA);
  }

  // draw the scores on the image for visualization
  std::string score_text =
      "Sc: " + std::to_string(S_cover) + ", Ss: " + std::to_string(S_struct);
  cv::putText(debug_seq_frame,
              score_text,
              cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX,
              0.8,
              cv::Scalar(0, 255, 0),
              2);

  output_payload->setFrameInformation(keypoints_2d,
                                      filtered_landmarks,
                                      filtered_bearing_vectors,
                                      bow_vec,
                                      filtered_descriptors_mat);
  output_payload->landmarks_ = landmark_manager_->getLandmarks();
  output_payload->timestamp_map_ = timestamp_map_;
  output_payload->covis_graph_ = landmark_manager_->getCovisGraph();
  output_payload->timestamp_kf_ = curr_frame->timestamp_;
  output_payload->T_base_cam_ = B_Pose_Cam_;

  // Frame cache statistics
  output_payload->frame_cache_memory_bytes_ = cache_.getMemoryUsage();
  output_payload->frame_cache_size_ = cache_.size();

  output_payload->seq_frames = seq_frames_;
  output_payload->debug_seq_frame =
      std::make_pair(lcd_frame_id, debug_seq_frame);
  output_payload->is_seq_frame = is_seq_frame;

  if (auto current_anchor_frame_id = getCurrentAnchorFrameId()) {
    auto anchor_grid = augmentAndFilterFrameFeatures(*current_anchor_frame_id);
    output_payload->coverage_score =
        anchor_grid ? anchor_grid->computeCoverageScore() : 0.0;
    output_payload->structure_score =
        anchor_grid ? anchor_grid->computeStructureScore() : 0.0;
  }

  output_payload->covisibility_score =
      landmark_manager_->computeCovisibilityScore(lcd_frame_id - 1,
                                                  lcd_frame_id);

  return output_payload;
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
void LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    filterKeypointsWithGrid(int img_width,
                            int img_height,
                            int grid_cols,
                            int grid_rows,
                            std::vector<cv::KeyPoint>* keypoints,
                            Landmarks* landmarks,
                            typename Database::Desc* descriptors_mat,
                            BearingVectors* bearing_vectors,
                            std::vector<StatusKeypointCV>* left_kpts_rect,
                            std::vector<StatusKeypointCV>* right_kpts_rect,
                            std::vector<LandmarkId>* landmark_ids) const {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(landmarks);
  CHECK_NOTNULL(descriptors_mat);
  CHECK_NOTNULL(bearing_vectors);

  if (keypoints->empty()) return;

  CHECK_EQ(keypoints->size(), landmarks->size());
  CHECK_EQ(keypoints->size(), bearing_vectors->size());
  if (left_kpts_rect) CHECK_EQ(keypoints->size(), left_kpts_rect->size());
  if (right_kpts_rect) CHECK_EQ(keypoints->size(), right_kpts_rect->size());
  if (landmark_ids) CHECK_EQ(keypoints->size(), landmark_ids->size());

  const float cell_width = static_cast<float>(img_width) / grid_cols;
  const float cell_height = static_cast<float>(img_height) / grid_rows;

  // Grid to store the best keypoint index for each cell
  std::vector<std::vector<int>> grid(grid_rows,
                                     std::vector<int>(grid_cols, -1));
  std::vector<std::vector<float>> grid_scores(
      grid_rows, std::vector<float>(grid_cols, -1.0f));

  // Find the best keypoint for each grid cell
  for (size_t i = 0; i < keypoints->size(); ++i) {
    const auto& kpt = (*keypoints)[i];
    const auto& lmk = (*landmarks)[i];

    // Skip keypoints without valid landmarks
    if (lmk.x() == 0.0 && lmk.y() == 0.0 && lmk.z() == 0.0) {
      continue;
    }

    int grid_x = static_cast<int>(kpt.pt.x / cell_width);
    int grid_y = static_cast<int>(kpt.pt.y / cell_height);

    // Clamp to grid bounds
    grid_x = std::max(0, std::min(grid_cols - 1, grid_x));
    grid_y = std::max(0, std::min(grid_rows - 1, grid_y));

    // Keep the keypoint with the highest response in each cell
    if (grid[grid_y][grid_x] == -1 ||
        kpt.response > grid_scores[grid_y][grid_x]) {
      grid[grid_y][grid_x] = i;
      grid_scores[grid_y][grid_x] = kpt.response;
    }
  }

  // Collect indices of keypoints to keep
  std::vector<int> indices_to_keep;
  for (int row = 0; row < grid_rows; ++row) {
    for (int col = 0; col < grid_cols; ++col) {
      if (grid[row][col] != -1) {
        indices_to_keep.push_back(grid[row][col]);
      }
    }
  }

  // Sort indices to maintain order
  std::sort(indices_to_keep.begin(), indices_to_keep.end());

  // Filter all data structures
  std::vector<cv::KeyPoint> filtered_keypoints;
  Landmarks filtered_landmarks;
  BearingVectors filtered_bearing_vectors;
  typename Database::Desc filtered_descriptors_mat;
  typename Database::DescVector filtered_descriptors_vec;
  std::vector<StatusKeypointCV> filtered_left_kpts_rect;
  std::vector<StatusKeypointCV> filtered_right_kpts_rect;
  std::vector<LandmarkId> filtered_landmark_ids;

  filtered_keypoints.reserve(indices_to_keep.size());
  filtered_landmarks.reserve(indices_to_keep.size());
  filtered_bearing_vectors.reserve(indices_to_keep.size());
  if (left_kpts_rect) filtered_left_kpts_rect.reserve(indices_to_keep.size());
  if (right_kpts_rect) filtered_right_kpts_rect.reserve(indices_to_keep.size());
  if (landmark_ids) filtered_landmark_ids.reserve(indices_to_keep.size());

  for (int idx : indices_to_keep) {
    filtered_keypoints.push_back((*keypoints)[idx]);
    filtered_landmarks.push_back((*landmarks)[idx]);
    filtered_bearing_vectors.push_back((*bearing_vectors)[idx]);

    if (!descriptors_mat->empty()) {
      filtered_descriptors_mat.push_back(descriptors_mat->row(idx));
    }

    if (left_kpts_rect) {
      filtered_left_kpts_rect.push_back((*left_kpts_rect)[idx]);
    }

    if (right_kpts_rect) {
      filtered_right_kpts_rect.push_back((*right_kpts_rect)[idx]);
    }

    if (landmark_ids) {
      filtered_landmark_ids.push_back((*landmark_ids)[idx]);
    }
  }

  // Replace original data with filtered data
  *keypoints = std::move(filtered_keypoints);
  *landmarks = std::move(filtered_landmarks);
  *bearing_vectors = std::move(filtered_bearing_vectors);
  *descriptors_mat = std::move(filtered_descriptors_mat);

  if (left_kpts_rect) {
    *left_kpts_rect = std::move(filtered_left_kpts_rect);
  }

  if (right_kpts_rect) {
    *right_kpts_rect = std::move(filtered_right_kpts_rect);
  }

  if (landmark_ids) {
    *landmark_ids = std::move(filtered_landmark_ids);
  }

  VLOG(1) << "Grid filtering: " << indices_to_keep.size()
          << " keypoints kept from original " << keypoints->size();
}

template <typename Database, typename FeatureDetector, typename FeatureMatcher>
FrameId LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    processAndAddMonoFrame(const Frame& frame,
                           const PointsWithIdMap& W_points_with_ids,
                           const gtsam::Pose3& W_Pose_Blkf) {
  std::vector<cv::KeyPoint> keypoints;
  typename Database::Desc descriptors_mat;
  typename Database::DescVector descriptors_vec;
  getNewFeaturesAndDescriptors(frame, &keypoints, &descriptors_mat);
  descriptorMatToVec(frame, descriptors_mat, &descriptors_vec);

  auto lcd_frame = std::make_shared<LCDFrame>(frame.timestamp_,
                                              FrameCache::NEW_ID,
                                              frame.id_,
                                              keypoints,
                                              Landmarks(),
                                              descriptors_vec,
                                              descriptors_mat,
                                              frame.versors_);
  lcd_frame->landmark_ids = frame.landmarks_;
  lcd_frame->W_Pose_Blkf_ = W_Pose_Blkf;
  lcd_frame->image_ = frame.img_;
  lcd_frame->cam_params_ = frame.cam_param_;
  return cache_.addFrame(lcd_frame);
}

/* ------------------------------------------------------------------------
 */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
FrameId LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::
    processAndAddStereoFrame(const StereoFrame& stereo_frame,
                             const Pose3& W_Pose_Blkf) {
  std::vector<cv::KeyPoint> keypoints;
  typename Database::Desc descriptors_mat;
  typename Database::DescVector descriptors_vec;
  getNewFeaturesAndDescriptors(
      stereo_frame.left_frame_, &keypoints, &descriptors_mat);
  descriptorMatToVec(
      stereo_frame.left_frame_, descriptors_mat, &descriptors_vec);

  // Build and store LCDFrame object.
  auto lcd_frame = std::make_shared<StereoLCDFrame>(
      stereo_frame.timestamp_,
      FrameCache::NEW_ID,
      stereo_frame.id_,
      keypoints,
      Landmarks(),  // Empty, will be populated in makeOutputPayload
      descriptors_vec,
      descriptors_mat,
      stereo_frame.left_frame_.versors_,
      stereo_frame.left_keypoints_rectified_,
      stereo_frame.right_keypoints_rectified_);
  lcd_frame->landmark_ids = stereo_frame.left_frame_.landmarks_;
  lcd_frame->W_Pose_Blkf_ = W_Pose_Blkf;
  lcd_frame->image_ = stereo_frame.left_frame_.img_;
  lcd_frame->cam_params_ = stereo_frame.left_frame_.cam_param_;
  return cache_.addFrame(lcd_frame);
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

/* ------------------------------------------------------------------------ */
template <typename Database, typename FeatureDetector, typename FeatureMatcher>
LCDStatus
LoopClosureDetector<Database, FeatureDetector, FeatureMatcher>::recoverPoseBody(
    const LCDFrame& ref_frame,
    const LCDFrame& cur_frame,
    const gtsam::Pose3& camMatch_T_camQuery_2d,
    const KeypointMatches& matches_match_query,
    gtsam::Pose3* bodyMatch_T_bodyQuery_3d,
    gtsam::Pose3* bodyQuery_T_bodyMatch_3d,
    std::vector<int>* inliers) {
  CHECK_NOTNULL(bodyMatch_T_bodyQuery_3d);
  CHECK_NOTNULL(inliers);

  gtsam::Pose3 camMatch_T_camQuery_3d;
  gtsam::Pose3 camQuery_T_camMatch_3d;
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
        if (!lmk) continue;
        Landmark camMatch_lmk =
            (ref_frame.W_Pose_Blkf_ * B_Pose_Cam_).inverse() * (*lmk);
        camQuery_bearing_vectors.push_back(query_bearing);
        camMatch_points.push_back(camMatch_lmk);
      }

      bool success = false;
      if (camMatch_points.size() > size_t(lcd_params_.min_pnp_num_landmarks_)) {
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

      // if (!success) {
      //   BearingVectors camMatch_bearing_vectors;
      //   Landmarks camQuery_points;
      //   for (const KeypointMatch& it : matches_match_query) {
      //     const BearingVector& match_bearing =
      //         ref_frame.bearing_vectors_.at(it.first);
      //     LandmarkId query_lmk_id = cur_frame.landmark_ids.at(it.second);
      //     auto lmk = landmark_manager_->getLandmark(query_lmk_id);
      //     if (!lmk) continue;
      //     Landmark camQuery_lmk =
      //         (cur_frame.W_Pose_Blkf_ * B_Pose_Cam_).inverse() * (*lmk);
      //     camMatch_bearing_vectors.push_back(match_bearing);
      //     camQuery_points.push_back(camQuery_lmk);
      //   }

      //   if (camQuery_points.size() > lcd_params_.min_pnp_num_landmarks_) {
      //     Pose3 camQuery_T_camMatch_2d_copy =
      //     camMatch_T_camQuery_2d.inverse(); success =
      //     tracker_->pnp(camMatch_bearing_vectors,
      //                             camQuery_points,
      //                             &camQuery_T_camMatch_3d,
      //                             inliers,
      //                             &camQuery_T_camMatch_2d_copy);
      //     camMatch_T_camQuery_3d = camQuery_T_camMatch_3d.inverse();
      //   } else {
      //     success = false;
      //   }
      //   if (success and camQuery_T_camMatch_3d.translation().norm() >
      //                       lcd_params_.max_pose_recovery_translation_) {
      //     success = false;
      //   }
      // }

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
        status = LCDStatus::FAILED_POSE_RECOVERY;
        // VLOG(1) << "pnp Pose recovery failed, fall back to 5ptRot";
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

  transformCameraPoseToBodyPose(camQuery_T_camMatch_3d,
                                bodyQuery_T_bodyMatch_3d);

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