/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.cpp
 * @brief  Pipeline for detection and reporting of Loop Closures between frames.
 * @author Marcus Abate
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include "kimera-vio/loopclosure/DBoWLoopClosureDetector.h"

#include <DBoW2/DBoW2.h>
#include <KimeraRPGO/RobustSolver.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <algorithm>
#include <string>
#include <vector>

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend-definitions.h"
#include "kimera-vio/loopclosure/LcdThirdPartyWrapper.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

/** Verbosity settings: (cumulative with every increase in level)
      0: Runtime errors and warnings, spin start and frequency are reported.
      1: Loop closure detections are reported as warnings.
      2: Loop closure failures are reported as errors.
      3: Statistics are reported at relevant steps.
**/

DEFINE_string(vocabulary_path,
              "../vocabulary/ORBvoc.yml",
              "Path to BoW vocabulary file for LoopClosureDetector module.");

DEFINE_bool(lcd_disable_stereo_match_depth_check,
            false,
            "disable thresholding of stereo landmark correspondences");

namespace VIO {

std::unique_ptr<OrbVocabulary> loadOrbVocabulary() {
  std::ifstream f_vocab(FLAGS_vocabulary_path.c_str());
  CHECK(f_vocab.good()) << "LoopClosureDetector: Incorrect vocabulary path: "
                        << FLAGS_vocabulary_path;
  f_vocab.close();

  auto vocab = std::make_unique<OrbVocabulary>();
  LOG(INFO) << "LoopClosureDetector:: Loading vocabulary from "
            << FLAGS_vocabulary_path;
  vocab->load(FLAGS_vocabulary_path);
  LOG(INFO) << "Loaded vocabulary with " << vocab->size() << " visual words.";
  return vocab;
}

PreloadedVocab::PreloadedVocab() { vocab = loadOrbVocabulary(); }

PreloadedVocab::PreloadedVocab(PreloadedVocab&& other) {
  vocab = std::move(other.vocab);
}

PreloadedVocab::~PreloadedVocab() {}

/* ------------------------------------------------------------------------ */
DBoWLoopClosureDetector::DBoWLoopClosureDetector(
    const LoopClosureDetectorParams& lcd_params,
    const CameraParams& tracker_cam_params,
    const gtsam::Pose3& B_Pose_Cam,
    const std::optional<VIO::StereoCamera::ConstPtr>& stereo_camera,
    const std::optional<StereoMatchingParams>& stereo_matching_params,
    const std::optional<VIO::RgbdCamera::ConstPtr>& rgbd_camera,
    bool log_output,
    PreloadedVocab::Ptr&& preloaded_vocab)
    : LoopClosureDetector(lcd_params,
                          tracker_cam_params,
                          B_Pose_Cam,
                          stereo_camera,
                          stereo_matching_params,
                          rgbd_camera,
                          log_output) {
  // Sparse stereo reconstruction members (only if stereo_camera is provided)
  if (stereo_camera) {
    VLOG(5) << "LoopClosureDetector initializing in stereo mode.";
    CHECK(stereo_camera_);
    auto lcd_stereo_params = stereo_matching_params.value();
    // In LCD we set min_dist and max_dist to not discard points
    // TODO: Find better solution instead of hardcoding
    if (FLAGS_lcd_disable_stereo_match_depth_check) {
      lcd_stereo_params.min_point_dist_ = 0.01;
      lcd_stereo_params.max_point_dist_ = 100.0;
    }
    stereo_matcher_ =
        std::make_unique<StereoMatcher>(stereo_camera_, lcd_stereo_params);
  } else {
    VLOG(5) << "LoopClosureDetector initializing in mono mode.";
  }

  // Initialize the ORB feature detector object:
  feature_detector_ = cv::ORB::create(lcd_params_.nfeatures_,
                                      lcd_params_.scale_factor_,
                                      lcd_params_.nlevels_,
                                      lcd_params_.edge_threshold_,
                                      lcd_params_.first_level_,
                                      lcd_params_.WTA_K_,
                                      lcd_params_.score_type_,
                                      lcd_params_.patch_sze_,
                                      lcd_params_.fast_threshold_);

  // Initialize our feature matching object:
  feature_matcher_ = cv::DescriptorMatcher::create(lcd_params_.matcher_type_);

  // Load ORB vocabulary:

  std::unique_ptr<OrbVocabulary> vocab;
  if (preloaded_vocab && preloaded_vocab->vocab) {
    vocab = std::move(preloaded_vocab->vocab);
  } else {
    vocab = loadOrbVocabulary();
  }

  // Initialize db_:
  db_ = std::make_unique<OrbDatabaseWrapper>(*vocab);
}

DBoWLoopClosureDetector::~DBoWLoopClosureDetector() {
  LOG(INFO) << "LoopClosureDetector desctuctor called.";
}

/* ------------------------------------------------------------------------ */
void DBoWLoopClosureDetector::getNewFeaturesAndDescriptors(
    const cv::Mat& img,
    std::vector<cv::KeyPoint>* keypoints,
    OrbDescriptor* descriptors_mat) {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors_mat);
  // TODO(marcus): switch on feature type (orb, etc) when more are supported
  // Extract ORB features and construct descriptors_vec.
  feature_detector_->detectAndCompute(
      img, cv::Mat(), *keypoints, *descriptors_mat);
}

/* ------------------------------------------------------------------------ */
void DBoWLoopClosureDetector::descriptorMatToVec(
    const OrbDescriptor& descriptors_mat,
    OrbDescriptorVec* descriptors_vec) {
  CHECK_NOTNULL(descriptors_vec);

  // TODO(marcus): tied to ORB, need to generalize!
  int L = feature_detector_->descriptorSize();
  descriptors_vec->resize(descriptors_mat.size().height);

  for (size_t i = 0; i < descriptors_vec->size(); i++) {
    (*descriptors_vec)[i] =
        cv::Mat(1, L, descriptors_mat.type());  // one row only
    descriptors_mat.row(i).copyTo((*descriptors_vec)[i].row(0));
  }
}

/* ------------------------------------------------------------------------ */
void DBoWLoopClosureDetector::detectLoopById(const FrameId& frame_id,
                                             LoopResult* result) {
  const auto frame = cache_.getFrame(frame_id);
  if (!frame) {
    if (result) {
      result->status_ = LCDStatus::NO_MATCHES;
    }

    return;
  }

  DBoW2::BowVector curr_bow_vec;
  db_->transform(frame->descriptors_vec_, curr_bow_vec);
  detectLoop(frame_id, curr_bow_vec, result);
}

/* ------------------------------------------------------------------------ */
void DBoWLoopClosureDetector::detectLoop(const FrameId& frame_id,
                                         const DBoW2::BowVector& bow_vec,
                                         LoopResult* result) {
  CHECK_NOTNULL(result);
  result->query_id_ = {frame_id};

  int max_possible_match_id = frame_id - lcd_params_.recent_frames_window_;
  if (max_possible_match_id < 0) {
    max_possible_match_id = 0;
  }

  // Query for BoW vector matches in database.
  DBoW2::QueryResults query_result;
  db_->query(bow_vec,
             query_result,
             lcd_params_.max_db_results_,
             max_possible_match_id);

  if (query_result.empty()) {
    result->status_ = LCDStatus::NO_MATCHES;
    return;
  }

  double nss_factor = 1.0;
  if (lcd_params_.use_nss_ && latest_global_vec_) {
    nss_factor = db_->getVocabulary()->score(bow_vec, *latest_global_vec_);
  } else {
    LOG_IF(ERROR, !lcd_params_.use_nss_)
        << "Setting use_nss as false is deprecated.";
  }

  if (lcd_params_.use_nss_ && nss_factor < lcd_params_.min_nss_factor_) {
    result->status_ = LCDStatus::LOW_NSS_FACTOR;
    return;
  }

  // Remove low scores from the QueryResults based on nss.
  DBoW2::QueryResults::iterator query_it =
      lower_bound(query_result.begin(),
                  query_result.end(),
                  DBoW2::Result(0, lcd_params_.alpha_ * nss_factor),
                  DBoW2::Result::geq);
  if (query_it != query_result.end()) {
    query_result.resize(query_it - query_result.begin());
  }

  // Begin grouping and checking matches.
  if (query_result.empty()) {
    result->status_ = LCDStatus::LOW_SCORE;
    return;
  }

  // Set best candidate to highest scorer.
  result->match_id_ = {query_result[0].Id};

  // Compute islands in the matches.
  // An island is a group of matches with close frame_ids.
  std::vector<MatchIsland> islands;
  lcd_tp_wrapper_->computeIslands(&query_result, &islands);

  if (islands.empty()) {
    result->status_ = LCDStatus::NO_GROUPS;
    return;
  }

  // Find the best island grouping using MatchIsland sorting.
  const MatchIsland& best_island =
      *std::max_element(islands.begin(), islands.end());

  // Run temporal constraint check on this best island.
  bool pass_temporal_constraint =
      lcd_tp_wrapper_->checkTemporalConstraint(frame_id, best_island);

  if (!pass_temporal_constraint) {
    result->status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
    return;
  }

  verifyAndRecoverPose(result);
}

/* ------------------------------------------------------------------------ */
void DBoWLoopClosureDetector::setDatabase(const OrbDatabaseWrapper& db) {
  db_ = std::make_unique<OrbDatabaseWrapper>(db);
}

/* ------------------------------------------------------------------------ */
void DBoWLoopClosureDetector::setVocabulary(const OrbVocabulary& voc) {
  db_->setVocabulary(voc);
}

}  // namespace VIO
