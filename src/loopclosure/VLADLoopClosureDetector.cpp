#include "kimera-vio/loopclosure/VLADLoopClosureDetector.h"

namespace VIO {
class VLADLoopClosureDetector;

DEFINE_double(max_nss_vlad_distance,
              0.06,
              "Maximum NSS distance for VLAD loop closure detection.");

void VLADLoopClosureDetector::detectLoop(
    const FrameId& frame_id,
    const Database::GlobalDesc& global_desc,
    LoopResult* result) {
  CHECK_NOTNULL(result);
  CHECK_NOTNULL(db_);
  result->query_id_ = frame_id;

  int max_possible_match_id = frame_id - lcd_params_.recent_frames_window_;
  if (max_possible_match_id < 0) {
    max_possible_match_id = 0;
  }

  Database::Database::QueryResults query_result(lcd_params_.max_db_results_);
  Database::Database::QueryDistances query_distance(
      lcd_params_.max_db_results_, std::numeric_limits<float>::max());

  db_->search(global_desc,
              lcd_params_.max_db_results_,
              query_result,
              query_distance,
              max_possible_match_id);

  db_->add(global_desc);

  if (query_result.empty()) {
    result->status_ = LCDStatus::NO_MATCHES;
    return;
  }

  double nss_distance = 0.0;
  if (lcd_params_.use_nss_ && latest_global_vec_) {
    nss_distance = db_->distance(global_desc, *latest_global_vec_);
  } else {
    LOG_IF(ERROR, !lcd_params_.use_nss_)
        << "Setting use_nss as false is deprecated.";
  }

  if (lcd_params_.use_nss_ && nss_distance > FLAGS_max_nss_vlad_distance) {
    result->status_ = LCDStatus::LOW_NSS_FACTOR;
    return;
  }

  auto faiss_to_dbow_queryresults =
      [&](Database::Database::QueryResults& query_result,
          Database::Database::QueryDistances& query_distance)
      -> DBoW2::QueryResults {
    DBoW2::QueryResults dbow_query_result;
    for (size_t i = 0; i < query_result.size(); ++i) {
      static constexpr double kL2DistanceToScoreFactor = 10.0;
      float score = std::exp(-kL2DistanceToScoreFactor * query_distance[i]);
      DBoW2::Result result;
      result.Id = query_result[i];
      result.Score = score;
      dbow_query_result.push_back(result);
    }
    return dbow_query_result;
  };

  auto dbow_query_result =
      faiss_to_dbow_queryresults(query_result, query_distance);

  // Remove high distances from the QueryResults based on nss.
  for (size_t i = 0; i < query_result.size(); ++i) {
    if (query_distance[i] > nss_distance) {
      query_result.erase(query_result.begin() + i);
      query_distance.erase(query_distance.begin() + i);
      --i;  // Adjust index after erasure.
    }
  }

  // Set best candidate to highest scorer.
  result->match_id_ = query_result[0];

  // Compute islands in the matches.
  // An island is a group of matches with close frame_ids.
  std::vector<MatchIsland> islands;
  lcd_tp_wrapper_->computeIslands(&dbow_query_result, &islands);

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

void VLADLoopClosureDetector::getNewFeaturesAndDescriptors(
    const Frame& frame,
    std::vector<cv::KeyPoint>* keypoints,
    typename Database::Desc* descriptors_mat) {
  CHECK_NOTNULL(keypoints);
  CHECK_NOTNULL(descriptors_mat);

  for (auto const& keypoint : frame.keypoints_) {
    keypoints->push_back(cv::KeyPoint(
        keypoint.x, keypoint.y, 0.0f));  // size is not used in VLAD
  }
}

void VLADLoopClosureDetector::descriptorMatToVec(
    const Frame& frame,
    const typename Database::DescMat& descriptors_mat,
    typename Database::DescVector* descriptors_vec) {
  CHECK(not frame.xfeat_M1_.empty());
  CHECK(not frame.xfeat_x_prep_.empty());

  CHECK_NOTNULL(descriptors_vec);
  descriptors_vec->clear();
  descriptors_vec->push_back(frame.xfeat_M1_);
  descriptors_vec->push_back(frame.xfeat_x_prep_);
}

}  // namespace VIO