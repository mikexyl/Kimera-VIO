#include "kimera-vio/loopclosure/VLADLoopClosureDetector.h"

namespace VIO {

size_t VLADLoopClosureDetector::new_seq_id_ = 0;

void VLADLoopClosureDetector::computeSequenceGlobalDesc(
    const FrameId target_frame_id) {
  auto new_frame = cache_.getFrame(target_frame_id);
  new_frame->seq_id_ = new_seq_id_;
  new_frame->descriptors_vec_.clear();

  if (target_frame_id % lcd_params_.jist_seq_interval_ == 0) {
    new_seq_frames_.emplace_back(new_frame);
  }

  if (new_seq_frames_.size() ==
      static_cast<size_t>(lcd_params_.jist_seq_length_)) {
    // We have enough frames for a sequence. Proceed with loop detection.
    VLOG(2) << "VLADLoopClosureDetector: Processing sequence of size: "
            << new_seq_frames_.size() << ".";

    auto global_desc = cv::Mat();
    db_->transform(new_seq_frames_, global_desc);

    for (auto seq_frame : new_seq_frames_) {
      seq_frame->descriptors_vec_.clear();
      seq_frame->clearImage();
    }

    new_frame->descriptors_vec_.push_back(global_desc.clone());

    new_seq_frames_.clear();
    new_seq_id_++;
  }
}

void VLADLoopClosureDetector::detectLoop(const FrameId& frame_id,
                                         LoopResult* result,
                                         FrameId* query_frame,
                                         FrameIdSet* global_candidates) {
  throw std::runtime_error("removed");
}

void VLADLoopClosureDetector::detectLoopOutsideLocalWindow(
    const FrameId& frame_id,
    LoopResult* result,
    FrameId* query_frame,
    FrameIdSet* global_candidates) {
  CHECK_NOTNULL(result);
  CHECK_NOTNULL(db_);
  result->query_id_ = {frame_id};
  if (query_frame) {
    *query_frame = frame_id;
  }

  cv::Mat global_desc = db_->get(frame_id);
  CHECK(!global_desc.empty())
      << "VLADLoopClosureDetector: Global descriptor for frame " << frame_id
      << " is empty.";

  if (frame_id < static_cast<FrameId>(lcd_params_.recent_frames_window_ +
                                      lcd_params_.max_db_results_ +
                                      lcd_params_.local_window_size_)) {
    VLOG(1) << "VLADLoopClosureDetector: Not enough frames processed yet. "
            << "Skipping loop closure detection.";
    result->status_ = LCDStatus::NO_MATCHES;
    return;
  }

  int max_possible_match_id = frame_id - lcd_params_.recent_frames_window_;
  if (max_possible_match_id < 0) {
    max_possible_match_id = 0;
  }

  int top_k = lcd_params_.max_db_results_ + lcd_params_.recent_frames_window_;

  Database::Database::QueryResults query_result(top_k, -1);
  Database::Database::QueryDistances query_distance(
      top_k, std::numeric_limits<float>::max());

  db_->search(global_desc, top_k, query_result, query_distance);

  // remove -1 from query_result
  for (size_t i = 0; i < query_result.size(); ++i) {
    if (query_result[i] == -1 or query_result[i] >= max_possible_match_id) {
      query_result.erase(query_result.begin() + i);
      query_distance.erase(query_distance.begin() + i);
      --i;  // Adjust index after erasure.
    }
  }

  if (VLOG_IS_ON(1)) {
    // print query results and distances
    std::stringstream ss;
    ss << "VLADLoopClosureDetector: query results: ";
    for (size_t i = 0; i < query_result.size(); ++i)
      ss << "{" << query_result[i] << ", " << query_distance[i] << "} ";
    VLOG(1) << ss.str();
  }

  // if the query result has recent frames, throw error
  for (const auto& id : query_result) {
    if (id >= max_possible_match_id) {
      throw std::runtime_error(
          "VLADLoopClosureDetector: Query result contains recent frames. "
          "This should not happen.");
    }
  }

  if (query_result.empty()) {
    VLOG(1) << "VLADLoopClosureDetector: No matches found.";
    result->status_ = LCDStatus::NO_MATCHES;
    return;
  }

  auto faiss_to_dbow_queryresults =
      [&](Database::Database::QueryResults& query_result,
          Database::Database::QueryDistances& query_distance)
      -> DBoW2::QueryResults {
    DBoW2::QueryResults dbow_query_result;
    for (size_t i = 0; i < query_result.size(); ++i) {
      DBoW2::Result result;
      result.Id = query_result[i];
      result.Score = query_distance[i];
      dbow_query_result.push_back(result);
    }
    return dbow_query_result;
  };

  auto dbow_query_result =
      faiss_to_dbow_queryresults(query_result, query_distance);

  // Begin grouping and checking matches.
  if (query_result.empty()) {
    result->status_ = LCDStatus::LOW_SCORE;
    return;
  }

  // Set best candidate to the lowest label index
  result->match_id_ = {static_cast<unsigned long>(query_result[0])};
  if (global_candidates) {
    global_candidates->clear();
    for (const auto& id : query_result) {
      global_candidates->insert(static_cast<FrameId>(id));
    }
  }

  // Compute islands in the matches.
  // An island is a group of matches with close frame_ids.
  std::vector<MatchIsland> islands;
  lcd_tp_wrapper_->computeIslands(&dbow_query_result, &islands);

  if (islands.empty()) {
    VLOG(1) << "VLADLoopClosureDetector: No islands found in matches.";
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
    VLOG(1) << "VLADLoopClosureDetector: Failed temporal constraint check.";
    result->status_ = LCDStatus::FAILED_TEMPORAL_CONSTRAINT;
    return;
  }

  verifyAndRecoverPose(result);
  if (result->status_ != LCDStatus::LOOP_DETECTED) {
    VLOG(1) << "VLADLoopClosureDetector: Failed pose verification or recovery.";
  }
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

  *descriptors_mat = frame.descriptors_;
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

LCDFrame::Ptr VLADLoopClosureDetector::processMonoPnP(
    const Frame& frame,
    const PointsWithIdMap& W_points_with_ids,
    const gtsam::Pose3& W_Pose_Blkf) {
  size_t nr_kpts = frame.keypoints_.size();
  CHECK_EQ(frame.landmarks_.size(), nr_kpts);
  CHECK_EQ(frame.versors_.size(), nr_kpts);
  CHECK_EQ(frame.keypoints_undistorted_.size(), nr_kpts);

  auto keypoints = frame.keypoints_;

  BearingVectors undistorted_bearing_vectors;
  for (const auto& pt : keypoints) {
    undistorted_bearing_vectors.push_back(
        UndistorterRectifier::GetBearingVector(pt, frame.cam_param_));
  }

  std::vector<cv::KeyPoint> keypoints_to_save;
  for (const auto& pt : keypoints) {
    keypoints_to_save.push_back(cv::KeyPoint(pt.x, pt.y, 0.0f));
  }

  auto lcd_frame = std::make_shared<LCDFrame>(
      frame.timestamp_,
      FrameCache::NEW_ID,
      frame.id_,
      keypoints_to_save,
      Landmarks(),
      std::vector<cv::Mat>{frame.xfeat_M1_, frame.xfeat_x_prep_},
      frame.descriptors_,
      undistorted_bearing_vectors,
      W_Pose_Blkf);
  lcd_frame->landmark_ids = frame.landmarks_;
  lcd_frame->cam_params_ = frame.cam_param_;

  return lcd_frame;
}

}  // namespace VIO