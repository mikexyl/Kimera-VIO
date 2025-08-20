#pragma once

#include <KimeraRPGO/RobustSolver.h>

#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/RgbdFrame.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/StereoMatcher.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/loopclosure/LandmarkManager.h"
#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/loopclosure/LcdThirdPartyWrapper.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

class LoopClosureDetectorBase {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetectorBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureDetectorBase);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using IsBackendQueueFilledCallback = std::function<bool()>;

  LoopClosureDetectorBase() = default;
  virtual ~LoopClosureDetectorBase() = default;

  /* ------------------------------------------------------------------------
   */
  /**
   * @brief Register a loop closure between two frames in a threadsafe manner
   * @param[in] query_id most recent frame
   * @param[in] match_id previous frame that was matched to query
   */
  virtual LoopResult registerFrames(FrameId query_id, FrameId match_id) = 0;

  virtual LcdOutput::UniquePtr spinOnce(const LcdInput& input) = 0;

  /* ------------------------------------------------------------------------
   */
  /** @brief Register callback for checking the size of the input queue.
   * Knowing this can help determine when to optimize the factor graph and
   * when to wait for additional inputs to be added first.
   * @param[in] cb A callback function.
   */
  virtual inline void registerIsBackendQueueFilledCallback(
      const IsBackendQueueFilledCallback& cb) = 0;
};

template <typename Database, typename FeatureDector, typename FeatureMatcher>
class LoopClosureDetector : public LoopClosureDetectorBase {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LoopClosureDetector(
      const LoopClosureDetectorParams& lcd_params,
      const CameraParams& tracker_cam_params,
      const gtsam::Pose3& B_Pose_Cam,
      const std::optional<VIO::StereoCamera::ConstPtr>& stereo_camera,
      const std::optional<StereoMatchingParams>& stereo_matching_params,
      const std::optional<VIO::RgbdCamera::ConstPtr>& rgbd_camera,
      bool log_output);

  virtual ~LoopClosureDetector() = default;

  /* ------------------------------------------------------------------------
   */
  /**
   * @brief Register a loop closure between two frames in a threadsafe manner
   * @param[in] query_id most recent frame
   * @param[in] match_id previous frame that was matched to query
   */
  LoopResult registerFrames(FrameId query_id, FrameId match_id) override {
    LoopResult result;
    result.query_id_ = {query_id};
    result.match_id_ = {match_id};
    verifyAndRecoverPose(&result);
    return result;
  }

  /* ------------------------------------------------------------------------ */
  /** @brief Computes the indices of keypoints that match between two frames.
   * @param[in] ref_descriptors The descriptors from the query frame.
   * @param[in] cur_descriptors The descriptors from the match frame.
   * @param[out] matches_match_query Map of matching keypoint indices between
   * match frame and query frame.
   * @param[in] cut_matches If true, Lowe's Ratio Test will be used to cut
   *  out bad matches before sending output.
   */
  virtual void computeDescriptorMatches(
      const typename Database::Desc& ref_descriptors,
      const typename Database::Desc& cur_descriptors,
      KeypointMatches* matches_match_query,
      bool cut_matches = false) const {
    CHECK_NOTNULL(matches_match_query);
    matches_match_query->clear();

    // Get two best matches between frame descriptors.
    std::vector<DMatchVec> matches;
    double lowe_ratio = 1.0;
    if (cut_matches) lowe_ratio = lcd_params_.lowe_ratio_;

    feature_matcher_->knnMatch(cur_descriptors, ref_descriptors, matches, 2u);

    const size_t& n_matches = matches.size();
    for (size_t i = 0; i < n_matches; i++) {
      const DMatchVec& match = matches[i];
      if (match.size() < 2) continue;
      if (match[0].distance < lowe_ratio * match[1].distance) {
        // Store trainIdx first because this represents the kpt from the
        // ref frame. For LCD, this would be the match and not the query.
        // For tracker outlier-rejection we use (ref, cur) always.
        matches_match_query->push_back(
            std::make_pair(match[0].trainIdx, match[0].queryIdx));
      }
    }
  }

  virtual void computeDescriptorMatches(const LCDFrame& ref,
                                        const LCDFrame& curr,
                                        KeypointMatches* matches_match_query,
                                        bool cut_matches = false) const {
    CHECK_NOTNULL(matches_match_query);

    return computeDescriptorMatches(ref.descriptors_mat_,
                                    curr.descriptors_mat_,
                                    matches_match_query,
                                    cut_matches);
  }

  void verifyAndRecoverPose(LoopResult* result) {
    CHECK_NOTNULL(result);
    CHECK_EQ(result->match_id_.size(), 1u) << "not impleeented yet";

    result->match_id_.emplace_back(result->query_id_[0]);
    result->query_id_.emplace_back(result->match_id_[0]);
    result->relative_pose_.resize(2);

    const auto match_frame = cache_.getFrame(result->match_id_[0]);
    const auto query_frame = cache_.getFrame(result->query_id_[0]);
    if (!match_frame || !query_frame) {
      VLOG(1) << "LoopClosureDetector: No match or query frame found for "
              << "match_id: " << result->match_id_[0]
              << ", query_id: " << result->query_id_[0];
      result->status_ = LCDStatus::NO_MATCHES;
      return;
    }

    // Find correspondences between keypoints.
    KeypointMatches matches_match_query;
    computeDescriptorMatches(
        *match_frame, *query_frame, &matches_match_query, true);
    VLOG(1) << "LoopClosureDetector: Found " << matches_match_query.size()
            << " kp matches between frames " << result->match_id_[0] << " and "
            << result->query_id_[0];

    // Perform geometric verification check.
    gtsam::Pose3 camMatch_T_camQuery_2d;
    std::vector<int> inliers;
    bool pass_geometric_verification =
        geometricVerificationCam2d2d(*match_frame,
                                     *query_frame,
                                     matches_match_query,
                                     &camMatch_T_camQuery_2d,
                                     &inliers);

    if (!pass_geometric_verification) {
      result->status_ = LCDStatus::FAILED_GEOM_VERIFICATION;
      return;
    }

    auto status = recoverPoseBody(*match_frame,
                                  *query_frame,
                                  camMatch_T_camQuery_2d,
                                  matches_match_query,
                                  &(result->relative_pose_[0]),
                                  &(result->relative_pose_[1]),
                                  &inliers);
    result->status_ = status;
  }

  virtual LcdOutput::UniquePtr spinOnce(const LcdInput& input) override;

  /* ------------------------------------------------------------------------
   */
  /** @brief Register callback for checking the size of the input queue.
   * Knowing this can help determine when to optimize the factor graph and
   * when to wait for additional inputs to be added first.
   * @param[in] cb A callback function.
   */
  inline void registerIsBackendQueueFilledCallback(
      const IsBackendQueueFilledCallback& cb) override {
    is_backend_queue_filled_cb_ = cb;
  }

  /* ------------------------------------------------------------------------
   */
  /** @brief Initializes the RobustSolver member with an initial prior factor,
   *  which can be the first OdometryFactor given by the Backend.
   * @param[in] factor An OdometryFactor representing the pose between the
   *  initial state of the vehicle and the first keyframe.
   */
  void initializePGO(const OdometryFactor& factor);

  /* ------------------------------------------------------------------------
   */
  /** @brief Adds an odometry factor to the PGO and optimizes the trajectory.
   *  No actual optimization is performed on the RPGO side for odometry.
   * @param[in] factor An OdometryFactor representing the Backend's guess for
   *  odometry between two consecutive keyframes.
   */
  void addOdometryFactorAndOptimize(const OdometryFactor& factor);

  /* ------------------------------------------------------------------------
   */
  /** @brief Adds a loop-closure factor to the PGO and optimizes the
   * trajectory.
   * @param[in] factor A LoopClosureFactor representing the relative pose
   *  between two frames that are not (necessarily) consecutive.
   */
  void addLoopClosureFactorAndOptimize(const LoopClosureFactor& factor);

  /* ------------------------------------------------------------------------
   */
  /** @brief Processed a single frame and adds it to relevant internal
   * databases. Also generates associated bearing vectors for PnP.
   * @param[in] frame A Frame object with one images, landmarks, and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @param[in] points_with_ids A PointsWithIdMap object obtained from the
   * backend's output. Expressed in world frame.
   * @param[in] W_Pose_Blkf A gtsam::Pose3 representing the VIO estimate of
   * the pose of the body wrt to world frame at the given keframe.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddMonoFrame(const Frame& frame,
                                 const PointsWithIdMap& W_points_with_ids,
                                 const gtsam::Pose3& W_Pose_Blkf);

  /* ------------------------------------------------------------------------
   */
  /** @brief Processed a single rgbd-frame and adds it to relevant internal
   * databases.
   * @param[in] rgbd_frame A RgbdFrame object with two images and a pose to
   * the body frame at a minimum. Other fields may also be populated.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddRgbdFrame(const RgbdFrame& rgbd_frame);

  /* ------------------------------------------------------------------------
   */
  /** @brief Processed a single stereo-frame and adds it to relevant internal
   * databases.
   * @param[in] stereo_frame A StereoFrame object with two images and a pose
   * to the body frame at a minimum. Other fields may also be populated.
   * @return The local ID of the frame after it is added to the databases.
   */
  FrameId processAndAddStereoFrame(const StereoFrame& stereo_frame);

  /**
   * @brief Get cache of LCD keyframes.
   * @return The FrameCache containing keyframe information.
   */
  const FrameCache& getFrameCache() const { return cache_; }

  /* ------------------------------------------------------------------------
   */
  /** @brief Returns the pose between the inertial world-reference frame and
   * the "map" frame, which is the error between the VIO and the PGO
   * trajectories.
   * @return The pose of the map frame relative to the world frame.
   */
  const gtsam::Pose3 getWPoseMap() const;

  /* ------------------------------------------------------------------------
   */
  /** @brief Returns the pose between the optimized world reference frame
   * (map) and the VIO world reference frame (odom).
   * @return The pose of the odom frame relative to the map frame.
   */
  const gtsam::Pose3 getMapPoseOdom() const;

  /* ------------------------------------------------------------------------
   */
  /** @brief Returns the values of the PGO, which is the full trajectory of
   * the PGO.
   * @return The gtsam::Values (poses) of the PGO.
   */
  const gtsam::Values getPGOTrajectory() const;

  /* ------------------------------------------------------------------------
   */
  /** @brief Returns the Nonlinear-Factor-Graph from the PGO.
   * @return The gtsam::NonlinearFactorGraph of the optimized trajectory from
   *  the PGO.
   */
  const gtsam::NonlinearFactorGraph getPGOnfg() const;

  /* ------------------------------------------------------------------------
   */
  /** @brief Detect features in frame for use with BoW and return keypoints
   * and descriptors. Currently only ORB features are supported.
   * @param[in] img Image from which to get features and descriptors.
   * @param[out] keypoints The ORB keypoints that are detected in the image.
   * @param[out] descriptors_mat The descriptors associated with the ORB
   * keypoints in a matrix form.
   * @param[out] descriptors_vec The descriptors vectorized.
   */
  virtual void getNewFeaturesAndDescriptors(
      const cv::Mat& img,
      std::vector<cv::KeyPoint>* keypoints,
      typename Database::Desc* descriptors_mat) = 0;

  virtual void getNewFeaturesAndDescriptors(
      const Frame& frame,
      std::vector<cv::KeyPoint>* keypoints,
      typename Database::Desc* descriptors_mat) {
    getNewFeaturesAndDescriptors(frame.img_, keypoints, descriptors_mat);
  }
  /* ------------------------------------------------------------------------
   */
  /** @brief Convert an ORB descriptor from matrix form to vector form for
   * use with BoW.
   * @param[in] descriptors_mat An Database::Desc matrix with input
   * descriptors
   * @param[out] descriptors_vec The descriptors in vectorize format
   */
  virtual void descriptorMatToVec(
      const typename Database::DescMat& descriptors_mat,
      typename Database::DescVector* descriptors_vec) {}

  virtual void descriptorMatToVec(
      const Frame& frame,
      const typename Database::DescMat& descriptors_mat,
      typename Database::DescVector* descriptors_vec) {
    descriptorMatToVec(descriptors_mat, descriptors_vec);
  }

  /* ------------------------------------------------------------------------
   */
  /** @brief Clears all keypoints and features from an input StereoFrame and
   *  fills it with ORB features.
   * @param[in] keypoints A vector of KeyPoints representing the ORB keypoints
   *  identified by an ORB detector.
   * @param[out] A StereoFrame initially filled with front-end features,
   *  which is then replaced with ORB features from the keypoints parameter.
   */
  // TODO(marcus): utils and reorder (or just static)
  void rewriteStereoFrameFeatures(const std::vector<cv::KeyPoint>& keypoints,
                                  StereoFrame* stereo_frame) const;

  /* ------------------------------------------------------------------------
   */
  bool geometricVerificationCam2d2d(const LCDFrame& ref_frame,
                                    const LCDFrame& cur_frame,
                                    const KeypointMatches& matches_match_query,
                                    gtsam::Pose3* camMatch_T_camQuery_2d,
                                    std::vector<int>* inliers) {
    CHECK_NOTNULL(camMatch_T_camQuery_2d);
    CHECK_NOTNULL(inliers);

    TrackingStatusPose result;
    if (matches_match_query.empty()) {
      VLOG(10) << "LoopClosureDetector: failure to find matching keypoints "
                  "between reference and current frames."
               << "\n reference id: " << ref_frame.id_
               << " current id: " << cur_frame.id_;
      result = std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
    } else {
      result =
          tracker_->geometricOutlierRejection2d2d(ref_frame.bearing_vectors_,
                                                  cur_frame.bearing_vectors_,
                                                  matches_match_query,
                                                  inliers);

      *camMatch_T_camQuery_2d = result.second;
    }

    if (logger_)
      logger_->logGeometricVerification(
          ref_frame.timestamp_, cur_frame.timestamp_, *camMatch_T_camQuery_2d);

    return result.first == TrackingStatus::VALID;
  }

  /* ------------------------------------------------------------------------
   */
  /** @brief Determine the 3D pose betwen two frames.
   * @param[in] ref_id The frame ID of the match image in the database.
   * @param[in] cur_id The frame ID of the query image in the database.
   * @param[in] camMatch_T_camQuery_2d The relative pose between the match
   * frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[out] bodyMatch_T_bodyQuery_3d The 3D pose between the match
   * frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[out] inliers The inliers to use from the keypoint matches,
   * determined at the geometricVerificationCam2d2d stage.
   * @return True if the pose is recovered successfully, false otherwise.
   */
  LCDStatus recoverPoseBody(const LCDFrame& ref_frame,
                            const LCDFrame& cur_frame,
                            const gtsam::Pose3& camMatch_T_camQuery_2d,
                            const KeypointMatches& matches_query_match,
                            gtsam::Pose3* bodyMatch_T_bodyQuery_3d,
                            gtsam::Pose3* bodyQuery_T_bodyMatch_3d,
                            std::vector<int>* inliers);

  virtual LCDFrame::Ptr processMonoPnP(const Frame& frame,
                                       const PointsWithIdMap& W_points_with_ids,
                                       const gtsam::Pose3& W_Pose_Blkf) = 0;

  /* ------------------------------------------------------------------------
   */
  /** @brief Refine relative pose given by ransac using smart factors.
   * @param[in] ref_id The frame ID of the match image in the database.
   * @param[in] cur_id The frame ID of the query image in the database.
   * @param[in] camMatch_T_camQuery_3d The relative pose between the match
   * frame
   *  and the query frame, in the coordinates of the match frame.
   * @param[in] inlier correspondences (from ransac) in the query frame
   * @param[in] inlier correspondences (from ransac) in the match frame
   * @return refined relative pose
   */
  gtsam::Pose3 refinePoses(const StereoLCDFrame& ref_frame,
                           const StereoLCDFrame& cur_frame,
                           const gtsam::Pose3& camMatch_T_camQuery_3d,
                           const KeypointMatches& matches_query_match);

  /* ------------------------------------------------------------------------
   */
  /** @brief Gives the transform between two frames in the body frame given
   *  that same transform in the camera frame.
   * @param[in] camMatch_T_camQuery The relative pose between two frames in
   * the camera coordinate frame.
   * @param[out] bodyMatch_T_bodyQuery The relative pose between two frames in
   * the
   *  body coordinate frame.
   */
  // TODO(marcus): these should be private or util
  void transformCameraPoseToBodyPose(
      const gtsam::Pose3& camMatch_T_camQuery,
      gtsam::Pose3* bodyMatch_T_bodyQuery) const {
    CHECK_NOTNULL(bodyMatch_T_bodyQuery);
    *bodyMatch_T_bodyQuery =
        B_Pose_Cam_ * camMatch_T_camQuery * B_Pose_Cam_.inverse();
  }

  /* ------------------------------------------------------------------------
   */
  /** @brief The inverse of transformCameraPoseToBodyPose.
   * @param[in] bodyMatch_T_bodyQuery The relative pose between two frames in
   * the
   *  body coordinate frame.
   * @param[out] camMatch_T_camQuery The relative pose between two frames in
   * the camera coordinate frame.
   * @return
   */
  void transformBodyPoseToCameraPose(const gtsam::Pose3& bodyMatch_T_bodyQuery,
                                     gtsam::Pose3* camMatch_T_camQuery) const {
    CHECK_NOTNULL(camMatch_T_camQuery);
    *camMatch_T_camQuery =
        B_Pose_Cam_.inverse() * bodyMatch_T_bodyQuery * B_Pose_Cam_;
  }

  /* ------------------------------------------------------------------------
   */
  /** @brief Runs all checks on a frame and determines whether it a
   * loop-closure with a previous frame or not. Fills the LoopResult with this
   *         information.
   * @param[in] frame_id A FrameId representing the ID of the latest LCDFrame
   * added to the database, which will be used to detect loop-closures.
   * @param[in] bow_vec A descriptor vector for the latest LCDFrame to be
   * scored and added to the database.
   * @param[out] result A pointer to the LoopResult that is filled with the
   *                    result of the loop-closure detection stage.
   */
  virtual void detectLoop(const FrameId& frame_id,
                          const typename Database::GlobalDesc& bow_vec,
                          LoopResult* result) = 0;

  virtual void print() const {
    lcd_params_.print();
    // TODO(marcus): implement
  }

  virtual std::map<int, double> globalDescToMap(
      const typename Database::GlobalDesc& global_desc) = 0;

  virtual void cleanFrame(const FrameId& frame_id) {
    auto frame = cache_.getFrame(frame_id);
    if (frame) {
      cleanFrame(frame);
    } else {
      LOG(WARNING) << "LoopClosureDetector: Attempted to clean frame with ID "
                   << frame_id << " but it does not exist in the cache.";
    }
  }

  virtual void cleanFrame(const LCDFrame::Ptr& frame) {}

 protected:
  enum class LcdState {
    Bootstrap,  //! Lcd is initializing
    Nominal     //! Lcd is running in nominal mode
  };
  LcdState lcd_state_ = LcdState::Bootstrap;

  LoopClosureDetectorParams lcd_params_;

  IsBackendQueueFilledCallback is_backend_queue_filled_cb_;

  // TODO(Toni): we should be using the FeatureDetector/Description class...
  // ORB extraction and matching members
  cv::Ptr<FeatureDector> feature_detector_;
  cv::Ptr<FeatureMatcher> feature_matcher_;

  // Store camera parameters and StereoFrame stuff once
  StereoCamera::ConstPtr stereo_camera_;
  StereoMatcher::UniquePtr stereo_matcher_;

  // Rgbd specific camera
  RgbdCamera::ConstPtr rgbd_camera_;

  // BoW database
  std::unique_ptr<Database> db_;
  FrameCache cache_;
  FrameIDTimestampMap timestamp_map_;

  gtsam::SharedNoiseModel shared_noise_model_;

  // Robust PGO members
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;
  std::pair<gtsam::Symbol, gtsam::Pose3> W_Pose_B_kf_vio_;
  gtsam::Pose3 B_Pose_Cam_;

  std::unique_ptr<typename Database::GlobalDesc> latest_global_vec_;

  Tracker::UniquePtr tracker_;

  std::unique_ptr<LcdLandmarkManager> landmark_manager_{nullptr};

  // Queue-checking callback
  int num_lc_unoptimized_;

  std::unique_ptr<LcdThirdPartyWrapper> lcd_tp_wrapper_;

  // Logging members
  std::unique_ptr<LoopClosureDetectorLogger> logger_;
  LcdDebugInfo debug_info_;

  // Parameter members
  const bool log_output_ = false;
};

}  // namespace VIO

#include "LoopClosureDetector-inl.h"