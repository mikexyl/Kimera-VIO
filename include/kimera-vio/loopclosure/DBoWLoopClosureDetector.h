/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LoopClosureDetector.h
 * @brief  Pipeline for detection and reporting of Loop Closures between frames
 * @author Marcus Abate
 * @author Luca Carlone
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

// Add DBoW2 headers for TemplatedVocabulary and TemplatedDatabase
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedDatabase.h>
#include <DBoW2/TemplatedVocabulary.h>

#include "kimera-vio/frontend/RgbdCamera.h"
#include "kimera-vio/frontend/StereoCamera.h"
#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/loopclosure/LoopClosureDetector.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

/* ------------------------------------------------------------------------ */
// Forward declare KimeraRPGO, a private dependency.
namespace KimeraRPGO {
class RobustSolver;
}

namespace DBoW2 {
class BowVector;
class FORB;

template <class D, class F>
class TemplatedVocabulary;

template <class D, class F>
class TemplatedDatabase;

}  // namespace DBoW2

typedef DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB> OrbVocabulary;
struct OrbDatabaseWrapper : DBoW2::TemplatedDatabase<cv::Mat, DBoW2::FORB> {
  using Base = DBoW2::TemplatedDatabase<cv::Mat, DBoW2::FORB>;
  using GlobalDesc = DBoW2::BowVector;
  using Desc = VIO::OrbDescriptor;
  using DescVector = VIO::OrbDescriptorVec;
  using DescMat = cv::Mat;

  // default constructor
  template <typename... Args>
  OrbDatabaseWrapper(Args&&... args) : Base(std::forward<Args>(args)...) {}

  void transform(const DescVector& desc_vec, GlobalDesc& bow_vec) const {
    return getVocabulary()->transform(desc_vec, bow_vec);
  }
};

namespace VIO {

class LcdThirdPartyWrapper;  // forward declare to avoid DBoW2 header

// quick wrapper class to get around forward declaration of OrbVocabulary
struct PreloadedVocab {
  KIMERA_DELETE_COPY_CONSTRUCTORS(PreloadedVocab);
  using Ptr = std::unique_ptr<PreloadedVocab>;

  PreloadedVocab();
  PreloadedVocab(PreloadedVocab&& other);
  ~PreloadedVocab();

  std::unique_ptr<OrbVocabulary> vocab;
};

/* ------------------------------------------------------------------------ */
class DBoWLoopClosureDetector
    : public LoopClosureDetector<OrbDatabaseWrapper,
                                 cv::ORB,
                                 cv::DescriptorMatcher> {
 public:
  KIMERA_POINTER_TYPEDEFS(DBoWLoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(DBoWLoopClosureDetector);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* ------------------------------------------------------------------------ */
  /** @brief Constructor: detects loop-closures and updates internal PGO.
   * @param[in] lcd_params Parameters for the instance of LoopClosureDetector.
   * @param[in] log_output Output-logging flag. If set to true, the logger is
   *  instantiated and output/statistics are logged at every spinOnce().
   */
  DBoWLoopClosureDetector(
      const LoopClosureDetectorParams& lcd_params,
      const CameraParams& tracker_cam_params,
      const gtsam::Pose3& B_Pose_Cam,
      const std::optional<VIO::StereoCamera::ConstPtr>& stereo_camera =
          std::nullopt,
      const std::optional<StereoMatchingParams>& stereo_matching_params =
          std::nullopt,
      const std::optional<VIO::RgbdCamera::ConstPtr>& rgbd_camera =
          std::nullopt,
      bool log_output = false,
      PreloadedVocab::Ptr&& preloaded_vocab = nullptr);

  /* ------------------------------------------------------------------------ */
  virtual ~DBoWLoopClosureDetector();

  /* ------------------------------------------------------------------------ */
  /** @brief Find a loop closure given a frame id
   * @param[in] frame_id A FrameId representing the ID of the latest LCDFrame
   *                     added to the database.
   * @param[out] result A pointer to the LoopResult that is filled with the
   *                    result of the loop-closure detection stage.
   */
  void detectLoopById(const FrameId& frame_id, LoopResult* result);

  /* ------------------------------------------------------------------------ */
  /** @brief Runs all checks on a frame and determines whether it a loop-closure
   *         with a previous frame or not. Fills the LoopResult with this
   *         information.
   * @param[in] frame_id A FrameId representing the ID of the latest LCDFrame
   * added to the database, which will be used to detect loop-closures.
   * @param[in] bow_vec A descriptor vector for the latest LCDFrame to be scored
   *                    and added to the database.
   * @param[out] result A pointer to the LoopResult that is filled with the
   *                    result of the loop-closure detection stage.
   */
  void detectLoop(const FrameId& frame_id,
                  const DBoW2::BowVector& bow_vec,
                  LoopResult* result) override;

 public:
  /* ------------------------------------------------------------------------ */
  /** @brief Returns the RAW pointer to the BoW database.
   * @return A pointer to the BoW database.
   *
   * WARNING: This is a potentially dangerous method to use because it requires
   *  a manual deletion of the pointer before it goes out of scope.
   */
  inline const OrbDatabaseWrapper* getBoWDatabase() const {
    return dynamic_cast<const OrbDatabaseWrapper*>(db_.get());
  }

  /* ------------------------------------------------------------------------ */
  /** @brief Set the OrbDatabase internal member.
   * @param[in] db An OrbDatabase object.
   */
  void setDatabase(const OrbDatabaseWrapper& db);

  /* @brief Set the vocabulary of the BoW detector.
   * @param[in] voc An OrbVocabulary object.
   */
  void setVocabulary(const OrbVocabulary& voc);

  std::map<int, double> globalDescToMap(
      const typename OrbDatabaseWrapper::GlobalDesc& global_desc) override {
    return std::map<int, double>(global_desc.begin(), global_desc.end());
  }

  LCDFrame::Ptr processMonoPnP(const Frame& frame,
                                const PointsWithIdMap& W_points_with_ids,
                                const gtsam::Pose3& W_Pose_Blkf) override {
    // We use existing features instead of new ORB ones like in the stereo
    // case because we have to use existing 3D points from the backend as
    // in Mono mode we cannot compute 3D points via stereo reconstruction.

    // TODO(marcus): check the backend param for generating the
    // LandmarksWithIdMap so that if it's none we
    // can throw exception in lcd ctor
    size_t nr_kpts = frame.keypoints_.size();
    CHECK_EQ(frame.landmarks_.size(), nr_kpts);
    CHECK_EQ(frame.versors_.size(), nr_kpts);
    CHECK_EQ(frame.keypoints_undistorted_.size(), nr_kpts);

    // Re-detect tracker features but with orientation for better
    // descriptors NOTE: see feature/omni/stereo from mubarik
    // TODO(marcus): collapse this with feature/omni/stereo and consider
    // changing KeypointCV to be the full keypoint instead of point2f
    cv::Ptr<cv::GFTTDetector> gftt_feature_detector_ =
        cv::GFTTDetector::create(nr_kpts * 10,  // 2x to increase chance we
                                                // detect our tracked features
                                 0.001,         // quality_level
                                 40,            // min_distance
                                 3,             // block_size
                                 false,         // use_harris_detector
                                 0.04           // k
        );
    std::vector<cv::KeyPoint> keypoints_for_descriptor_compute;
    gftt_feature_detector_->detect(frame.img_,
                                   keypoints_for_descriptor_compute);

    // Compute ORB descriptors for all GFTT keypoints. Many will be
    // culled. We have to compute here because this step will cull
    // keypoints that do not have computable descriptors ahead of time.
    // Descriptors then must be re-computed at the end after culling for
    // other reasons, so unfortunately need two descriptor computes. This
    // is not very compute-friendly but avoids multiple passes of
    // approximate data association.
    // TODO(marcus): figure out if multiple nested for-loops is faster
    // than this.
    OrbDatabaseWrapper::Desc descriptors_mat;
    feature_detector_->compute(
        frame.img_, keypoints_for_descriptor_compute, descriptors_mat);

    // Do data association, with relaxed threshold for what constitutes a
    // match. Also identify which data have 3D points in the backend.
    // Store relevant members for LCDFrame.
    std::vector<cv::KeyPoint> keypoints_for_descriptor_compute_culled;
    std::vector<cv::KeyPoint> keypoints_to_save;
    BearingVectors undistorted_bearing_vectors;
    Landmarks keypoints_3d;

    double threshold = 7;  // px
    for (size_t i = 0; i < nr_kpts; i++) {
      auto& kp = frame.keypoints_[i];

      cv::KeyPoint closestKeypoint;
      double min_dist = std::numeric_limits<double>::max();

      for (auto& kp_detected : keypoints_for_descriptor_compute) {
        double dist = std::fabs(kp_detected.pt.x - kp.x) +
                      std::fabs(kp_detected.pt.y - kp.y);
        if (dist < min_dist) {
          min_dist = dist;
          closestKeypoint = kp_detected;
        }
      }

      // If this keypoint candidate passes the distance check, make sure
      // it has an associated 3D landmark in the backend. If so, store
      // members.
      if (min_dist < threshold) {
        const LandmarkId& lmk_id = frame.landmarks_[i];
        if (W_points_with_ids.find(lmk_id) != W_points_with_ids.end()) {
          // store the cv::KeyPoint version, not frame.keypoints_ because
          // useful for descriptor compute:
          keypoints_for_descriptor_compute_culled.push_back(closestKeypoint);
          keypoints_to_save.push_back(cv::KeyPoint(kp.x, kp.y, 0.0f));
          CHECK_LT(std::abs(frame.versors_[i].norm() - 1.0), 1e-6)
              << "Versor norm: " << frame.versors_[i].norm();
          undistorted_bearing_vectors.push_back(
              UndistorterRectifier::GetBearingVector(kp, frame.cam_param_));
          // Convert point from world frame to local camera frame so that
          // the reference frame matches the convention used in the stereo
          // case.
          Landmark cam_keypoint_3d = (W_Pose_Blkf * B_Pose_Cam_).inverse() *
                                     W_points_with_ids.at(lmk_id);
          keypoints_3d.push_back(cam_keypoint_3d);
        } else {
          VLOG(10) << "ProcessAndAddMonoFrame: landmark id not in world "
                      "points!";
        }
      }
    }

    // Re-generate descriptors for remaining keypoints if necessary
    // (usually is)
    size_t nr_kpts_culled_b4_recompute =
        keypoints_for_descriptor_compute_culled.size();
    if (nr_kpts_culled_b4_recompute < keypoints_for_descriptor_compute.size()) {
      VLOG(10) << "ProcessAndAddMonoFrame: recomputing descriptors.";
      descriptors_mat = OrbDatabaseWrapper::Desc();
      feature_detector_->compute(
          frame.img_, keypoints_for_descriptor_compute_culled, descriptors_mat);
    }
    OrbDatabaseWrapper::DescVector descriptors_vec;
    descriptorMatToVec(descriptors_mat, &descriptors_vec);

    size_t nr_kpts_culled = keypoints_for_descriptor_compute_culled.size();
    CHECK_EQ(keypoints_to_save.size(), nr_kpts_culled);
    CHECK_EQ(keypoints_3d.size(), nr_kpts_culled);
    CHECK_EQ(descriptors_vec.size(), nr_kpts_culled);
    CHECK_EQ(undistorted_bearing_vectors.size(), nr_kpts_culled);
    CHECK_EQ(descriptors_vec.size(), nr_kpts_culled);

    return std::make_shared<LCDFrame>(frame.timestamp_,
                                      FrameCache::NEW_ID,
                                      frame.id_,
                                      keypoints_to_save,
                                      keypoints_3d,
                                      descriptors_vec,
                                      descriptors_mat,
                                      undistorted_bearing_vectors);
  }

 private:
  void getNewFeaturesAndDescriptors(const cv::Mat& img,
                                    std::vector<cv::KeyPoint>* keypoints,
                                    OrbDescriptor* descriptors_mat) override;

  void descriptorMatToVec(const OrbDescriptor& descriptors_mat,
                          OrbDescriptorVec* descriptors_vec) override;

 private:
  // Store latest computed objects for temporal matching and nss scoring
};

}  // namespace VIO
