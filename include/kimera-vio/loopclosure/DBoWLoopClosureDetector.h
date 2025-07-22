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
