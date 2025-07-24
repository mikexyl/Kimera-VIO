#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(FeatureTracker);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FeatureTracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureTracker() = default;
  virtual ~FeatureTracker() = default;

  /**
   * @brief trackFeatures Tracks features from frame to frame.
   * @param ref_frame Reference frame with features to track.
   * @param cur_frame Current frame where features are tracked.
   * @return True if tracking was successful, false otherwise.
   */
  virtual void track(Frame* ref_frame,
                     Frame* cur_frame,
                     cv::InputArray prevPts,
                     cv::InputOutputArray nextPts,
                     cv::OutputArray status,
                     cv::OutputArray err,
                     cv::Size winSize = cv::Size(21, 21),
                     int maxLevel = 3,
                     cv::TermCriteria criteria = cv::TermCriteria(
                         cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                         30,
                         0.01),
                     int flags = 0,
                     double minEigThreshold = 1e-4) = 0;

  virtual void trackDesc(Frame* ref_frame,
                         Frame* cur_frame,
                         cv::Mat homography,
                         int search_radius,
                         const std::vector<cv::Point2f>& predictedPts,
                         DMatchVec* matches) {}
};

}  // namespace VIO