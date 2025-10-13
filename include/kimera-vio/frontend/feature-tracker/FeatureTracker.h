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
                     const std::vector<cv::Point2f>& prevPts,
                     std::vector<cv::Point2f>* nextPts,
                     std::vector<int>* status,
                     cv::OutputArray err,
                     std::vector<float>* stds,
                     std::vector<float>* scores) = 0;

  virtual void trackDesc(Frame* ref_frame,
                         Frame* cur_frame,
                         cv::Mat homography,
                         int search_radius,
                         const std::vector<cv::Point2f>& predictedPts,
                         DMatchVec* matches) {}
};

}  // namespace VIO