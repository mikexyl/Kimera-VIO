#pragma once

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {
class OpticalFlowCV : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(OpticalFlowCV);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpticalFlowCV);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OpticalFlowCV() = default;
  virtual ~OpticalFlowCV() = default;

  void track(Frame* ref_frame,
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
             double minEigThreshold = 1e-4) override {
    cv::calcOpticalFlowPyrLK(ref_frame->img_,
                             cur_frame->img_,
                             prevPts,
                             nextPts,
                             status,
                             err,
                             winSize,
                             maxLevel,
                             criteria,
                             flags,
                             minEigThreshold);
  }

  static OpticalFlowCV::Ptr Create() {
    return std::make_shared<OpticalFlowCV>();
  }
};
}  // namespace VIO