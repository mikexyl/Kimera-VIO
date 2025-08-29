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
             const std::vector<cv::Point2f>& prevPts,
             std::vector<cv::Point2f>* nextPts,
             std::vector<int>* prev_next_matches,
             cv::OutputArray err,
             cv::Size winSize = cv::Size(21, 21),
             int maxLevel = 3,
             cv::TermCriteria criteria = cv::TermCriteria(
                 cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                 30,
                 0.01),
             int flags = 0,
             double minEigThreshold = 1e-4) override {
    std::vector<uchar> status_vec;
    cv::calcOpticalFlowPyrLK(ref_frame->img_,
                             cur_frame->img_,
                             prevPts,
                             *nextPts,
                             status_vec,
                             err,
                             winSize,
                             maxLevel,
                             criteria,
                             flags,
                             minEigThreshold);
    prev_next_matches->clear();
    for (size_t i = 0; i < status_vec.size(); ++i) {
      if (status_vec[i]) {
        (*prev_next_matches)[i] = i;
      }
    }
  }

  static OpticalFlowCV::Ptr Create() {
    return std::make_shared<OpticalFlowCV>();
  }
};
}  // namespace VIO