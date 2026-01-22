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
             std::vector<float>* stds,
             std::vector<float>* scores,
             const cv::Mat& mask = cv::Mat()) override {
    LOG(FATAL) << "OpticalFlowCV::track not implemented";
  }

  static OpticalFlowCV::Ptr Create() {
    return std::make_shared<OpticalFlowCV>();
  }
};
}  // namespace VIO