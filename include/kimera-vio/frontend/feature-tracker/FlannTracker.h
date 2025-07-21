#pragma once

#include <opencv2/calib3d.hpp>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {

inline std::vector<cv::DMatch> matchWithInitialFlow(
    const cv::Mat& descriptors1,
    const std::vector<cv::Point2f>& keypoints1,
    const cv::Mat& descriptors2,
    const std::vector<cv::Point2f>& keypoints2,
    const std::vector<cv::Point2f>& predictedPts,
    float searchRadius,
    int normType = cv::NORM_L2) {
  CV_Assert(descriptors1.rows == (int)keypoints1.size());
  CV_Assert(descriptors2.rows == (int)keypoints2.size());
  CV_Assert(descriptors1.rows == (int)predictedPts.size());
  CV_Assert(descriptors1.type() == descriptors2.type());

  const int N = descriptors1.rows;
  const int M = descriptors2.rows;
  std::vector<cv::DMatch> matches;

  for (int i = 0; i < N; ++i) {
    const cv::Point2f& pred = predictedPts[i];
    float bestDist = std::numeric_limits<float>::max();
    int bestJ = -1;

    // brute‐force spatial filter + descriptor comparison
    for (int j = 0; j < M; ++j) {
      // spatial check
      if (norm(pred - keypoints2[j]) > searchRadius) continue;

      // descriptor distance
      float d = norm(descriptors1.row(i), descriptors2.row(j), normType);
      if (d < bestDist and d < 0.5) {
        bestDist = d;
        bestJ = j;
      }
    }

    if (bestJ >= 0) {
      matches.emplace_back(cv::DMatch(i, bestJ, bestDist));
    }
  }

  return matches;
}

class FlannTracker : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(FlannTracker);
  KIMERA_DELETE_COPY_CONSTRUCTORS(FlannTracker);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FlannTracker() = default;
  virtual ~FlannTracker() = default;

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
    throw std::runtime_error(
        "FlannMatcher does not support optical flow tracking. "
        "Use LighterGlueCV or OpticalFlowCV instead.");
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 const std::vector<cv::Point2f>& predictedPts,
                 DMatchVec* matches) override {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(matches);

    matches->clear();

    *matches = matchWithInitialFlow(ref_frame->descriptors_,
                                    ref_frame->keypoints_,
                                    cur_frame->descriptors_,
                                    cur_frame->keypoints_,
                                    predictedPts,
                                    15.0f);
  }
};
}  // namespace VIO