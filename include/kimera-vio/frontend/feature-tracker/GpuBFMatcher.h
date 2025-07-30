#pragma once

#include <xfeat-cpp/gpu_matcher.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {
class GpuBFMatcher : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(GpuBFMatcher);
  KIMERA_DELETE_COPY_CONSTRUCTORS(GpuBFMatcher);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GpuBFMatcher(int nkpts, float min_sim = 0.4) : min_sim_(min_sim) {
    gpu_matcher_ = std::make_unique<xfeat::CuMatcher>();
    gpu_matcher_->init(nkpts, nkpts, 64);  // Assuming 64 is the descriptor size
    VLOG(1) << "GpuBFMatcher initialized with max keypoints: " << nkpts;
  }
  virtual ~GpuBFMatcher() = default;

  void track(Frame* ref_frame,
             Frame* cur_frame,
             cv::InputArray /*prevPts*/,
             cv::InputOutputArray /*lnextPts*/,
             cv::OutputArray status,
             cv::OutputArray err,
             cv::Size /*winSize*/ = cv::Size(21, 21),
             int /*maxLevel*/ = 3,
             cv::TermCriteria /*criteria*/ = cv::TermCriteria(
                 cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                 30,
                 0.01),
             int /*flags*/ = 0,
             double /*minEigThreshold*/ = 1e-4) override {
    throw std::runtime_error(
        "GpuBFMatcher does not support optical flow tracking. ");
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 cv::Mat homography,
                 int search_radius,
                 const std::vector<cv::Point2f>& predictedPts,
                 DMatchVec* matches) override {
    CHECK_NOTNULL(gpu_matcher_);
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(matches);

    CHECK(not ref_frame->keypoints_.empty());
    CHECK(not cur_frame->keypoints_.empty());

    xfeat::DetectionResult det0, det1;
    // keypoints vec to mat
    det0.keypoints = cv::Mat(ref_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < ref_frame->keypoints_.size(); ++i) {
      det0.keypoints.at<float>(i, 0) = ref_frame->keypoints_[i].x;
      det0.keypoints.at<float>(i, 1) = ref_frame->keypoints_[i].y;
    }
    det0.descriptors = ref_frame->descriptors_;

    det1.keypoints = cv::Mat(cur_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < cur_frame->keypoints_.size(); ++i) {
      det1.keypoints.at<float>(i, 0) = cur_frame->keypoints_[i].x;
      det1.keypoints.at<float>(i, 1) = cur_frame->keypoints_[i].y;
    }
    det1.descriptors = cur_frame->descriptors_;

    std::vector<cv::DMatch> matches_vec;
    matches_vec =
        gpu_matcher_->match(det0, det1, min_sim_, homography, search_radius);

    // copy matches to DMatchVec
    matches->clear();
    matches->reserve(matches_vec.size());
    for (const auto& match : matches_vec) {
      matches->emplace_back(match.queryIdx, match.trainIdx, match.distance);
    }

    VLOG(1) << "found " << matches->size() << " matches, time gap: "
            << (cur_frame->timestamp_ - ref_frame->timestamp_) / 1e6 << " ms";
  }

 private:
  std::unique_ptr<xfeat::CuMatcher> gpu_matcher_;
  float min_sim_ = 0.4;  // Minimum similarity threshold for matching
};
}  // namespace VIO