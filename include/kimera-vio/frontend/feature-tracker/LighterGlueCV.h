#pragma once

#include <xfeat-cpp/lighterglue_cv.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTrakcer.h"

namespace VIO {
class LighterGlueCV : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(LighterGlueCV);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LighterGlueCV);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Params = xfeat::LighterGlueCV::Params;

  LighterGlueCV(Params params) : lg_matcher_(params) {}
  virtual ~LighterGlueCV() = default;

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
    // number of previous points should be equal to number of descriptors
    CHECK_EQ(ref_frame->keypoints_.size(), ref_frame->descriptors_.rows)
        << "Number of previous points does not match number of descriptors in "
           "reference frame!";

    CHECK_EQ(ref_frame->keypoints_.size(), lg_matcher_.params_.n_kpts)
        << "Number of previous points does not match expected n_kpts!";

    auto ref_desc = ref_frame->descriptors_;
    auto cur_desc = cur_frame->descriptors_;

    CHECK(not ref_frame->keypoints_.empty());
    CHECK(not cur_frame->keypoints_.empty());
    CHECK(!ref_desc.empty()) << "Reference frame descriptors are empty!";
    CHECK(!cur_desc.empty()) << "Current frame descriptors are empty!";

    std::vector<cv::DMatch> matches;
    xfeat::DetectionResult det0, det1;
    // keypoints vec to mat
    det0.keypoints = cv::Mat(ref_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < ref_frame->keypoints_.size(); ++i) {
      det0.keypoints.at<float>(i, 0) = ref_frame->keypoints_[i].x;
      det0.keypoints.at<float>(i, 1) = ref_frame->keypoints_[i].y;
    }
    det0.descriptors = ref_desc;

    det1.keypoints = cv::Mat(cur_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < cur_frame->keypoints_.size(); ++i) {
      det1.keypoints.at<float>(i, 0) = cur_frame->keypoints_[i].x;
      det1.keypoints.at<float>(i, 1) = cur_frame->keypoints_[i].y;
    }
    det1.descriptors = cur_desc;

    cv::Size image_size0(640, 352);  // Default size, can be changed
    cv::Size image_size1(640, 352);  // Default size, can be changed

    lg_matcher_.match(det0, image_size0, det1, image_size1, matches);

    std::vector<uchar>& status_vec =
        *reinterpret_cast<std::vector<uchar>*>(status.getObj());
    status_vec.resize(ref_frame->keypoints_.size(), 0);  // Initialize to 0
    // set status
    for (auto match : matches) {
      status_vec[match.queryIdx] = 1;  // Mark as found
    }

    std::vector<float>& err_vec =
        *reinterpret_cast<std::vector<float>*>(err.getObj());
    err_vec.resize(ref_frame->keypoints_.size(), 0.0f);
  }

  static FeatureTracker::Ptr Create(Params params) {
    return std::make_shared<LighterGlueCV>(params);
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 DMatchVec* matches) override {
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

    cv::Size image_size0 = ref_frame->img_.size();  // Use actual image size
    cv::Size image_size1 = cur_frame->img_.size();  // Use actual image size

    lg_matcher_.match(det0, image_size0, det1, image_size1, *matches);
  }

 private:
  xfeat::LighterGlueCV lg_matcher_;
};
}  // namespace VIO