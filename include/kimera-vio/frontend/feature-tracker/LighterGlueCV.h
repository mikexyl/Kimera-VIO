#pragma once

#include <xfeat-cpp/lighterglue_cv.h>

#include <opencv2/calib3d.hpp>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {
class LighterGlueCV : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(LighterGlueCV);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LighterGlueCV);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Params = xfeat::LighterGlueCV::Params;

  LighterGlueCV(Ort::Env& env, Params params) : lg_matcher_(env, params) {}
  virtual ~LighterGlueCV() = default;

  void track(Frame* ref_frame,
             Frame* cur_frame,
             const std::vector<cv::Point2f>& prevPts,
             std::vector<cv::Point2f>* nextPts,
             std::vector<int>* prev_next_matches,
             cv::OutputArray err,
             std::vector<float>* stds,
             std::vector<float>* scores) override {
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
    det0.scores.create(ref_frame->scores_.size(), 1, CV_32F);
    // copy data from ref_frame->scores_ to det0.scores
    for (size_t i = 0; i < ref_frame->scores_.size(); ++i) {
      det0.scores.at<float>(i) = ref_frame->scores_[i];
    }

    det1.keypoints = cv::Mat(cur_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < cur_frame->keypoints_.size(); ++i) {
      det1.keypoints.at<float>(i, 0) = cur_frame->keypoints_[i].x;
      det1.keypoints.at<float>(i, 1) = cur_frame->keypoints_[i].y;
    }
    det1.descriptors = cur_desc;
    det1.scores.create(cur_frame->scores_.size(), 1, CV_32F);
    for (size_t i = 0; i < cur_frame->scores_.size(); ++i) {
      det1.scores.at<float>(i) = cur_frame->scores_[i];
    }

    cv::Size image_size0(640, 480);  // Default size, can be changed
    cv::Size image_size1(640, 480);  // Default size, can be changed

    lg_matcher_.match(det0, image_size0, det1, image_size1, matches);

    VLOG(1) << "found " << matches.size() << " matches, time gap: "
            << (cur_frame->timestamp_ - ref_frame->timestamp_) / 1e6 << " ms";

    prev_next_matches->resize(ref_frame->keypoints_.size(),
                              -1);  // Initialize to -1
    // set status
    for (auto match : matches) {
      (*prev_next_matches)[match.queryIdx] = match.trainIdx;  // Mark as found
      VLOG(1) << "Match found: " << match.queryIdx << " -> " << match.trainIdx;
    }

    std::vector<float>& err_vec =
        *reinterpret_cast<std::vector<float>*>(err.getObj());
    err_vec.resize(ref_frame->keypoints_.size(), 0.0f);
  }

  static FeatureTracker::Ptr Create(Ort::Env& env, Params params) {
    return std::make_shared<LighterGlueCV>(env, params);
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 cv::Mat homography,
                 int search_radius,
                 const std::vector<cv::Point2f>& predictedPts,
                 DMatchVec* matches) override {
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(matches);

    CHECK(not ref_frame->keypoints_undistorted_.empty());
    CHECK(not cur_frame->keypoints_undistorted_.empty());
    CHECK_EQ(ref_frame->keypoints_undistorted_.size(),
             ref_frame->keypoints_.size());
    CHECK_EQ(cur_frame->keypoints_undistorted_.size(),
             cur_frame->keypoints_.size());

    xfeat::DetectionResult det0, det1;
    // keypoints vec to mat
    det0.keypoints = cv::Mat(ref_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < ref_frame->keypoints_.size(); ++i) {
      det0.keypoints.at<float>(i, 0) = ref_frame->keypoints_[i].x;
      det0.keypoints.at<float>(i, 1) = ref_frame->keypoints_[i].y;
    }
    det0.descriptors = ref_frame->descriptors_;
    det0.scores.create(ref_frame->scores_.size(), 1, CV_32F);
    // copy data from ref_frame->scores_ to det0.scores
    for (size_t i = 0; i < ref_frame->scores_.size(); ++i) {
      det0.scores.at<float>(i) = ref_frame->scores_[i];
    }

    det1.keypoints = cv::Mat(cur_frame->keypoints_.size(), 2, CV_32F);
    for (size_t i = 0; i < cur_frame->keypoints_.size(); ++i) {
      det1.keypoints.at<float>(i, 0) = cur_frame->keypoints_[i].x;
      det1.keypoints.at<float>(i, 1) = cur_frame->keypoints_[i].y;
    }
    det1.descriptors = cur_frame->descriptors_;
    det1.scores.create(cur_frame->scores_.size(), 1, CV_32F);
    // copy data from ref_frame->scores_ to det0.scores
    for (size_t i = 0; i < cur_frame->scores_.size(); ++i) {
      det1.scores.at<float>(i) = cur_frame->scores_[i];
    }

    cv::Size image_size0 = ref_frame->img_.size();  // Use actual image size
    cv::Size image_size1 = cur_frame->img_.size();  // Use actual image size

    lg_matcher_.match(det0, image_size0, det1, image_size1, *matches);

    // filter matches by finding homography
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& match : *matches) {
      pts1.push_back({det0.keypoints.at<float>(match.queryIdx, 0),
                      det0.keypoints.at<float>(match.queryIdx, 1)});
      pts2.push_back({det1.keypoints.at<float>(match.trainIdx, 0),
                      det1.keypoints.at<float>(match.trainIdx, 1)});
    }
    if (pts1.size() < 4) {
      matches->clear();
      return;
    }
    cv::Mat mask;
    cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC, 3.5, mask, 100, 0.9);
    mask = mask.reshape(1, mask.total());
    DMatchVec filtered_matches;
    for (size_t i = 0; i < matches->size(); ++i) {
      if (mask.at<uchar>(i, 0) > 0) {
        filtered_matches.push_back((*matches)[i]);
      }
    }
    matches->swap(filtered_matches);

    VLOG(1) << "found " << matches->size() << " matches, time gap: "
            << (cur_frame->timestamp_ - ref_frame->timestamp_) / 1e6 << " ms";
  }

 private:
  xfeat::LighterGlueCV lg_matcher_;
};
}  // namespace VIO