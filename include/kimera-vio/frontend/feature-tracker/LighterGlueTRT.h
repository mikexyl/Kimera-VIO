#pragma once

#include <xfeat-cpp/lighterglue_trt.h>

#include <array>
#include <opencv2/calib3d.hpp>
#include <string>
#include <vector>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {

class LighterGlueTRT : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(LighterGlueTRT);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LighterGlueTRT);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Params {
    std::string model_path;
    float min_score = -1.0f;
    int n_kpts = 500;
    bool verbose = false;
  };

  explicit LighterGlueTRT(const Params& params)
      : matcher_(params.model_path, params.verbose),
        min_score_(params.min_score),
        max_keypoints_(params.n_kpts) {
    CHECK(!params.model_path.empty());
    CHECK_GT(max_keypoints_, 0);
    LOG(INFO) << "Using native TensorRT LighterGlue tracker: "
              << params.model_path;
  }

  ~LighterGlueTRT() override = default;

  void track(Frame* ref_frame,
             Frame* cur_frame,
             const std::vector<cv::Point2f>& prev_pts,
             std::vector<cv::Point2f>* next_pts,
             std::vector<int>* prev_next_matches,
             cv::OutputArray err,
             std::vector<float>* stds,
             std::vector<float>* scores,
             const cv::Mat& mask) override {
    (void)prev_pts;
    (void)next_pts;
    (void)stds;
    (void)scores;
    (void)mask;
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(prev_next_matches);

    const DMatchVec matches = matchFrames(*ref_frame, *cur_frame);
    prev_next_matches->assign(ref_frame->keypoints_.size(), -1);
    for (const auto& match : matches) {
      (*prev_next_matches)[match.queryIdx] = match.trainIdx;
    }

    auto& err_vec = *reinterpret_cast<std::vector<float>*>(err.getObj());
    err_vec.assign(ref_frame->keypoints_.size(), 0.0f);
  }

  void trackDesc(Frame* ref_frame,
                 Frame* cur_frame,
                 cv::Mat homography,
                 int search_radius,
                 const std::vector<cv::Point2f>& predicted_pts,
                 DMatchVec* matches) override {
    (void)homography;
    (void)search_radius;
    (void)predicted_pts;
    CHECK_NOTNULL(ref_frame);
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(matches);

    *matches = matchFrames(*ref_frame, *cur_frame);
    if (matches->size() < 4) {
      matches->clear();
      return;
    }

    std::vector<cv::Point2f> reference_points;
    std::vector<cv::Point2f> current_points;
    reference_points.reserve(matches->size());
    current_points.reserve(matches->size());
    for (const auto& match : *matches) {
      reference_points.push_back(ref_frame->keypoints_.at(match.queryIdx));
      current_points.push_back(cur_frame->keypoints_.at(match.trainIdx));
    }

    cv::Mat inlier_mask;
    cv::findHomography(reference_points,
                       current_points,
                       cv::RANSAC,
                       3.5,
                       inlier_mask,
                       100,
                       0.9);
    if (inlier_mask.empty()) {
      matches->clear();
      return;
    }
    inlier_mask = inlier_mask.reshape(1, inlier_mask.total());
    DMatchVec filtered_matches;
    filtered_matches.reserve(matches->size());
    for (size_t index = 0; index < matches->size(); ++index) {
      if (inlier_mask.at<uchar>(index, 0) > 0) {
        filtered_matches.push_back(matches->at(index));
      }
    }
    matches->swap(filtered_matches);
  }

 private:
  static xfeat::DetectionResult detectionResult(const Frame& frame) {
    CHECK(!frame.keypoints_.empty());
    CHECK(!frame.descriptors_.empty());
    CHECK_EQ(frame.keypoints_.size(), frame.descriptors_.rows);
    CHECK_EQ(frame.descriptors_.cols, 64);

    xfeat::DetectionResult result;
    result.keypoints = cv::Mat(frame.keypoints_.size(), 2, CV_32F);
    for (size_t index = 0; index < frame.keypoints_.size(); ++index) {
      result.keypoints.at<float>(index, 0) = frame.keypoints_.at(index).x;
      result.keypoints.at<float>(index, 1) = frame.keypoints_.at(index).y;
    }
    result.descriptors = frame.descriptors_;
    return result;
  }

  DMatchVec matchFrames(const Frame& reference, const Frame& current) {
    xfeat::DetectionResult reference_features = detectionResult(reference);
    xfeat::DetectionResult current_features = detectionResult(current);
    CHECK_LE(reference_features.keypoints.rows, max_keypoints_)
        << "Reference features exceed the LighterGlue TensorRT profile";
    CHECK_LE(current_features.keypoints.rows, max_keypoints_)
        << "Current features exceed the LighterGlue TensorRT profile";

    const std::array<float, 2> reference_size = {
        static_cast<float>(reference.img_.cols),
        static_cast<float>(reference.img_.rows)};
    const std::array<float, 2> current_size = {
        static_cast<float>(current.img_.cols),
        static_cast<float>(current.img_.rows)};
    std::vector<float> match_scores;
    const auto match_indices = matcher_.match(reference_features,
                                              reference_size,
                                              current_features,
                                              current_size,
                                              min_score_,
                                              &match_scores);

    DMatchVec matches;
    for (size_t query_index = 0; query_index < match_indices.size();
         ++query_index) {
      for (const int train_index : match_indices.at(query_index)) {
        const float score = match_scores.at(query_index);
        matches.emplace_back(static_cast<int>(query_index),
                             train_index,
                             0,
                             score);
      }
    }
    VLOG(1) << "TensorRT LighterGlue found " << matches.size()
            << " matches";
    return matches;
  }

  xfeat::LighterGlueTRT matcher_;
  float min_score_;
  int max_keypoints_;
};

}  // namespace VIO
