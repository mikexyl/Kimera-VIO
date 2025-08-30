/**
 * @file   FeatureDetector.cpp
 * @brief  Base class for feature detector interface
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"

#include <xfeat-cpp/xfeat_cv.h>

#include <algorithm>
#include <numeric>

#include "kimera-vio/frontend/UndistorterRectifier.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsOpenCV.h"  // Just for ExtractCorners...

namespace VIO {

LandmarkId FeatureDetector::lmk_id = 0;

FeatureDetector::FeatureDetector(
    const FeatureDetectorParams& feature_detector_params,
    std::shared_ptr<Ort::Env> env)
    : feature_detector_params_(feature_detector_params),
      non_max_suppression_(nullptr),
      feature_detector_() {
  // TODO(Toni): parametrize as well whether we use bucketing or anms...
  // Right now we assume we want anms not bucketing...
  if (feature_detector_params.enable_non_max_suppression_) {
    non_max_suppression_ = std::make_unique<AdaptiveNonMaximumSuppression>(
        feature_detector_params.non_max_suppression_type_);
  }
  // We always try to extract max_nr_keypoints_before_anms_ keypoints and then
  // pass to nonmax suppression that prunes them to
  // feature_detector_params_.max_features_per_frame_

  // TODO(Toni): find a way to pass params here using args lists
  switch (feature_detector_params.feature_detector_type_) {
    case FeatureDetectorType::FAST: {
      // Fast threshold, usually in range [10, 35]
      feature_detector_ = cv::FastFeatureDetector::create(
          feature_detector_params.fast_thresh_, true);
      break;
    }
    case FeatureDetectorType::ORB: {
      static constexpr float scale_factor = 1.2f;
      static constexpr int n_levels = 8;
      static constexpr int edge_threshold =
          10;  // Very small bcs we don't use descriptors (yet).
      static constexpr int first_level = 0;
      static constexpr int WTA_K = 0;  // We don't use descriptors (yet).
#if CV_VERSION_MAJOR == 3
      static constexpr int score_type = cv::ORB::HARRIS_SCORE;
#else
      static constexpr cv::ORB::ScoreType score_type =
          cv::ORB::ScoreType::HARRIS_SCORE;
#endif
      static constexpr int patch_size = 2;  // We don't use descriptors (yet).
      feature_detector_ =
          cv::ORB::create(feature_detector_params.max_nr_keypoints_before_anms_,
                          scale_factor,
                          n_levels,
                          edge_threshold,
                          first_level,
                          WTA_K,
                          score_type,
                          patch_size,
                          feature_detector_params.fast_thresh_);
      break;
    }
    case FeatureDetectorType::AGAST: {
      LOG(FATAL) << "AGAST feature detector not implemented.";
      break;
    }
    case FeatureDetectorType::GFTT: {
      // goodFeaturesToTrack detector.
      feature_detector_ = cv::GFTTDetector::create(
          feature_detector_params.max_nr_keypoints_before_anms_,
          feature_detector_params_.quality_level_,
          feature_detector_params_
              .min_distance_btw_tracked_and_detected_features_,
          feature_detector_params_.block_size_,
          feature_detector_params_.use_harris_corner_detector_,
          feature_detector_params_.k_);
      break;
    }
    case FeatureDetectorType::XFEAT: {
      xfeat::XFeatCV::Params xfeat_params;
      xfeat_params.max_features =
          feature_detector_params_.max_features_per_frame_;
      xfeat_params.xfeat_path = feature_detector_params_.xfeat_path_;
      xfeat_params.interp_bicubic_path =
          feature_detector_params_.interp_bicubic_path_;
      xfeat_params.interp_bilinear_path =
          feature_detector_params_.interp_bilinear_path_;
      xfeat_params.interp_nearest_path =
          feature_detector_params_.interp_nearest_path_;
      xfeat_params.use_gpu = feature_detector_params_.xfeat_use_gpu_;
      xfeat_params.anms = feature_detector_params_.enable_non_max_suppression_;
      xfeat_params.nkpts_before_anms =
          feature_detector_params_.max_nr_keypoints_before_anms_;
      xfeat_params.keypoint_detection = 0;  // Use xfeat to detect keypoints

      auto xfeat = xfeat::XFeatCV::create(*env, xfeat_params);
      feature_detector_ = xfeat;
      break;
    }
    case FeatureDetectorType::GFTT_XFEAT: {
      xfeat::XFeatCV::Params xfeat_params;
      xfeat_params.max_features =
          feature_detector_params_.max_features_per_frame_;
      xfeat_params.xfeat_path = feature_detector_params_.xfeat_path_;
      xfeat_params.interp_bicubic_path =
          feature_detector_params_.interp_bicubic_path_;
      xfeat_params.interp_bilinear_path =
          feature_detector_params_.interp_bilinear_path_;
      xfeat_params.interp_nearest_path =
          feature_detector_params_.interp_nearest_path_;
      xfeat_params.use_gpu = feature_detector_params_.xfeat_use_gpu_;
      xfeat_params.anms = feature_detector_params_.enable_non_max_suppression_;
      xfeat_params.nkpts_before_anms =
          feature_detector_params_.max_nr_keypoints_before_anms_;
      xfeat_params.keypoint_detection = 1;  // Use GFTT to detect keypoints

      auto xfeat = xfeat::XFeatCV::create(*env, xfeat_params);
      feature_detector_ = xfeat;
      break;
    }
    default: {
      LOG(FATAL) << "Unknown feature detector type: "
                 << VIO::to_underlying(
                        feature_detector_params.feature_detector_type_);
    }
  }
}

// TODO(Toni) Optimize this function.
// NOTE: for stereo cameras we pass R to ensure we rectify the versors
// and 3D points of the features we detect.
void FeatureDetector::featureDetectionTracked(Frame* cur_frame,
                                              Frame* ref_frame,
                                              std::optional<cv::Mat> R) {
  CHECK_NOTNULL(cur_frame);

  // Check how many new features we need: maxFeaturesPerFrame_ - n_existing
  // features If ref_frame has zero features this simply detects
  // maxFeaturesPerFrame_ new features for cur_frame
  int n_existing = 0;  // count existing (tracked) features
  for (size_t i = 0u; i < cur_frame->landmarks_.size(); ++i) {
    // count nr of valid keypoints
    if (cur_frame->landmarks_[i] != -1) ++n_existing;
    // features that have been tracked so far have Age+1
    cur_frame->landmarks_age_.at(i)++;
    // Note: this is done here (rather than the tracker) since the detection is
    // done at keyframes and landmarks_age_ counts the nr of keyframes a
    // keypoint is observed in
  }

  // Detect new features in image.
  // detect this much new corners if possible
  int nr_corners_needed = std::max(
      feature_detector_params_.max_features_per_frame_ - n_existing, 0);
  // debug_info_.need_n_corners_ = nr_corners_needed;

  ///////////////// FEATURE DETECTION //////////////////////
  // Actual feature detection: detects new keypoints where there are no
  // currently tracked ones
  // auto start_time_tic = utils::Timer::tic();
  const KeypointsCV& corners =
      featureDetection(cur_frame, ref_frame, nr_corners_needed, nullptr);
  const size_t& n_corners = corners.size();

  // debug_info_.featureDetectionTime_ =
  // utils::Timer::toc(start_time_tic).count(); debug_info_.extracted_corners_ =
  // n_corners;

  if (n_corners > 0u) {
    ///////////////// STORE NEW KEYPOINTS  //////////////////////
    // Store features in our Frame
    const size_t& prev_nr_keypoints = cur_frame->keypoints_.size();
    const size_t& new_nr_keypoints = prev_nr_keypoints + n_corners;
    cur_frame->landmarks_.reserve(new_nr_keypoints);
    cur_frame->landmarks_age_.reserve(new_nr_keypoints);
    cur_frame->keypoints_.reserve(new_nr_keypoints);
    cur_frame->scores_.reserve(new_nr_keypoints);
    cur_frame->versors_.reserve(new_nr_keypoints);

    // Incremental id assigned to new landmarks
    const CameraParams& cam_param = cur_frame->cam_param_;
    for (const KeypointCV& corner : corners) {
      cur_frame->landmarks_.push_back(lmk_id);
      // New keypoint, so seen in a single (key)frame so far.
      cur_frame->landmarks_age_.push_back(1u);
      cur_frame->keypoints_.push_back(corner);
      cur_frame->scores_.push_back(0.0);  // NOT IMPLEMENTED
      cur_frame->versors_.push_back(
          UndistorterRectifier::GetBearingVector(corner, cam_param, R));
      ++lmk_id;
    }
    VLOG(10) << "featureExtraction: frame " << cur_frame->id_
             << ",  Nr tracked keypoints: " << prev_nr_keypoints
             << ",  Nr extracted keypoints: " << n_corners
             << ",  total: " << cur_frame->keypoints_.size()
             << "  (max: " << feature_detector_params_.max_features_per_frame_
             << ")";
  } else {
    LOG(WARNING) << "No corners extracted for frame with id: "
                 << cur_frame->id_;
  }
}

std::vector<cv::KeyPoint> FeatureDetector::rawFeatureDetection(
    const cv::Mat& img,
    const cv::Mat& mask) {
  std::vector<cv::KeyPoint> keypoints;
  CHECK(feature_detector_);
  feature_detector_->detect(img, keypoints, mask);
  return keypoints;
}

KeypointsCV FeatureDetector::featureDetection(Frame* cur_frame,
                                              Frame* ref_frame,
                                              const int& need_n_corners,
                                              std::vector<int>* tracked_kp_id) {
  // cv::namedWindow("Input Image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Input Image", cur_frame.img_);

  // TODO(TONI): an alternative approach is to find all features,
  // do max-suppression, and then remove those detections that are close to
  // the already found ones, even you could cut feature tracks that are no
  // longer good quality or visible early on if they don't have detected
  // keypoints nearby by! The mask is interpreted as: 255 -> consider, 0 ->
  // don't consider.
  // Actual raw feature detection
  CHECK_NOTNULL(cur_frame);
  CHECK_NOTNULL(tracked_kp_id);

  if (ref_frame) {
    CHECK_NE(cur_frame->id_, ref_frame->id_);
  }
  std::vector<cv::KeyPoint> keypoints;

  cv::Mat mask;
  if (cur_frame->detection_mask_.empty()) {
    mask = cv::Mat(cur_frame->img_.size(), CV_8U, cv::Scalar(255));
  } else {
    mask = cur_frame->detection_mask_;
  }

  std::set<LandmarkId> existing_lmks;
  if (ref_frame) {
    existing_lmks.insert(ref_frame->landmarks_.begin(),
                         ref_frame->landmarks_.end());
    VLOG(1) << "prev lmk id range: "
            << *std::minmax_element(ref_frame->landmarks_.begin(),
                                    ref_frame->landmarks_.end())
                    .first
            << " - "
            << *std::minmax_element(ref_frame->landmarks_.begin(),
                                    ref_frame->landmarks_.end())
                    .second;
  }

  for (size_t i = 0u; i < cur_frame->keypoints_.size(); ++i) {
    if (cur_frame->landmarks_.at(i) != -1 and
        existing_lmks.count(cur_frame->landmarks_.at(i)) > 0) {
      tracked_kp_id->push_back(i);

      // Only mask keypoints that are being triangulated (I guess
      // feature tracks? should be made more explicit)
      cv::circle(mask,
                 cur_frame->keypoints_.at(i),
                 feature_detector_params_
                     .min_distance_btw_tracked_and_detected_features_,
                 cv::Scalar(0),
                 CV_FILLED);
    }
  }
  if (feature_detector_params_.feature_detector_type_ >=
      FeatureDetectorType::XFEAT) {
    // when using xfeat, we detect features and compute descriptors
    auto xfeat_detector =
        std::dynamic_pointer_cast<xfeat::XFeatCV>(feature_detector_);
    CHECK_NOTNULL(xfeat_detector);
    std::vector<cv::Vec2d> keypoint_stds;

    for (size_t i = 0; i < cur_frame->keypoints_.size(); i++) {
      keypoints.emplace_back(cv::KeyPoint(cur_frame->keypoints_.at(i), 1.0));
    }

    if (not keypoints.empty()) {
      VLOG(1) << "cur lmk id range: "
              << *std::minmax_element(cur_frame->landmarks_.begin(),
                                      cur_frame->landmarks_.end())
                      .first
              << " - "
              << *std::minmax_element(cur_frame->landmarks_.begin(),
                                      cur_frame->landmarks_.end())
                      .second;
    }

    auto input_kpts = keypoints;
    if (input_kpts.size()) {
      VLOG(1) << input_kpts[0].pt;
    }

    std::vector<double> xfeat_scores;
    xfeat_detector->detectAndCompute(cur_frame->img_,
                                     {},
                                     keypoints,
                                     cur_frame->descriptors_,
                                     true,
                                     &cur_frame->xfeat_M1_,
                                     &cur_frame->xfeat_x_prep_,
                                     &keypoint_stds,
                                     //  nullptr,
                                     &xfeat_scores);
    // check the first elements of the input_kpts the same as keypoints
    CHECK_LE(input_kpts.size(), keypoints.size());
    CHECK_EQ(cur_frame->keypoints_.size(), cur_frame->scores_.size());
    if (input_kpts.size()) {  // make sure the xfeat doesn't not change the
                              // input keypoints
      CHECK_LE(cv::norm(input_kpts[0].pt - keypoints[0].pt), 1e-1);
    }

    CHECK_EQ(xfeat_scores.size(), keypoints.size());

    cur_frame->secd_scores_.resize(xfeat_scores.size(), 0.0);
    for (size_t i = 0; i < xfeat_scores.size(); ++i) {
      cur_frame->secd_scores_.at(i) =
          xfeat_scores.at(i) * cur_frame->scores_.at(i);
    }

    VLOG(1) << "finish xfeat detection " << keypoints.size();
  } else {
    keypoints = rawFeatureDetection(cur_frame->img_, mask);
    VLOG(1) << "Need n corners: " << need_n_corners;
    // Tolerance of the number of returned points in percentage.
    if (non_max_suppression_) {
      static constexpr float tolerance = 0.1;
      keypoints = non_max_suppression_->suppressNonMax(
          keypoints,
          need_n_corners,
          tolerance,
          cur_frame->img_.cols,
          cur_frame->img_.rows,
          feature_detector_params_.nr_horizontal_bins_,
          feature_detector_params_.nr_vertical_bins_,
          feature_detector_params_.binning_mask_);
    }
  }

  // TODO(Toni): we should be using cv::KeyPoint... not cv::Point2f...
  KeypointsCV new_corners;
  cv::KeyPoint::convert(keypoints, new_corners);

  // TODO(Toni) this takes a ton of time 27ms each time...
  // Change window_size, and term_criteria to improve timing
  if (new_corners.size() > 0) {
    if (feature_detector_params_.enable_subpixel_corner_refinement_) {
      LOG(FATAL) << "subpixel corner refinement disabled";
    }
  }

  return new_corners;
}

void FeatureDetector::featureDetection(Frame* cur_frame,
                                       std::optional<cv::Mat> R,
                                       Frame* ref_frame) {
  bool use_tracked_features = feature_detector_params_.feature_detector_type_ <
                              FeatureDetectorType::XFEAT;
  // if we use XFEAT we always detect all new features and then match them
  if (use_tracked_features) {
    featureDetectionTracked(cur_frame, ref_frame, R);
  } else {
    // If we don't use tracked features, we just detect new features
    featureDetectionNew(cur_frame, ref_frame, R);
  }
}

void FeatureDetector::featureDetectionNew(Frame* cur_frame,
                                          Frame* ref_frame,
                                          std::optional<cv::Mat> R) {
  CHECK_NOTNULL(cur_frame);

  int nr_corners_needed = feature_detector_params_.max_features_per_frame_;

  std::vector<int> tracked_corners;

  CHECK_GT(cur_frame->keypoints_.size(), 0) << cur_frame->id_;

  auto corners = featureDetection(
      cur_frame, ref_frame, nr_corners_needed, &tracked_corners);
  VLOG(1) << "tracked corners: " << tracked_corners.size();

  CHECK_EQ(cur_frame->keypoints_.size(), cur_frame->descriptors_.rows);
  CHECK_EQ(cur_frame->keypoints_.size(), cur_frame->scores_.size());

  // here we only detect and compute keypoints and descriptors
  // matching is done afterwards by the lighterglue matcher
}

}  // namespace VIO
