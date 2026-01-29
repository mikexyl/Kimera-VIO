#pragma once

#include <glog/logging.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/feature-tracker/FeatureTracker.h"

namespace VIO {

/**
 * @brief OpticalFlowCV implements feature tracking using OpenCV's
 * Lucas-Kanade optical flow with automatic feature detection when
 * tracked points fall below a threshold.
 */
class OpticalFlowCV : public FeatureTracker {
 public:
  KIMERA_POINTER_TYPEDEFS(OpticalFlowCV);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpticalFlowCV);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Params {
    //! Minimum number of tracked features before detecting new ones
    int min_features_threshold = 100;
    //! Maximum number of features to detect
    int max_features = 500;

    //! Lucas-Kanade parameters
    int klt_win_size = 21;
    int klt_max_level = 3;
    int klt_max_iter = 30;
    double klt_eps = 0.01;

    //! Feature detection parameters (goodFeaturesToTrack)
    double quality_level = 0.01;
    double min_distance = 10.0;
    int block_size = 3;
    bool use_harris = false;
    double harris_k = 0.04;

    //! Forward-backward error threshold for robust tracking
    double fwd_bwd_error_threshold = 1.0;

    //! Whether to use forward-backward consistency check
    bool use_fwd_bwd_check = true;

    //! Downsampling parameters
    bool enable_downsample = false;
    double downsample_scale = 0.5;
  };

  OpticalFlowCV() : params_() {}
  explicit OpticalFlowCV(const Params& params) : params_(params) {}
  virtual ~OpticalFlowCV() = default;

  /**
   * @brief Track features from reference frame to current frame using
   *        OpenCV Lucas-Kanade optical flow and detect new features
   *        when tracked count falls below threshold.
   */
  void track(Frame* ref_frame,
             Frame* cur_frame,
             const std::vector<cv::Point2f>& prevPts,
             std::vector<cv::Point2f>* nextPts,
             std::vector<int>* prev_next_matches,
             cv::OutputArray err,
             std::vector<float>* stds,
             std::vector<float>* scores,
             const cv::Mat& mask = cv::Mat()) override {
    CHECK_NOTNULL(cur_frame);
    CHECK_NOTNULL(nextPts);
    CHECK_NOTNULL(prev_next_matches);
    CHECK_NOTNULL(stds);
    CHECK_NOTNULL(scores);

    // Convert current frame to grayscale
    cv::Mat cur_gray;
    if (cur_frame->img_.channels() > 1) {
      cv::cvtColor(cur_frame->img_, cur_gray, cv::COLOR_BGR2GRAY);
    } else {
      cur_gray = cur_frame->img_;
    }

    // Set up termination criteria for optical flow
    cv::TermCriteria term_criteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        params_.klt_max_iter,
        params_.klt_eps);
    cv::Size win_size(params_.klt_win_size, params_.klt_win_size);

    // Track existing points if we have a reference frame and previous points
    std::vector<cv::Point2f> tracked_pts;
    std::vector<int>
        tracked_src_indices;  // Index in prevPts for each tracked point
    std::vector<float> tracked_errors;

    if (ref_frame && !prevPts.empty()) {
      // Convert reference frame to grayscale
      cv::Mat ref_gray;
      if (ref_frame->img_.channels() > 1) {
        cv::cvtColor(ref_frame->img_, ref_gray, cv::COLOR_BGR2GRAY);
      } else {
        ref_gray = ref_frame->img_;
      }

      // Preparation for downsampling
      cv::Mat cur_gray_proc, ref_gray_proc;
      std::vector<cv::Point2f> prevPts_proc;
      float scale = 1.0f;

      if (params_.enable_downsample) {
        scale = static_cast<float>(params_.downsample_scale);
        cv::resize(cur_gray,
                   cur_gray_proc,
                   cv::Size(),
                   scale,
                   scale,
                   cv::INTER_LINEAR);
        cv::resize(ref_gray,
                   ref_gray_proc,
                   cv::Size(),
                   scale,
                   scale,
                   cv::INTER_LINEAR);
        prevPts_proc.resize(prevPts.size());
        for (size_t i = 0; i < prevPts.size(); ++i) {
          prevPts_proc[i] = prevPts[i] * scale;
        }
      } else {
        cur_gray_proc = cur_gray;
        ref_gray_proc = ref_gray;
        prevPts_proc = prevPts;
      }

      // Forward tracking: ref -> cur
      std::vector<cv::Point2f> fwd_pts;
      std::vector<uchar> fwd_status;
      std::vector<float> fwd_err;

      cv::calcOpticalFlowPyrLK(ref_gray_proc,
                               cur_gray_proc,
                               prevPts_proc,
                               fwd_pts,
                               fwd_status,
                               fwd_err,
                               win_size,
                               params_.klt_max_level,
                               term_criteria);

      // Backward tracking for consistency check if enabled
      std::vector<cv::Point2f> bwd_pts;
      std::vector<uchar> bwd_status;
      std::vector<float> bwd_err;

      if (params_.use_fwd_bwd_check) {
        cv::calcOpticalFlowPyrLK(cur_gray_proc,
                                 ref_gray_proc,
                                 fwd_pts,
                                 bwd_pts,
                                 bwd_status,
                                 bwd_err,
                                 win_size,
                                 params_.klt_max_level,
                                 term_criteria);
      }

      // Filter tracked points
      for (size_t i = 0; i < prevPts.size(); ++i) {
        if (!fwd_status[i]) continue;

        // Check bounds
        cv::Point2f pt = fwd_pts[i];
        if (params_.enable_downsample) {
          // Upscale the point back to original resolution
          pt = pt * (1.0f / scale);
        }

        if (pt.x < 0 || pt.y < 0 || pt.x >= cur_gray.cols ||
            pt.y >= cur_gray.rows) {
          continue;
        }

        // Forward-backward consistency check
        if (params_.use_fwd_bwd_check) {
          if (!bwd_status[i]) continue;
          cv::Point2f bwd_pt = bwd_pts[i];
          if (params_.enable_downsample) {
            bwd_pt = bwd_pt * (1.0f / scale);
          }
          float fwd_bwd_err = cv::norm(prevPts[i] - bwd_pt);
          if (fwd_bwd_err > params_.fwd_bwd_error_threshold) continue;
        }

        // Check mask (e.g., sky segmentation)
        if (!mask.empty()) {
          int x = static_cast<int>(pt.x);
          int y = static_cast<int>(pt.y);
          if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows) {
            if (mask.at<uint8_t>(y, x) != 0) {
              continue;  // Masked out
            }
          }
        }

        tracked_pts.push_back(pt);
        tracked_src_indices.push_back(static_cast<int>(i));
        tracked_errors.push_back(fwd_err[i]);
      }
    }

    // Detect new features if tracked count is below threshold
    std::vector<cv::Point2f> new_pts;
    std::vector<float> new_responses;

    int num_to_detect =
        params_.max_features - static_cast<int>(tracked_pts.size());
    bool should_detect =
        static_cast<int>(tracked_pts.size()) < params_.min_features_threshold;

    if (should_detect && num_to_detect > 0) {
      // Create detection mask combining:
      // 1. Input mask (e.g., sky segmentation)
      // 2. Exclusion zones around existing tracked points
      cv::Mat detection_mask;
      if (!mask.empty()) {
        detection_mask = (mask == 0);  // Invert: 0 in mask means valid
      } else {
        detection_mask = cv::Mat(cur_gray.size(), CV_8UC1, cv::Scalar(255));
      }

      // Mark exclusion zones around tracked points
      int exclusion_radius = static_cast<int>(params_.min_distance);
      for (const auto& pt : tracked_pts) {
        cv::circle(detection_mask, pt, exclusion_radius, cv::Scalar(0), -1);
      }

      // Detect new features
      std::vector<cv::Point2f> detected_pts;

      cv::Mat cur_gray_proc;
      cv::Mat detection_mask_proc;
      float scale = 1.0f;

      if (params_.enable_downsample) {
        scale = static_cast<float>(params_.downsample_scale);
        cv::resize(cur_gray,
                   cur_gray_proc,
                   cv::Size(),
                   scale,
                   scale,
                   cv::INTER_LINEAR);
        cv::resize(detection_mask,
                   detection_mask_proc,
                   cv::Size(),
                   scale,
                   scale,
                   cv::INTER_NEAREST);
      } else {
        cur_gray_proc = cur_gray;
        detection_mask_proc = detection_mask;
      }

      cv::goodFeaturesToTrack(cur_gray_proc,
                              detected_pts,
                              num_to_detect,
                              params_.quality_level,
                              params_.min_distance * scale,
                              detection_mask_proc,
                              params_.block_size,
                              params_.use_harris,
                              params_.harris_k);

      // Get responses for detected features using cornerMinEigenVal
      cv::Mat eigen_vals;
      cv::cornerMinEigenVal(cur_gray_proc, eigen_vals, params_.block_size);

      for (const auto& pt : detected_pts) {
        cv::Point2f original_pt = pt;
        if (params_.enable_downsample) {
          original_pt = pt * (1.0f / scale);
        }

        int x = static_cast<int>(pt.x);
        int y = static_cast<int>(pt.y);

        // Check bounds on the PROCESSED image/eigen_vals (which are downsampled
        // if enabled)
        if (x >= 0 && x < eigen_vals.cols && y >= 0 && y < eigen_vals.rows) {
          new_pts.push_back(original_pt);
          new_responses.push_back(eigen_vals.at<float>(y, x));
        }
      }

      VLOG(2) << "OpticalFlowCV: Tracked " << tracked_pts.size()
              << " features, detected " << new_pts.size() << " new features";
    }

    // Populate output: combine tracked and new points
    nextPts->clear();
    prev_next_matches->clear();
    stds->clear();
    scores->clear();

    // First, resize prev_next_matches to match prevPts size (all -1 initially)
    if (!prevPts.empty()) {
      prev_next_matches->assign(prevPts.size(), -1);
    }

    // Add tracked points
    for (size_t i = 0; i < tracked_pts.size(); ++i) {
      int next_idx = static_cast<int>(nextPts->size());
      nextPts->push_back(tracked_pts[i]);
      stds->push_back(8.0f);  // Default std (similar to VilibTracker level 0)

      // Normalize error to [0, 1] score (lower error = higher score)
      float normalized_score = std::max(0.0f, 1.0f - tracked_errors[i] / 30.0f);
      scores->push_back(normalized_score);

      // Update match: prevPts[tracked_src_indices[i]] -> nextPts[next_idx]
      (*prev_next_matches)[tracked_src_indices[i]] = next_idx;
    }

    // Add newly detected points (no match to previous frame)
    for (size_t i = 0; i < new_pts.size(); ++i) {
      nextPts->push_back(new_pts[i]);
      stds->push_back(8.0f);  // Default std

      // Normalize response to [0, 1] score
      float max_response = 0.001f;  // Avoid division by zero
      for (float r : new_responses) {
        max_response = std::max(max_response, r);
      }
      float normalized_score = new_responses[i] / max_response;
      scores->push_back(normalized_score);
    }

    // Copy error to output if requested
    if (err.needed()) {
      cv::Mat error_mat(static_cast<int>(stds->size()), 1, CV_32F);
      for (size_t i = 0; i < stds->size(); ++i) {
        error_mat.at<float>(static_cast<int>(i)) = (*stds)[i];
      }
      error_mat.copyTo(err);
    }

    VLOG(1) << "OpticalFlowCV: Output " << nextPts->size() << " total features";
  }

  static OpticalFlowCV::Ptr Create() {
    return std::make_shared<OpticalFlowCV>();
  }

  static OpticalFlowCV::Ptr Create(const Params& params) {
    return std::make_shared<OpticalFlowCV>(params);
  }

 private:
  Params params_;
};

}  // namespace VIO