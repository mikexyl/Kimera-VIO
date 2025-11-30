/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoMatcher.cpp
 * @brief  Class describing stereo matching algorithms.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/StereoMatcher.h"

#include <cuda_runtime.h>
#include <glog/logging.h>
#include <libsgm.h>

#include <opencv2/calib3d.hpp>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/utils/Macros.h"

namespace sgm {
class StereoSGM;
}
namespace VIO {

StereoMatcher::StereoMatcher(const StereoCamera::ConstPtr& stereo_camera,
                             const StereoMatchingParams& stereo_matching_params)
    : stereo_camera_(stereo_camera),
      stereo_matching_params_(stereo_matching_params),
      dense_stereo_params_() {
  dense_stereo_params_.use_sgbm_ = true;
  dense_stereo_params_.use_mode_HH_ = false;
  dense_stereo_params_.min_disparity_ = 0;  // Always 0 for libSGM
  dense_stereo_params_.num_disparities_ = stereo_matching_params_.templ_cols_;
  dense_stereo_params_.sad_window_size_ = 5;
  // LibSGM uses different P1/P2 scaling - use more conservative values
  int P1 = 10;            // Small penalty for small disparity changes
  int P2 = 120;           // Larger penalty for large disparity changes
  int disp12MaxDiff = 2;  // Relaxed LR consistency check (was 1, too strict)
  int preFilterCap = 31;
  int uniquenessRatio = 10;     // 10% margin for uniqueness (90% confidence)
  int speckleWindowSize = 100;  // Larger window for speckle filtering
  int speckleRange = 2;
  dense_stereo_params_.p1_ = P1;
  dense_stereo_params_.p2_ = P2;
  dense_stereo_params_.disp_12_max_diff_ = disp12MaxDiff;
  dense_stereo_params_.pre_filter_cap_ = preFilterCap;
  dense_stereo_params_.uniqueness_ratio_ = uniquenessRatio;
  dense_stereo_params_.speckle_window_size_ = speckleWindowSize;
  dense_stereo_params_.speckle_range_ = speckleRange;
  dense_stereo_params_.median_blur_disparity_ = true;
}

void StereoMatcher::denseStereoReconstruction(
    const cv::Mat& left_img_rectified,
    const cv::Mat& right_img_rectified,
    cv::Mat* disparity_img) {
  CHECK_NOTNULL(disparity_img);
  CHECK_EQ(disparity_img->cols, left_img_rectified.cols);
  CHECK_EQ(right_img_rectified.cols, left_img_rectified.cols);
  CHECK_EQ(disparity_img->rows, left_img_rectified.rows);
  CHECK_EQ(right_img_rectified.rows, left_img_rectified.rows);
  CHECK_EQ(right_img_rectified.type(), left_img_rectified.type());
  CHECK_EQ(disparity_img->type(), CV_32F);
  CHECK(stereo_camera_);

  // LibSGM only supports grayscale images - convert if needed
  cv::Mat left_gray, right_gray;
  if (left_img_rectified.channels() == 3) {
    cv::cvtColor(left_img_rectified, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img_rectified, right_gray, cv::COLOR_BGR2GRAY);
  } else if (left_img_rectified.channels() == 1) {
    left_gray = left_img_rectified;
    right_gray = right_img_rectified;
  } else {
    LOG(FATAL) << "Input image must have 1 or 3 channels, got "
               << left_img_rectified.channels();
  }

  // Downsample images for SGM to reduce GPU memory usage
  cv::Mat left_sgm, right_sgm;
  int downscale_factor =
      std::max(1, dense_stereo_params_.sgm_downscale_factor_);
  if (downscale_factor > 1) {
    cv::resize(left_gray,
               left_sgm,
               cv::Size(left_gray.cols / downscale_factor,
                        left_gray.rows / downscale_factor),
               0,
               0,
               cv::INTER_AREA);
    cv::resize(right_gray,
               right_sgm,
               cv::Size(right_gray.cols / downscale_factor,
                        right_gray.rows / downscale_factor),
               0,
               0,
               cv::INTER_AREA);
    VLOG(1) << "Downsampled images from " << left_gray.cols << "x"
            << left_gray.rows << " to " << left_sgm.cols << "x" << left_sgm.rows
            << " (factor: " << downscale_factor << ") for SGM processing";
  } else {
    left_sgm = left_gray;
    right_sgm = right_gray;
  }

  // Setup stereo matcher
  if (sgm_ == nullptr) {
    if (dense_stereo_params_.use_sgbm_) {
      if (dense_stereo_params_.use_mode_HH_) {
        LOG(FATAL) << "Only MODE_SGBM is supported for now.";
      }
      sgm::StereoSGM::Parameters sgm_params;
      sgm_params.P1 = dense_stereo_params_.p1_;
      sgm_params.P2 = dense_stereo_params_.p2_;
      sgm_params.uniqueness =
          1.0f - dense_stereo_params_.uniqueness_ratio_ / 100.0f;
      sgm_params.subpixel = false;
      sgm_params.min_disp = dense_stereo_params_.min_disparity_;
      sgm_params.LR_max_diff = dense_stereo_params_.disp_12_max_diff_;
      sgm_params.census_type = sgm::CensusType::SYMMETRIC_CENSUS_9x7;
      sgm_params.path_type = sgm::PathType::SCAN_8PATH;

      int in_bit = 8;
      // Use downsampled image dimensions for SGM initialization
      int sgm_num_disparities =
          dense_stereo_params_.num_disparities_ / downscale_factor;
      sgm_num_disparities = std::max(
          16, (sgm_num_disparities / 16) * 16);  // Round to multiple of 16
      sgm_ = std::make_shared<sgm::StereoSGM>(left_sgm.cols,
                                              left_sgm.rows,
                                              sgm_num_disparities,
                                              in_bit,
                                              16,
                                              sgm::EXECUTE_INOUT_HOST2HOST,
                                              sgm_params);

      VLOG(1) << "LibSGM parameters: P1=" << sgm_params.P1
              << ", P2=" << sgm_params.P2
              << ", uniqueness=" << sgm_params.uniqueness
              << ", num_disp=" << sgm_num_disparities
              << ", LR_max_diff=" << sgm_params.LR_max_diff
              << ", input_channels=" << left_img_rectified.channels()
              << " (converted to grayscale)"
              << ", downscale_factor=" << downscale_factor
              << ", SGM image size=" << left_sgm.cols << "x" << left_sgm.rows;
      // cv_stereo_matcher =
      //     cv::StereoSGBM::create(dense_stereo_params_.min_disparity_,
      //                            dense_stereo_params_.num_disparities_,
      //                            dense_stereo_params_.sad_window_size_,
      //                            dense_stereo_params_.p1_ * n_channel,
      //                            dense_stereo_params_.p2_ * n_channel,
      //                            dense_stereo_params_.disp_12_max_diff_,
      //                            dense_stereo_params_.pre_filter_cap_,
      //                            dense_stereo_params_.uniqueness_ratio_,
      //                            dense_stereo_params_.speckle_window_size_,
      //                            dense_stereo_params_.speckle_range_,
      //                            mode);
    } else {
      LOG(FATAL) << "Only CUDA SGM dense stereo is supported for now.";
    }
  }

  // Reconstruct scene
  // LibSGM outputs CV_16S format (disparity scaled by 16 when subpixel=true)
  cv::Mat disparity_16s_sgm(left_sgm.size(), CV_16S);

  // Synchronize CUDA before libsgm execution to avoid conflicts with vilib/ONNX
  cudaError_t sync_err = cudaDeviceSynchronize();
  if (sync_err != cudaSuccess) {
    LOG(ERROR) << "CUDA sync error before libsgm: "
               << cudaGetErrorString(sync_err);
    cudaGetLastError();  // Clear error
  }

  // Log GPU memory usage before and after libsgm execution to detect
  // possible memory increases / leaks.
  size_t free_before = 0, total_before = 0;
  size_t used_before = 0;
  cudaError_t mem_err = cudaMemGetInfo(&free_before, &total_before);
  if (mem_err == cudaSuccess) {
    used_before = total_before - free_before;
    VLOG(10) << "GPU memory before libsgm: used=" << used_before
              << " bytes, free=" << free_before << " bytes, total="
              << total_before << " bytes";
  } else {
    LOG(ERROR) << "cudaMemGetInfo before libsgm failed: "
               << cudaGetErrorString(mem_err);
  }

  sgm_->execute(left_sgm.data, right_sgm.data, disparity_16s_sgm.data);

  // Log memory after execution and compute difference.
  size_t free_after = 0, total_after = 0;
  size_t used_after = 0;
  mem_err = cudaMemGetInfo(&free_after, &total_after);
  if (mem_err == cudaSuccess) {
    used_after = total_after - free_after;
    VLOG(10) << "GPU memory after libsgm: used=" << used_after
              << " bytes, free=" << free_after << " bytes, total="
              << total_after << " bytes, increase=" << (used_after - used_before)
              << " bytes";
  } else {
    LOG(ERROR) << "cudaMemGetInfo after libsgm failed: "
               << cudaGetErrorString(mem_err);
  }

  // Check for errors immediately after libsgm
  cudaError_t exec_err = cudaGetLastError();
  if (exec_err != cudaSuccess) {
    LOG(ERROR) << "CUDA error after libsgm execute: "
               << cudaGetErrorString(exec_err)
               << " - This indicates GPU memory corruption!";
  }

  // Synchronize after libsgm to ensure completion before other CUDA ops
  sync_err = cudaDeviceSynchronize();
  if (sync_err != cudaSuccess) {
    LOG(ERROR) << "CUDA sync error after libsgm: "
               << cudaGetErrorString(sync_err)
               << " - GPU may be in unstable state!";
    cudaGetLastError();  // Clear error to allow continuation
  }

  // Get invalid disparity value
  int invalid_disp = sgm_->get_invalid_disparity();

  // Upsample disparity back to original resolution if downsampled
  cv::Mat disparity_16s;
  if (downscale_factor > 1) {
    cv::resize(disparity_16s_sgm,
               disparity_16s,
               cv::Size(left_gray.cols, left_gray.rows),
               0,
               0,
               cv::INTER_LINEAR);
    // Scale disparity values by downscale factor (disparity scales with image
    // size)
    disparity_16s *= downscale_factor;
    VLOG(1) << "Upsampled disparity from " << disparity_16s_sgm.cols << "x"
            << disparity_16s_sgm.rows << " to " << disparity_16s.cols << "x"
            << disparity_16s.rows << " and scaled disparities by "
            << downscale_factor;
  } else {
    disparity_16s = disparity_16s_sgm;
  }

  // Create mask for invalid disparities before conversion
  cv::Mat valid_mask = disparity_16s != (invalid_disp * downscale_factor);

  // Convert from CV_16S to CV_32F and divide by 16 to get actual disparity
  disparity_16s.convertTo(*disparity_img, CV_32F);

  // Optionally, smooth the disparity image BEFORE marking invalid pixels
  // This prevents median blur from blending invalid values with valid ones
  if (dense_stereo_params_.median_blur_disparity_) {
    cv::Mat disparity_valid;
    disparity_img->copyTo(disparity_valid, valid_mask);
    cv::medianBlur(disparity_valid, disparity_valid, 5);
    disparity_valid.copyTo(*disparity_img, valid_mask);
  }

  // Set invalid pixels to -1 (negative to indicate invalid)
  disparity_img->setTo(-1.0f, ~valid_mask);

  // Optionally, post-filter disparity
  if (dense_stereo_params_.post_filter_disparity_) {
    // Use disparity post-filter
    // wls_filter = createDisparityWLSFilter(left_matcher);
    // Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    // See
    // https://docs.opencv.org/3.3.1/d3/d14/tutorial_ximgproc_disparity_filtering.html#gsc.tab=0
  }

  static constexpr bool debug = false;
  if (debug) {
    // cv::Mat raw_disp_vis;
    // cv::ximgproc::getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    // cv::namedWindow("raw disparity", WINDOW_AUTOSIZE);
    // cv::imshow("raw disparity", raw_disp_vis);
    // cv::Mat filtered_disp_vis;
    // cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    // cv::namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    // cv::imshow("filtered disparity", filtered_disp_vis);
    // cv::waitKey();
  }
}

void StereoMatcher::sparseStereoReconstruction(StereoFrame* stereo_frame) {
  CHECK_NOTNULL(stereo_frame);
  //! Undistort rectify left/right images
  // CHECK(!stereo_frame->isRectified());
  // TODO(marcus): LoopClosureDetector rewrites stereoframes that are already
  //   rectified using this function! That's why the above check doesn't work...
  if (stereo_frame->isRectified()) {
    VLOG(1) << "sparseStereoMatching: StereoFrame is already rectified!";
  }
  stereo_camera_->undistortRectifyStereoFrame(stereo_frame);
  CHECK(stereo_frame->isRectified());

  //! Undistort rectify left keypoints
  CHECK_GT(stereo_frame->left_frame_.keypoints_.size(), 0u)
      << "Call feature detection on left frame first...";
  stereo_camera_->undistortRectifyLeftKeypoints(
      stereo_frame->left_frame_.keypoints_,
      &stereo_frame->left_keypoints_rectified_);

  CHECK(!stereo_frame->getLeftImgRectified().empty() &&
        !stereo_frame->getRightImgRectified().empty())
      << "sparseStereoMatching: rectified images are empty!";

  // Use dense stereo reconstruction to get disparity image
  cv::Mat disparity_img(stereo_frame->getLeftImgRectified().rows,
                        stereo_frame->getLeftImgRectified().cols,
                        CV_32F);
  auto start_dense = std::chrono::high_resolution_clock::now();
  denseStereoReconstruction(stereo_frame->getLeftImgRectified(),
                            stereo_frame->getRightImgRectified(),
                            &disparity_img);
  auto end_dense = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dense_duration = end_dense - start_dense;

  stereo_frame->left_disp_img_ = disparity_img.clone();

  // Generate left depth image for visualization/debugging
  // depth = fx * baseline / disparity (vectorized operation)
  const auto& stereo_calib = stereo_camera_->getStereoCalib();
  CHECK(stereo_calib);
  double fx_b = stereo_calib->fx() * stereo_camera_->getBaseline();

  // Create mask for valid disparities (positive values)
  cv::Mat valid_disp_mask = disparity_img > 0.0f;

  // Compute depth only for valid disparities: depth = fx_b / disparity
  stereo_frame->left_depth_img_ = cv::Mat::zeros(disparity_img.size(), CV_32F);
  cv::divide(fx_b, disparity_img, stereo_frame->left_depth_img_, 1.0, CV_32F);

  // Set invalid depths back to 0
  stereo_frame->left_depth_img_.setTo(0.0f, ~valid_disp_mask);

  stereo_frame->keypoints_depth_.clear();
  stereo_frame->keypoints_depth_.reserve(
      stereo_frame->left_keypoints_rectified_.size());
  stereo_frame->right_keypoints_rectified_.clear();
  stereo_frame->right_keypoints_rectified_.reserve(
      stereo_frame->left_keypoints_rectified_.size());

  size_t n_valid_depths = 0;
  size_t n_invalid_disparity = 0;
  size_t n_out_of_range = 0;
  size_t n_bounds_check_failed = 0;

  for (const auto& left_kpt : stereo_frame->left_keypoints_rectified_) {
    if (left_kpt.first != KeypointStatus::VALID) {
      stereo_frame->right_keypoints_rectified_.push_back(
          std::make_pair(left_kpt.first, KeypointCV(0.0, 0.0)));
      stereo_frame->keypoints_depth_.push_back(0.0);
      continue;
    }

    int x = static_cast<int>(std::round(left_kpt.second.x));
    int y = static_cast<int>(std::round(left_kpt.second.y));

    // Check bounds
    if (x < 0 || x >= disparity_img.cols || y < 0 || y >= disparity_img.rows) {
      stereo_frame->right_keypoints_rectified_.push_back(
          std::make_pair(KeypointStatus::NO_DEPTH, KeypointCV(0.0, 0.0)));
      stereo_frame->keypoints_depth_.push_back(0.0);
      n_bounds_check_failed++;
      continue;
    }

    float disparity = disparity_img.at<float>(y, x);

    // Check if disparity is valid (negative values indicate invalid matches)
    if (disparity <= 0.0f || !std::isfinite(disparity)) {
      stereo_frame->right_keypoints_rectified_.push_back(
          std::make_pair(KeypointStatus::NO_DEPTH, KeypointCV(0.0, 0.0)));
      stereo_frame->keypoints_depth_.push_back(0.0);
      n_invalid_disparity++;
      VLOG(3) << "Invalid disparity " << disparity << " at keypoint (" << x
              << ", " << y << ")";
      continue;
    }

    double depth = fx_b / disparity;

    // Check depth range
    if (depth < stereo_matching_params_.min_point_dist_ ||
        depth > stereo_matching_params_.max_point_dist_) {
      stereo_frame->right_keypoints_rectified_.push_back(
          std::make_pair(KeypointStatus::NO_DEPTH, KeypointCV(0.0, 0.0)));
      stereo_frame->keypoints_depth_.push_back(0.0);
      n_out_of_range++;
      VLOG(3) << "Depth " << depth << " out of range ["
              << stereo_matching_params_.min_point_dist_ << ", "
              << stereo_matching_params_.max_point_dist_ << "] at keypoint ("
              << x << ", " << y << "), disparity=" << disparity;
      continue;
    }

    // Compute right keypoint location: right_x = left_x - disparity
    KeypointCV right_kpt(left_kpt.second.x - disparity, left_kpt.second.y);
    stereo_frame->right_keypoints_rectified_.push_back(
        std::make_pair(KeypointStatus::VALID, right_kpt));
    stereo_frame->keypoints_depth_.push_back(depth);
    n_valid_depths++;
  }

  //! Fill out right frame keypoints
  CHECK_GT(stereo_frame->right_keypoints_rectified_.size(), 0);
  stereo_camera_->distortUnrectifyRightKeypoints(
      stereo_frame->right_keypoints_rectified_,
      &stereo_frame->right_frame_.keypoints_);

  //! Fill out 3D keypoints in ref frame of left camera
  stereo_frame->keypoints_3d_.clear();
  stereo_frame->keypoints_3d_.reserve(
      stereo_frame->right_keypoints_rectified_.size());
  size_t n_valid_points = 0;
  for (size_t i = 0; i < stereo_frame->right_keypoints_rectified_.size(); i++) {
    if (stereo_frame->right_keypoints_rectified_[i].first ==
        KeypointStatus::VALID) {
      // NOTE: versors are already in the rectified frame.
      Vector3 versor = stereo_frame->left_frame_.versors_.at(i);
      CHECK_GE(versor(2), 1e-3)
          << "sparseStereoMatching: found point with nonpositive depth!";
      // keypoints_depth_ is not the norm of the vector, it is the z component.
      stereo_frame->keypoints_3d_.push_back(
          versor * stereo_frame->keypoints_depth_.at(i) / versor(2));
      n_valid_points++;
    } else {
      stereo_frame->keypoints_3d_.push_back(Vector3::Zero());
    }
  }
}

void StereoMatcher::sparseStereoReconstruction(
    const cv::Mat& left_img_rectified,
    const cv::Mat& right_img_rectified,
    const StatusKeypointsCV& left_keypoints_rectified,
    StatusKeypointsCV* right_keypoints_rectified) {
  CHECK_NOTNULL(right_keypoints_rectified);
  CHECK(stereo_camera_);
  const auto& stereo_calib = stereo_camera_->getStereoCalib();
  CHECK(stereo_calib);
  const auto& baseline = stereo_calib->baseline();
  const auto& fx = stereo_calib->fx();
  getRightKeypointsRectified(left_img_rectified,
                             right_img_rectified,
                             left_keypoints_rectified,
                             fx,
                             baseline,
                             right_keypoints_rectified);
}

void StereoMatcher::getRightKeypointsRectified(
    const cv::Mat& left_img_rectified,
    const cv::Mat& right_img_rectified,
    const StatusKeypointsCV& left_keypoints_rectified,
    const double& fx,
    const double& baseline,
    StatusKeypointsCV* right_keypoints_rectified) const {
  CHECK_NOTNULL(right_keypoints_rectified)->clear();
  right_keypoints_rectified->reserve(left_keypoints_rectified.size());

  int verbosity = 0;

  // The stripe has to be placed in the right image, on the left-hand-side wrt
  // x of the left feature, since: disparity = left_px.x - right_px.x, hence
  // we check: right_px.x < left_px.x a stripe to select in the right image
  // (this must contain match as epipolar lines are horizontal)
  // must be odd; p/m stripe_extra_rows/2 pixels
  // to deal with rectification error
  int stripe_rows = stereo_matching_params_.templ_rows_ +
                    stereo_matching_params_.stripe_extra_rows_;

  // dimension of the search space in right camera is defined by min depth:
  // depth = fx * b / disparity => max disparity = fx * b / minDepth;
  int stripe_cols =
      std::round(fx * baseline / stereo_matching_params_.min_point_dist_) +
      stereo_matching_params_.templ_cols_ + 4;  // 4 is a tolerance

  if (stripe_cols % 2 != 1) {
    // make it odd, if it is not
    stripe_cols += 1;
  }

  if (stripe_cols > right_img_rectified.cols) {
    // if we exagerated with the stripe columns
    stripe_cols = right_img_rectified.cols;
  }

  // Serial version (could be parallelized).
  for (const StatusKeypointCV& left_keypoint_rectified :
       left_keypoints_rectified) {
    // If left point is invalid, set right point to be invalid and continue
    if (left_keypoint_rectified.first != KeypointStatus::VALID) {
      // Skip invalid points (fill in with placeholders in right)
      // Gtsam is able to deal with non-valid stereo matches.
      right_keypoints_rectified->push_back(
          std::make_pair(left_keypoint_rectified.first, KeypointCV(0.0, 0.0)));
      continue;
    }

    // Do left->right matching
    const KeypointCV& left_rectified_i = left_keypoint_rectified.second;
    StatusKeypointCV right_rectified_i_candidate;
    double matching_val_LR;
    searchRightKeypointEpipolar(left_img_rectified,
                                left_rectified_i,
                                right_img_rectified,
                                stripe_cols,
                                stripe_rows,
                                stereo_matching_params_,
                                &right_rectified_i_candidate,
                                &matching_val_LR);

    // TODO(Toni): Here we could perform bidirectional checking...

    right_keypoints_rectified->push_back(right_rectified_i_candidate);
  }

  if (verbosity > 0) {
    std::vector<cv::DMatch> matches;
    for (size_t i = 0; i < left_keypoints_rectified.size(); ++i) {
      if (left_keypoints_rectified[i].first == KeypointStatus::VALID &&
          right_keypoints_rectified->at(i).first == KeypointStatus::VALID) {
        matches.push_back(cv::DMatch(i, i, 0.0));
      }
    }

    const auto match_img =
        UtilsOpenCV::DrawCornersMatches(left_img_rectified,
                                        left_keypoints_rectified,
                                        right_img_rectified,
                                        *right_keypoints_rectified,
                                        matches);
    cv::imshow("stereo matches", match_img);
    cv::waitKey(0);
  }
}

void StereoMatcher::searchRightKeypointEpipolar(
    const cv::Mat& left_img_rectified,
    const KeypointCV& left_keypoint_rectified,
    const cv::Mat& right_rectified,
    const int& stripe_cols,
    const int& stripe_rows,
    const StereoMatchingParams& stereo_matching_params,
    StatusKeypointCV* right_keypoint_rectified,
    double* score) const {
  CHECK_NOTNULL(right_keypoint_rectified);
  CHECK_NOTNULL(score);

  // Correlation matrix
  cv::Mat result;

  int rounded_left_rectified_i_x = round(left_keypoint_rectified.x);
  int rounded_left_rectified_i_y = round(left_keypoint_rectified.y);

  // CORRECTLY PLACE THE TEMPLATE (IN LEFT IMAGE)
  // y-component of upper left corner of template
  int temp_corner_y =
      rounded_left_rectified_i_y - (stereo_matching_params.templ_rows_ - 1) / 2;
  if (temp_corner_y < 0 || temp_corner_y + stereo_matching_params.templ_rows_ >
                               left_img_rectified.rows - 1) {
    // template exceeds bottom or top of the image
    // skip point too close to up or down boundary
    *score = -1.0;
    *right_keypoint_rectified =
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, KeypointCV(0.0, 0.0));
    return;
  }
  // Compensate when the template falls off the image.
  int offset_temp = 0;
  int temp_corner_x =
      rounded_left_rectified_i_x - (stereo_matching_params.templ_cols_ - 1) / 2;
  // Template exceeds on the left-hand-side of the image.
  if (temp_corner_x < 0) {
    // offset_temp a bit to make the template inside the image.
    offset_temp = temp_corner_x;
    // Because of the offset_temp, the template corner ends up on the image
    // border
    temp_corner_x = 0;
  }
  // Template exceeds on the right-hand-side of the image
  if (temp_corner_x + stereo_matching_params.templ_cols_ >
      left_img_rectified.cols - 1) {
    LOG_IF(FATAL, offset_temp != 0)
        << "Offset_temp cannot exceed in both directions!";
    // Amount that exceeds
    offset_temp = (temp_corner_x + stereo_matching_params.templ_cols_) -
                  (left_img_rectified.cols - 1);
    // Corner has to be offset_temp to the left by the amount that exceeds
    temp_corner_x -= offset_temp;
  }

  // Create template
  cv::Rect templ_selector(temp_corner_x,
                          temp_corner_y,
                          stereo_matching_params.templ_cols_,
                          stereo_matching_params.templ_rows_);
  cv::Mat templ(left_img_rectified, templ_selector);

  // CORRECTLY PLACE THE STRIPE (IN RIGHT IMAGE)
  // y-component of upper left corner of stripe
  int stripe_corner_y = rounded_left_rectified_i_y - (stripe_rows - 1) / 2;
  if (stripe_corner_y < 0 ||
      stripe_corner_y + stripe_rows > right_rectified.rows - 1) {
    // stripe exceeds bottom or top of the image
    *score = -1.0;
    *right_keypoint_rectified =
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, KeypointCV(0.0, 0.0));
    return;
  }

  // Compensate when the template falls off the image
  int offset_stripe = 0;
  // y-component of upper left corner of stripe
  int stripe_corner_x = rounded_left_rectified_i_x +
                        (stereo_matching_params.templ_cols_ - 1) / 2 -
                        stripe_cols;
  if (stripe_corner_x + stripe_cols > right_rectified.cols - 1) {
    // stripe exceeds on the right of image
    // amount that exceeds
    offset_stripe =
        (stripe_corner_x + stripe_cols) - (right_rectified.cols - 1);
    stripe_corner_x -= offset_stripe;
  }

  // Stripe exceeds on the left of the image
  // set to left-most column
  if (stripe_corner_x < 0) {
    stripe_corner_x = 0;
  }

  // Create stripe
  cv::Rect stripe_selector(
      stripe_corner_x, stripe_corner_y, stripe_cols, stripe_rows);
  cv::Mat stripe(right_rectified, stripe_selector);

  // Find template and normalize results
  double min_val;
  double max_val;
  cv::Point min_loc;
  cv::Point max_loc;

  cv::matchTemplate(stripe, templ, result, CV_TM_SQDIFF);
  normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  // Localizing the best match with minMaxLoc
  cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc, cv::Mat());

  // Position within the result matrix
  cv::Point matchLoc = min_loc;
  matchLoc.x += stripe_corner_x + (stereo_matching_params.templ_cols_ - 1) / 2 +
                offset_temp;
  // From result to image
  matchLoc.y += stripe_corner_y + (stereo_matching_params.templ_rows_ - 1) / 2;
  // Our desired pixel match
  KeypointCV match_px(matchLoc.x, matchLoc.y);

  // Refine keypoint with subpixel accuracy.
  if (stereo_matching_params.subpixel_refinement_) {
    // TODO(Toni): removed hardcoded!
    static const cv::TermCriteria criteria(
        CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    static const cv::Size winSize(10, 10);
    static const cv::Size zeroZone(-1, -1);
    std::vector<cv::Point2f> corner = {match_px};
    cv::cornerSubPix(right_rectified, corner, winSize, zeroZone, criteria);
    match_px = corner[0];
  }

  *score = min_val;
  if (min_val < stereo_matching_params.tolerance_template_matching_) {
    // Valid point with small mismatch wrt template
    *right_keypoint_rectified = std::make_pair(KeypointStatus::VALID, match_px);
  } else {
    *right_keypoint_rectified =
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, match_px);
  }
}

void StereoMatcher::getDepthFromRectifiedMatches(
    StatusKeypointsCV& left_keypoints_rectified,
    StatusKeypointsCV& right_keypoints_rectified,
    std::vector<double>* keypoints_depth) const {
  CHECK_NOTNULL(keypoints_depth)->clear();
  // depth = fx * baseline / disparity (should be fx = focal * sensorsize)
  double fx_b =
      stereo_camera_->getStereoCalib()->fx() * stereo_camera_->getBaseline();

  CHECK_EQ(left_keypoints_rectified.size(), right_keypoints_rectified.size())
      << "getDepthFromRectifiedMatches: size mismatch!";
  keypoints_depth->reserve(left_keypoints_rectified.size());

  int nrValidDepths = 0;
  // disparity = left_px.x - right_px.x, hence we check: right_px.x < left_px.x
  size_t i = 0;
  for (i = 0; i < left_keypoints_rectified.size(); i++) {
    if (left_keypoints_rectified[i].first == KeypointStatus::VALID &&
        right_keypoints_rectified[i].first == KeypointStatus::VALID) {
      KeypointCV left_px = left_keypoints_rectified[i].second;
      KeypointCV right_px = right_keypoints_rectified[i].second;
      double disparity = left_px.x - right_px.x;
      if (disparity >= 0.0) {
        // Valid.
        nrValidDepths += 1;
        double depth = fx_b / disparity;
        if (depth < stereo_matching_params_.min_point_dist_ ||
            depth > stereo_matching_params_.max_point_dist_) {
          right_keypoints_rectified[i].first = KeypointStatus::NO_DEPTH;
          keypoints_depth->push_back(0.0);
        } else {
          keypoints_depth->push_back(depth);
        }
      } else {
        // Right match was wrong.
        right_keypoints_rectified[i].first = KeypointStatus::NO_DEPTH;
        keypoints_depth->push_back(0.0);
      }
    } else {
      // Something is wrong.
      if (left_keypoints_rectified[i].first != KeypointStatus::VALID &&
          right_keypoints_rectified.at(i).first !=
              left_keypoints_rectified[i].first) {
        // We cannot have a valid right, without a valid left keypoint.
        LOG(WARNING)
            << "Cannot have a valid right kpt without also a valid left kpt!"
            << "\nLeft kpt status: "
            << to_underlying(left_keypoints_rectified[i].first)
            << "\nRight kpt status: "
            << to_underlying(right_keypoints_rectified.at(i).first);
        right_keypoints_rectified.at(i).first =
            left_keypoints_rectified[i].first;
      }
      keypoints_depth->push_back(0.0);
    }
  }
  CHECK_EQ(left_keypoints_rectified.size(), keypoints_depth->size())
      << "getDepthFromRectifiedMatches: depths size mismatch!";

  VLOG(1) << "getDepthFromRectifiedMatches: Found " << nrValidDepths
          << " valid depths out of " << left_keypoints_rectified.size()
          << " keypoints.";
}

}  // namespace VIO
