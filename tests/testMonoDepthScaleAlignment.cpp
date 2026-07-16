#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

#include "kimera-vio/backend/MonoDepthScaleAlignment.h"
#include "kimera-vio/backend/MonoDepthVGICPFactors.h"

#define EXPECT_TRUE(condition, message)            \
  do {                                             \
    if (!(condition)) {                            \
      std::cerr << "[FAIL] " << (message) << "\n"; \
      return EXIT_FAILURE;                         \
    }                                              \
  } while (false)

namespace {

constexpr double kTolerance = 1e-9;

VIO::MonoDepthRawPacket makePacket(const VIO::FrameId frame_id,
                                   const float depth_value = 2.0f) {
  VIO::MonoDepthRawPacket packet;
  packet.keyframe_id = frame_id;
  packet.timestamp = static_cast<VIO::Timestamp>(frame_id * 1000u);
  packet.depth = cv::Mat(16, 20, CV_32FC1, cv::Scalar(depth_value));
  packet.depth_support_mask = cv::Mat(16, 20, CV_8UC1, cv::Scalar(255u));
  packet.valid_mask = cv::Mat(16, 20, CV_8UC1, cv::Scalar(255u));
  packet.weight_image = cv::Mat(4, 5, CV_32FC1, cv::Scalar(0.75f));
  packet.source_image_bgr = cv::Mat(16, 20, CV_8UC3, cv::Scalar(1, 2, 3));
  packet.source_image_is_undistorted = true;
  packet.intrinsics.fx = 10.0;
  packet.intrinsics.fy = 10.0;
  packet.intrinsics.cx = 9.5;
  packet.intrinsics.cy = 7.5;
  packet.intrinsics.width = packet.depth.cols;
  packet.intrinsics.height = packet.depth.rows;
  packet.body_T_cam = gtsam::Pose3();
  packet.metadata.model_size = packet.weight_image.size();
  return packet;
}
VIO::MonoDepthParams makeParams(
    const VIO::MonoDepthScaleAlignmentMethod method,
    const VIO::MonoDepthMode mode = VIO::MonoDepthMode::kSingleView) {
  VIO::MonoDepthParams params;
  params.enabled = true;
  params.mode = mode;
  params.scale_alignment_method = method;
  params.min_depth_m = 0.1;
  params.max_depth_m = 30.0;
  return params;
}

void addLandmarkPairs(VIO::MonoDepthRawPacket* packet,
                      VIO::PointsWithIdMap* landmarks,
                      const VIO::LandmarkId first_id,
                      const std::size_t count,
                      const double landmark_depth) {
  for (std::size_t i = 0u; i < count; ++i) {
    const VIO::LandmarkId landmark_id =
        first_id + static_cast<VIO::LandmarkId>(i);
    packet->keypoints.emplace_back(static_cast<float>(5u + i),
                                   static_cast<float>(5u + (i % 3u)));
    packet->landmark_ids.push_back(landmark_id);
    (*landmarks)[landmark_id] = gtsam::Point3(0.0, 0.0, landmark_depth);
  }
}

void addLandmarkPairAt(VIO::MonoDepthRawPacket* packet,
                       VIO::PointsWithIdMap* landmarks,
                       const VIO::LandmarkId landmark_id,
                       const cv::Point2f& keypoint,
                       const double landmark_depth) {
  packet->keypoints.push_back(keypoint);
  packet->landmark_ids.push_back(landmark_id);
  (*landmarks)[landmark_id] = gtsam::Point3(0.0, 0.0, landmark_depth);
}

void resizePacketImages(VIO::MonoDepthRawPacket* packet,
                        const int rows,
                        const int cols,
                        const float depth_value) {
  packet->depth = cv::Mat(rows, cols, CV_32FC1, cv::Scalar(depth_value));
  packet->depth_support_mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255u));
  packet->valid_mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255u));
  packet->source_image_bgr = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(1, 2, 3));
  packet->intrinsics.width = cols;
  packet->intrinsics.height = rows;
}

std::size_t countLandmarkSampleStatus(
    const VIO::MonoDepthScaleAlignmentResult& result,
    const VIO::MonoDepthLandmarkScaleSampleStatus status) {
  return static_cast<std::size_t>(
      std::count_if(result.landmark_samples.begin(),
                    result.landmark_samples.end(),
                    [status](const VIO::MonoDepthLandmarkScaleSample& sample) {
                      return sample.status == status;
                    }));
}

int testStrictMethodParsingAndConfiguration() {
  using Method = VIO::MonoDepthScaleAlignmentMethod;
  EXPECT_TRUE(
      VIO::monoDepthScaleAlignmentMethodFromString("none") == Method::kNone,
      "none parses exactly");
  EXPECT_TRUE(VIO::monoDepthScaleAlignmentMethodFromString("relative_pose") ==
                  Method::kRelativePose,
              "relative_pose parses exactly");
  EXPECT_TRUE(VIO::monoDepthScaleAlignmentMethodFromString("landmarks") ==
                  Method::kLandmarks,
              "landmarks parses exactly");
  EXPECT_TRUE(VIO::monoDepthScaleAlignmentMethodToString(
                  Method::kRelativePose) == "relative_pose",
              "method string conversion is stable");

  for (const std::string& invalid : {"NONE", "relative-pose", "landmark", ""}) {
    bool threw = false;
    try {
      static_cast<void>(VIO::monoDepthScaleAlignmentMethodFromString(invalid));
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    EXPECT_TRUE(threw, "unknown or inexact method is rejected: " + invalid);
  }

  bool invalid_pair_threw = false;
  try {
    VIO::validateMonoDepthScaleAlignmentConfiguration(
        VIO::MonoDepthMode::kSingleView, Method::kRelativePose);
  } catch (const std::invalid_argument&) {
    invalid_pair_threw = true;
  }
  EXPECT_TRUE(invalid_pair_threw, "relative_pose with single_view is rejected");
  VIO::validateMonoDepthScaleAlignmentConfiguration(
      VIO::MonoDepthMode::kMultiView, Method::kRelativePose);

  VIO::MonoDepthParams invalid_flatness = makeParams(Method::kLandmarks);
  invalid_flatness.landmark_scale_flatness_radius = 0;
  bool invalid_radius_threw = false;
  try {
    static_cast<void>(VIO::makeMonoDepthScaleAligner(invalid_flatness));
  } catch (const std::invalid_argument&) {
    invalid_radius_threw = true;
  }
  EXPECT_TRUE(invalid_radius_threw,
              "a zero landmark flatness radius is rejected");

  invalid_flatness = makeParams(Method::kLandmarks);
  invalid_flatness.landmark_scale_max_relative_depth_variation =
      std::numeric_limits<double>::quiet_NaN();
  bool invalid_variation_threw = false;
  try {
    static_cast<void>(VIO::makeMonoDepthScaleAligner(invalid_flatness));
  } catch (const std::invalid_argument&) {
    invalid_variation_threw = true;
  }
  EXPECT_TRUE(invalid_variation_threw,
              "a non-finite landmark flatness cutoff is rejected");
  return EXIT_SUCCESS;
}

int testIdentityAlignment() {
  const VIO::MonoDepthRawPacket packet = makePacket(3u);
  const std::map<VIO::FrameId, gtsam::Pose3> poses;
  const VIO::PointsWithIdMap landmarks;
  const auto aligner = VIO::makeMonoDepthScaleAligner(
      makeParams(VIO::MonoDepthScaleAlignmentMethod::kNone));
  const auto result = aligner->align({packet, poses, landmarks});
  EXPECT_TRUE(result.valid, "none alignment is always valid");
  EXPECT_TRUE(std::abs(result.absolute_scale - 1.0) < kTolerance,
              "none alignment returns identity scale");
  EXPECT_TRUE(result.failure_reason.empty(),
              "none alignment has no failure reason");

  const auto aligned = VIO::applyMonoDepthScaleAlignment(packet, result);
  EXPECT_TRUE(aligned && aligned->scale_alignment.valid,
              "identity result produces an aligned packet");
  EXPECT_TRUE(std::abs(aligned->depth.at<float>(0, 0) - 2.0f) < 1e-6f,
              "identity alignment leaves depth values unchanged");
  EXPECT_TRUE(aligned->weight_image.data == packet.weight_image.data,
              "identity alignment reuses the one-time weight image");
  return EXIT_SUCCESS;
}

int testRelativePoseAlignment() {
  using Method = VIO::MonoDepthScaleAlignmentMethod;
  VIO::MonoDepthRawPacket packet = makePacket(2u);
  packet.da3_context_keyframe_id = 1u;
  packet.da3_context_body_T_cam = gtsam::Pose3();
  packet.da3_context_cam_T_current_cam =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0));
  const double pi = std::acos(-1.0);
  std::map<VIO::FrameId, gtsam::Pose3> poses{
      {1u,
       gtsam::Pose3(gtsam::Rot3::Rz(pi / 2.0), gtsam::Point3(1.0, 0.0, 0.0))},
      {2u, gtsam::Pose3(gtsam::Rot3::Rz(pi), gtsam::Point3(0.0, 1.0, 0.0))}};
  const VIO::PointsWithIdMap landmarks;
  const auto aligner = VIO::makeMonoDepthScaleAligner(
      makeParams(Method::kRelativePose, VIO::MonoDepthMode::kMultiView));
  const auto result = aligner->align({packet, poses, landmarks});
  EXPECT_TRUE(result.valid, "non-zero relative-pose displacements are valid");
  EXPECT_TRUE(
      std::abs(result.absolute_scale - std::sqrt(2.0) / 2.0) < kTolerance,
      "relative scale uses endpoint chord divided by DA3 displacement");
  EXPECT_TRUE(std::abs(result.metrics.at("odometry_camera_displacement") -
                       std::sqrt(2.0)) < kTolerance,
              "arc motion uses its endpoint camera-center chord");
  EXPECT_TRUE(result.candidate_count == 1u && result.inlier_count == 1u,
              "a valid pose pair contributes one generic candidate/inlier");

  VIO::MonoDepthRawPacket missing = makePacket(2u);
  EXPECT_TRUE(!aligner->align({missing, poses, landmarks}).valid,
              "missing DA3 pair metadata is rejected");

  packet.da3_context_cam_T_current_cam = gtsam::Pose3();
  EXPECT_TRUE(!aligner->align({packet, poses, landmarks}).valid,
              "zero DA3 displacement is rejected");
  packet.da3_context_cam_T_current_cam =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0));
  poses[2u] = poses[1u];
  EXPECT_TRUE(!aligner->align({packet, poses, landmarks}).valid,
              "zero optimized endpoint displacement is rejected");
  return EXIT_SUCCESS;
}

int testIndependentLandmarkScalesAndOutliers() {
  using Method = VIO::MonoDepthScaleAlignmentMethod;
  const auto aligner =
      VIO::makeMonoDepthScaleAligner(makeParams(Method::kLandmarks));
  EXPECT_TRUE(aligner->requiresOptimizedLandmarks(),
              "landmark strategy declares its optimized-landmark dependency");

  VIO::MonoDepthRawPacket first = makePacket(10u, 2.0f);
  VIO::MonoDepthRawPacket second = makePacket(11u, 4.0f);
  VIO::PointsWithIdMap landmarks;
  addLandmarkPairs(&first, &landmarks, 0, 8u, 4.0);
  addLandmarkPairs(&second, &landmarks, 100, 8u, 2.0);
  const std::map<VIO::FrameId, gtsam::Pose3> poses{{10u, gtsam::Pose3()},
                                                   {11u, gtsam::Pose3()}};
  const auto first_result = aligner->align({first, poses, landmarks});
  const auto second_result = aligner->align({second, poses, landmarks});
  EXPECT_TRUE(first_result.valid && second_result.valid,
              "the eight-pair boundary is accepted independently");
  EXPECT_TRUE(std::abs(first_result.absolute_scale - 2.0) < kTolerance,
              "first keyframe receives its own landmark scale");
  EXPECT_TRUE(std::abs(second_result.absolute_scale - 0.5) < kTolerance,
              "second keyframe receives a different landmark scale");

  for (VIO::LandmarkId landmark_id = 0; landmark_id < 8; ++landmark_id) {
    landmarks[landmark_id] = gtsam::Point3(0.0, 0.0, 20.0);
  }
  const auto frozen_first_result = aligner->align({first, poses, landmarks});
  EXPECT_TRUE(
      frozen_first_result == first_result &&
          frozen_first_result.metrics.at("scale_frozen") == 1.0,
      "a keyframe keeps its first valid landmark scale after landmarks move");

  VIO::MonoDepthRawPacket outliers = makePacket(12u, 2.0f);
  VIO::PointsWithIdMap outlier_landmarks;
  addLandmarkPairs(&outliers, &outlier_landmarks, 200, 8u, 4.0);
  addLandmarkPairs(&outliers, &outlier_landmarks, 300, 2u, 10.0);
  const std::map<VIO::FrameId, gtsam::Pose3> outlier_pose{
      {12u, gtsam::Pose3()}};
  const auto outlier_result =
      aligner->align({outliers, outlier_pose, outlier_landmarks});
  EXPECT_TRUE(outlier_result.valid,
              "eight consistent pairs survive two ratio outliers");
  EXPECT_TRUE(outlier_result.candidate_count == 10u &&
                  outlier_result.inlier_count == 8u,
              "landmark diagnostics report candidates and robust inliers");
  EXPECT_TRUE(std::abs(outlier_result.absolute_scale - 2.0) < kTolerance,
              "ratio outliers do not bias the robust scale");
  EXPECT_TRUE(
      outlier_result.landmark_samples.size() == 10u &&
          countLandmarkSampleStatus(
              outlier_result,
              VIO::MonoDepthLandmarkScaleSampleStatus::kLogRatioOutlier) == 2u,
      "per-landmark diagnostics identify the two log-ratio outliers");
  return EXIT_SUCCESS;
}

int testLandmarkFlatnessWeightingAndDepthEdgeSuppression() {
  using Method = VIO::MonoDepthScaleAlignmentMethod;
  VIO::MonoDepthParams params = makeParams(Method::kLandmarks);
  params.landmark_scale_flatness_radius = 2;
  params.landmark_scale_max_relative_depth_variation = 0.15;

  VIO::MonoDepthRawPacket softly_weighted = makePacket(13u, 2.0f);
  resizePacketImages(&softly_weighted, 32, 72, 2.0f);
  VIO::PointsWithIdMap soft_landmarks;
  VIO::LandmarkId landmark_id = 400;
  for (const int y : {6, 12}) {
    for (const int x : {6, 12, 18, 24}) {
      addLandmarkPairAt(
          &softly_weighted,
          &soft_landmarks,
          landmark_id++,
          cv::Point2f(static_cast<float>(x), static_cast<float>(y)),
          4.0);
    }
  }

  const std::array<cv::Point, 8u> kNeighborOffsets{cv::Point{-2, -2},
                                                   cv::Point{0, -2},
                                                   cv::Point{2, -2},
                                                   cv::Point{-2, 0},
                                                   cv::Point{2, 0},
                                                   cv::Point{-2, 2},
                                                   cv::Point{0, 2},
                                                   cv::Point{2, 2}};
  for (const int y : {6, 12}) {
    for (const int x : {42, 48, 54, 60}) {
      for (const cv::Point& offset : kNeighborOffsets) {
        softly_weighted.depth.at<float>(y + offset.y, x + offset.x) = 2.2f;
      }
      addLandmarkPairAt(
          &softly_weighted,
          &soft_landmarks,
          landmark_id++,
          cv::Point2f(static_cast<float>(x), static_cast<float>(y)),
          6.0);
    }
  }

  const std::map<VIO::FrameId, gtsam::Pose3> soft_pose{
      {softly_weighted.keyframe_id, gtsam::Pose3()}};
  const auto soft_aligner = VIO::makeMonoDepthScaleAligner(params);
  const auto soft_result =
      soft_aligner->align({softly_weighted, soft_pose, soft_landmarks});
  EXPECT_TRUE(soft_result.valid && soft_result.candidate_count == 16u &&
                  soft_result.inlier_count == 16u,
              "smooth and mildly varying samples remain robust candidates");
  EXPECT_TRUE(
      soft_result.absolute_scale > 2.0 && soft_result.absolute_scale < 2.25,
      "flat surfaces dominate the weighted scale over mildly varying "
      "depth patches");
  EXPECT_TRUE(soft_result.metrics.at("inlier_weight_sum") > 8.0 &&
                  soft_result.metrics.at("inlier_weight_sum") < 11.0,
              "Tukey flatness weights smoothly suppress non-flat samples");
  EXPECT_TRUE(soft_result.metrics.at("depth_edge_rejected_count") == 0.0,
              "sub-threshold depth variation is weighted rather than rejected");
  EXPECT_TRUE(
      soft_result.landmark_samples.size() == 16u &&
          countLandmarkSampleStatus(
              soft_result,
              VIO::MonoDepthLandmarkScaleSampleStatus::kFlatInlier) == 16u,
      "all softly weighted samples retain spatial inlier diagnostics");
  EXPECT_TRUE(soft_result.landmark_samples.front().flatness_weight == 1.0 &&
                  soft_result.landmark_samples.back().flatness_weight < 0.4,
              "per-landmark diagnostics retain the continuous flatness weight");

  VIO::MonoDepthRawPacket edge_suppressed = makePacket(14u, 2.0f);
  resizePacketImages(&edge_suppressed, 48, 64, 2.0f);
  edge_suppressed.depth.colRange(32, edge_suppressed.depth.cols)
      .setTo(cv::Scalar(8.0f));
  VIO::PointsWithIdMap edge_landmarks;
  landmark_id = 500;
  for (const int y : {6, 12}) {
    for (const int x : {6, 12, 18, 24}) {
      addLandmarkPairAt(
          &edge_suppressed,
          &edge_landmarks,
          landmark_id++,
          cv::Point2f(static_cast<float>(x), static_cast<float>(y)),
          4.0);
    }
  }
  for (const int y : {4, 9, 14, 19, 24, 29, 34, 39}) {
    addLandmarkPairAt(&edge_suppressed,
                      &edge_landmarks,
                      landmark_id++,
                      cv::Point2f(31.0f, static_cast<float>(y)),
                      6.0);
  }

  const std::map<VIO::FrameId, gtsam::Pose3> edge_pose{
      {edge_suppressed.keyframe_id, gtsam::Pose3()}};
  const auto edge_aligner = VIO::makeMonoDepthScaleAligner(params);
  const auto edge_result =
      edge_aligner->align({edge_suppressed, edge_pose, edge_landmarks});
  EXPECT_TRUE(edge_result.valid && edge_result.candidate_count == 8u &&
                  edge_result.inlier_count == 8u,
              "eight flat samples survive alongside eight depth-edge samples");
  EXPECT_TRUE(std::abs(edge_result.absolute_scale - 2.0) < kTolerance,
              "depth discontinuities cannot bias the landmark scale");
  EXPECT_TRUE(edge_result.metrics.at("landmark_raw_candidate_count") == 16.0 &&
                  edge_result.metrics.at("depth_edge_rejected_count") == 8.0,
              "alignment diagnostics expose raw and edge-rejected pair counts");
  EXPECT_TRUE(
      edge_result.landmark_samples.size() == 16u &&
          countLandmarkSampleStatus(
              edge_result,
              VIO::MonoDepthLandmarkScaleSampleStatus::kDepthEdgeRejected) ==
              8u,
      "spatial diagnostics preserve every depth-edge rejection");
  return EXIT_SUCCESS;
}


int testLandmarkFailures() {
  const auto aligner = VIO::makeMonoDepthScaleAligner(
      makeParams(VIO::MonoDepthScaleAlignmentMethod::kLandmarks));
  VIO::MonoDepthRawPacket insufficient = makePacket(20u);
  VIO::PointsWithIdMap landmarks;
  addLandmarkPairs(&insufficient, &landmarks, 0, 7u, 4.0);
  const std::map<VIO::FrameId, gtsam::Pose3> poses{{20u, gtsam::Pose3()}};
  const auto insufficient_result =
      aligner->align({insufficient, poses, landmarks});
  EXPECT_TRUE(
      !insufficient_result.valid && insufficient_result.candidate_count == 7u,
      "seven landmark pairs are insufficient");

  VIO::MonoDepthRawPacket malformed = insufficient;
  malformed.depth = cv::Mat(16, 20, CV_8UC1, cv::Scalar(2u));
  const auto malformed_result = aligner->align({malformed, poses, landmarks});
  EXPECT_TRUE(!malformed_result.valid && malformed_result.failure_reason.find(
                                             "malformed") != std::string::npos,
              "malformed depth is rejected with a reason");

  VIO::MonoDepthRawPacket arbitrary_scale = makePacket(20u, 0.1f);
  VIO::PointsWithIdMap arbitrary_scale_landmarks;
  addLandmarkPairs(&arbitrary_scale, &arbitrary_scale_landmarks, 100, 8u, 3.0);
  const auto arbitrary_scale_result =
      aligner->align({arbitrary_scale, poses, arbitrary_scale_landmarks});
  EXPECT_TRUE(arbitrary_scale_result.valid,
              std::string("a finite positive absolute landmark scale is ") +
                  "accepted: " + arbitrary_scale_result.failure_reason);
  EXPECT_TRUE(std::abs(arbitrary_scale_result.absolute_scale - 30.0) < 1e-5,
              "DA3 scale ambiguity is not constrained by a fixed threshold");
  EXPECT_TRUE(
      std::abs(arbitrary_scale_result.metrics.at("estimated_absolute_scale") -
               30.0) < 1e-5,
      "the unconstrained absolute estimate is exposed in diagnostics");
  return EXIT_SUCCESS;
}

int testFailClosedAndRecoveryFromCanonicalData() {
  using Method = VIO::MonoDepthScaleAlignmentMethod;
  VIO::MonoDepthRawPacket canonical = makePacket(31u, 2.0f);
  canonical.da3_context_keyframe_id = 30u;
  canonical.da3_context_body_T_cam = gtsam::Pose3();
  canonical.da3_context_cam_T_current_cam =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  const cv::Mat canonical_depth_snapshot = canonical.depth.clone();
  const cv::Mat canonical_weight_snapshot = canonical.weight_image.clone();
  const VIO::PointsWithIdMap landmarks;
  const auto aligner = VIO::makeMonoDepthScaleAligner(
      makeParams(Method::kRelativePose, VIO::MonoDepthMode::kMultiView));

  VIO::MonoDepthRawPacket distorted_geometry = canonical;
  distorted_geometry.source_image_is_undistorted = false;
  VIO::MonoDepthScaleAlignmentResult otherwise_valid;
  otherwise_valid.method = Method::kRelativePose;
  otherwise_valid.valid = true;
  otherwise_valid.absolute_scale = 2.0;
  const auto geometry_rejected =
      VIO::applyMonoDepthScaleAlignment(distorted_geometry, otherwise_valid);
  EXPECT_TRUE(geometry_rejected && !geometry_rejected->scale_alignment.valid &&
                  cv::countNonZero(geometry_rejected->valid_mask) == 0,
              "raw distorted packet geometry fails closed before consumers");

  std::map<VIO::FrameId, gtsam::Pose3> missing_context{
      {31u, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0))}};
  const auto failed_result =
      aligner->align({canonical, missing_context, landmarks});
  const auto rejected =
      VIO::applyMonoDepthScaleAlignment(canonical, failed_result);
  EXPECT_TRUE(rejected && !rejected->scale_alignment.valid,
              "an invalid estimate creates a rejected packet");
  EXPECT_TRUE(cv::countNonZero(rejected->valid_mask) == 0,
              "failed alignment zeroes the full-resolution valid mask");
  EXPECT_TRUE(cv::countNonZero(rejected->weight_image) == 0,
              "failed alignment zeroes the compact weight image");

  std::map<VIO::FrameId, gtsam::Pose3> recovered_poses{
      {30u, gtsam::Pose3()},
      {31u, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0))}};
  const auto recovered_result =
      aligner->align({canonical, recovered_poses, landmarks});
  const auto recovered =
      VIO::applyMonoDepthScaleAlignment(canonical, recovered_result);
  EXPECT_TRUE(recovered && recovered->scale_alignment.valid,
              "a later update retries and recovers from canonical data");
  EXPECT_TRUE(std::abs(recovered->depth.at<float>(0, 0) - 4.0f) < 1e-6f,
              "recovery applies the new absolute scale once");
  EXPECT_TRUE(recovered->weight_image.data == canonical.weight_image.data,
              "valid recovery reuses the original one-time weights");

  recovered_poses[31u] =
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0));
  const auto later_result =
      aligner->align({canonical, recovered_poses, landmarks});
  const auto later = VIO::applyMonoDepthScaleAlignment(canonical, later_result);
  EXPECT_TRUE(std::abs(later->depth.at<float>(0, 0) - 6.0f) < 1e-6f,
              "later refresh replaces rather than compounds the scale");
  EXPECT_TRUE(
      cv::countNonZero(canonical.depth != canonical_depth_snapshot) == 0,
      "canonical depth remains unchanged across refreshes");
  EXPECT_TRUE(cv::countNonZero(canonical.weight_image !=
                               canonical_weight_snapshot) == 0,
              "canonical compact weights remain unchanged across refreshes");
  return EXIT_SUCCESS;
}


int testVgicpReplacesAcceptedPairsAfterRefresh() {
  VIO::BackendParams backend_params;
  backend_params.vgicp_factors_enabled_ = true;
  backend_params.vgicp_use_weighted_icp_factor_ = true;
  backend_params.vgicp_min_shared_tracks_ = 1;
  backend_params.vgicp_max_edges_per_keyframe_ = 1;
  backend_params.vgicp_downsample_resolution_ = 0.01;
  backend_params.vgicp_voxel_resolution_ = 0.1;
  backend_params.vgicp_covariance_neighbors_ = 5;
  backend_params.vgicp_num_threads_ = 1;
  backend_params.vgicp_min_points_per_keyframe_ = 1;
  backend_params.vgicp_max_correspondence_distance_ = 5.0;
  backend_params.vgicp_factor_weight_ = 1.0;

  VIO::MonoDepthParams mono_depth_params =
      makeParams(VIO::MonoDepthScaleAlignmentMethod::kNone);
  mono_depth_params.point_stride = 1;
  mono_depth_params.max_points_per_keyframe = 1000;

  VIO::MonoDepthScaleAlignmentResult identity;
  identity.method = VIO::MonoDepthScaleAlignmentMethod::kNone;
  identity.valid = true;
  identity.absolute_scale = 1.0;
  const auto first =
      VIO::applyMonoDepthScaleAlignment(makePacket(50u), identity);
  const auto second =
      VIO::applyMonoDepthScaleAlignment(makePacket(51u), identity);
  const std::map<VIO::FrameId, VIO::MonoDepthRawPacket::ConstPtr> packets{
      {50u, first}, {51u, second}};

  VIO::MonoDepthVGICPFactors vgicp(backend_params, mono_depth_params);
  vgicp.replaceRawPackets(packets);
  gtsam::Values state;
  state.insert(gtsam::Symbol(VIO::kPoseSymbolChar, 50u), gtsam::Pose3());
  state.insert(gtsam::Symbol(VIO::kPoseSymbolChar, 51u), gtsam::Pose3());
  const gtsam::Values new_values;
  VIO::FeatureTracks tracks;
  VIO::FeatureTrack track(50u, gtsam::StereoPoint2(1.0, 1.0, 1.0));
  track.obs_.emplace_back(51u, gtsam::StereoPoint2(1.0, 1.0, 1.0), -1.0);
  tracks.emplace(1, std::move(track));

  const gtsam::NonlinearFactorGraph empty_current_graph;
  gtsam::FactorIndices first_delete_slots;
  gtsam::NonlinearFactorGraph first_graph;
  vgicp.addFactors(nullptr,
                   state,
                   new_values,
                   tracks,
                   empty_current_graph,
                   &first_delete_slots,
                   &first_graph);
  EXPECT_TRUE(first_graph.size() == 1u,
              "the first aligned packet pair adds one ICP factor");
  EXPECT_TRUE(first_delete_slots.empty(),
              "the first pair has no stale factor to delete");
  const double first_error = first_graph.at(0u)->error(state);
  vgicp.notifySmootherUpdateResult(true);

  VIO::MonoDepthScaleAlignmentResult doubled_scale = identity;
  doubled_scale.absolute_scale = 2.0;
  const auto rescaled_first =
      VIO::applyMonoDepthScaleAlignment(makePacket(50u), doubled_scale);
  const std::map<VIO::FrameId, VIO::MonoDepthRawPacket::ConstPtr>
      refreshed_packets{{50u, rescaled_first}, {51u, second}};
  vgicp.replaceRawPackets(refreshed_packets);
  gtsam::FactorIndices refresh_delete_slots;
  gtsam::NonlinearFactorGraph refreshed_graph;
  vgicp.addFactors(nullptr,
                   state,
                   new_values,
                   tracks,
                   first_graph,
                   &refresh_delete_slots,
                   &refreshed_graph);
  EXPECT_TRUE(
      refresh_delete_slots.size() == 1u && refresh_delete_slots.front() == 0u,
      "refresh schedules the previous ICP factor slot for deletion");
  EXPECT_TRUE(refreshed_graph.size() == 1u,
              "refresh adds exactly one replacement for the accepted pair");
  const double refreshed_error = refreshed_graph.at(0u)->error(state);
  EXPECT_TRUE(refreshed_error > first_error + 1e-6,
              "the replacement factor consumes the newly scaled cloud data");
  EXPECT_TRUE(refreshed_error < 10.0,
              "ICP factor cost is normalized by its source point count");
  vgicp.notifySmootherUpdateResult(true);

  vgicp.replaceRawPackets(refreshed_packets);
  gtsam::FactorIndices unchanged_delete_slots;
  gtsam::NonlinearFactorGraph unchanged_graph;
  vgicp.addFactors(nullptr,
                   state,
                   new_values,
                   tracks,
                   refreshed_graph,
                   &unchanged_delete_slots,
                   &unchanged_graph);
  EXPECT_TRUE(unchanged_delete_slots.empty() && unchanged_graph.empty(),
              "unchanged packet objects keep their existing factor and cloud");

  VIO::MonoDepthScaleAlignmentResult failure = identity;
  failure.valid = false;
  failure.failure_reason = "synthetic alignment failure";
  const auto rejected_first =
      VIO::applyMonoDepthScaleAlignment(makePacket(50u), failure);
  vgicp.replaceRawPackets({{50u, rejected_first}, {51u, second}});
  gtsam::FactorIndices rejected_delete_slots;
  gtsam::NonlinearFactorGraph rejected_graph;
  vgicp.addFactors(nullptr,
                   state,
                   new_values,
                   tracks,
                   refreshed_graph,
                   &rejected_delete_slots,
                   &rejected_graph);
  EXPECT_TRUE(rejected_delete_slots.size() == 1u && rejected_graph.empty(),
              "a failed refreshed packet removes the stale factor without "
              "using its old cloud");
  vgicp.notifySmootherUpdateResult(true);

  vgicp.replaceRawPackets(packets);
  gtsam::FactorIndices recovery_delete_slots;
  gtsam::NonlinearFactorGraph recovery_graph;
  vgicp.addFactors(nullptr,
                   state,
                   new_values,
                   tracks,
                   empty_current_graph,
                   &recovery_delete_slots,
                   &recovery_graph);
  EXPECT_TRUE(recovery_delete_slots.empty() && recovery_graph.size() == 1u,
              "the accepted pair recovers from canonical data without an old "
              "factor to delete");
  return EXIT_SUCCESS;
}



}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  int status = EXIT_SUCCESS;
  status |= testStrictMethodParsingAndConfiguration();
  status |= testIdentityAlignment();
  status |= testRelativePoseAlignment();
  status |= testIndependentLandmarkScalesAndOutliers();
  status |= testLandmarkFlatnessWeightingAndDepthEdgeSuppression();
  status |= testLandmarkFailures();
  status |= testFailClosedAndRecoveryFromCanonicalData();
  status |= testVgicpReplacesAcceptedPairsAfterRefresh();
  if (status == EXIT_SUCCESS) {
    std::cout << "All mono-depth scale alignment tests PASSED.\n";
  }
  return status;
}
