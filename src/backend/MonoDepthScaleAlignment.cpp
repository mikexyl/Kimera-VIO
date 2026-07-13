#include "kimera-vio/backend/MonoDepthScaleAlignment.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "kimera-vio/common/MonoDepthUtils.h"

namespace VIO {
namespace {

constexpr double kMinDisplacement = 1e-6;
constexpr std::size_t kMinLandmarkPairs = 8u;
constexpr double kLandmarkRatioInlierFactor = 2.0;

struct WeightedLogDepthRatio {
  double log_ratio = 0.0;
  double flatness_weight = 0.0;
  std::size_t diagnostic_index = 0u;
};

MonoDepthScaleAlignmentResult makeResult(
    const MonoDepthScaleAlignmentMethod method) {
  MonoDepthScaleAlignmentResult result;
  result.method = method;
  return result;
}

double weightedMedianLogRatio(std::vector<WeightedLogDepthRatio> candidates) {
  std::sort(
      candidates.begin(),
      candidates.end(),
      [](const WeightedLogDepthRatio& lhs, const WeightedLogDepthRatio& rhs) {
        return lhs.log_ratio < rhs.log_ratio;
      });
  double total_weight = 0.0;
  for (const auto& candidate : candidates) {
    total_weight += candidate.flatness_weight;
  }

  const double half_weight = 0.5 * total_weight;
  double cumulative_weight = 0.0;
  for (std::size_t i = 0u; i < candidates.size(); ++i) {
    cumulative_weight += candidates[i].flatness_weight;
    if (cumulative_weight < half_weight) {
      continue;
    }
    if (cumulative_weight == half_weight && i + 1u < candidates.size()) {
      return 0.5 * (candidates[i].log_ratio + candidates[i + 1u].log_ratio);
    }
    return candidates[i].log_ratio;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

bool sampleDepthBilinear(const cv::Mat& depth,
                         const cv::Mat& valid_mask,
                         const cv::Point2f& px,
                         float* sampled_depth) {
  if (sampled_depth == nullptr || depth.empty() || depth.type() != CV_32FC1 ||
      valid_mask.empty() || valid_mask.type() != CV_8UC1 ||
      valid_mask.rows < depth.rows || valid_mask.cols < depth.cols ||
      !std::isfinite(px.x) || !std::isfinite(px.y) || px.x < 0.0f ||
      px.y < 0.0f || px.x > static_cast<float>(depth.cols - 1) ||
      px.y > static_cast<float>(depth.rows - 1)) {
    return false;
  }

  const int x0 = static_cast<int>(std::floor(px.x));
  const int y0 = static_cast<int>(std::floor(px.y));
  const int x1 = std::min(x0 + 1, depth.cols - 1);
  const int y1 = std::min(y0 + 1, depth.rows - 1);
  if (valid_mask.at<uint8_t>(y0, x0) == 0u ||
      valid_mask.at<uint8_t>(y0, x1) == 0u ||
      valid_mask.at<uint8_t>(y1, x0) == 0u ||
      valid_mask.at<uint8_t>(y1, x1) == 0u) {
    return false;
  }

  const float z00 = depth.at<float>(y0, x0);
  const float z01 = depth.at<float>(y0, x1);
  const float z10 = depth.at<float>(y1, x0);
  const float z11 = depth.at<float>(y1, x1);
  if (!std::isfinite(z00) || !std::isfinite(z01) || !std::isfinite(z10) ||
      !std::isfinite(z11) || z00 <= 0.0f || z01 <= 0.0f || z10 <= 0.0f ||
      z11 <= 0.0f) {
    return false;
  }

  const float wx = px.x - static_cast<float>(x0);
  const float wy = px.y - static_cast<float>(y0);
  *sampled_depth = (1.0f - wx) * (1.0f - wy) * z00 + wx * (1.0f - wy) * z01 +
                   (1.0f - wx) * wy * z10 + wx * wy * z11;
  return true;
}

bool computeDepthFlatnessWeight(const cv::Mat& depth,
                                const cv::Mat& valid_mask,
                                const cv::Point2f& px,
                                const float center_depth,
                                const int radius,
                                const double max_relative_depth_variation,
                                double* relative_depth_variation,
                                double* flatness_weight) {
  if (relative_depth_variation == nullptr || flatness_weight == nullptr ||
      !std::isfinite(center_depth) || center_depth <= 0.0f || radius < 1 ||
      !std::isfinite(max_relative_depth_variation) ||
      max_relative_depth_variation <= 0.0) {
    return false;
  }

  const std::array<cv::Point2f, 8u> kDirections{cv::Point2f{-1.0f, -1.0f},
                                                cv::Point2f{0.0f, -1.0f},
                                                cv::Point2f{1.0f, -1.0f},
                                                cv::Point2f{-1.0f, 0.0f},
                                                cv::Point2f{1.0f, 0.0f},
                                                cv::Point2f{-1.0f, 1.0f},
                                                cv::Point2f{0.0f, 1.0f},
                                                cv::Point2f{1.0f, 1.0f}};
  double max_variation = 0.0;
  for (const cv::Point2f& direction : kDirections) {
    const cv::Point2f neighbor_px = px + static_cast<float>(radius) * direction;
    float neighbor_depth = 0.0f;
    if (!sampleDepthBilinear(depth, valid_mask, neighbor_px, &neighbor_depth)) {
      return false;
    }
    const double min_depth = std::min(static_cast<double>(center_depth),
                                      static_cast<double>(neighbor_depth));
    const double max_depth = std::max(static_cast<double>(center_depth),
                                      static_cast<double>(neighbor_depth));
    if (!std::isfinite(min_depth) || !std::isfinite(max_depth) ||
        min_depth <= 0.0) {
      return false;
    }
    max_variation = std::max(max_variation, max_depth / min_depth - 1.0);
  }

  *relative_depth_variation = max_variation;
  if (max_variation >= max_relative_depth_variation) {
    *flatness_weight = 0.0;
    return true;
  }

  // Tukey's biweight gives flat regions nearly unit weight, smoothly
  // suppresses samples approaching a depth discontinuity, and reaches exactly
  // zero at the configured cutoff.
  const double normalized = max_variation / max_relative_depth_variation;
  const double one_minus_squared = 1.0 - normalized * normalized;
  *flatness_weight = one_minus_squared * one_minus_squared;
  return std::isfinite(*flatness_weight) && *flatness_weight > 0.0;
}

class NoScaleAligner final : public MonoDepthScaleAligner {
 public:
  MonoDepthScaleAlignmentMethod method() const override {
    return MonoDepthScaleAlignmentMethod::kNone;
  }

  MonoDepthScaleAlignmentResult align(
      const MonoDepthScaleAlignmentInput&) const override {
    MonoDepthScaleAlignmentResult result = makeResult(method());
    result.valid = true;
    result.absolute_scale = 1.0;
    return result;
  }
};

class RelativePoseScaleAligner final : public MonoDepthScaleAligner {
 public:
  MonoDepthScaleAlignmentMethod method() const override {
    return MonoDepthScaleAlignmentMethod::kRelativePose;
  }

  MonoDepthScaleAlignmentResult align(
      const MonoDepthScaleAlignmentInput& input) const override {
    MonoDepthScaleAlignmentResult result = makeResult(method());
    const MonoDepthRawPacket& packet = input.canonical_packet;
    if (!packet.da3_context_keyframe_id.has_value() ||
        !packet.da3_context_body_T_cam.has_value() ||
        !packet.da3_context_cam_T_current_cam.has_value()) {
      result.failure_reason = "DA3 two-view relative pose metadata is missing";
      return result;
    }

    const auto context_pose_it =
        input.optimized_body_poses.find(*packet.da3_context_keyframe_id);
    const auto current_pose_it =
        input.optimized_body_poses.find(packet.keyframe_id);
    if (context_pose_it == input.optimized_body_poses.end() ||
        current_pose_it == input.optimized_body_poses.end()) {
      result.failure_reason =
          "optimized endpoint pose is unavailable for the DA3 frame pair";
      return result;
    }

    result.candidate_count = 1u;
    const double da3_displacement =
        packet.da3_context_cam_T_current_cam->translation().norm();
    const gtsam::Pose3 optimized_context_T_current =
        context_pose_it->second.compose(*packet.da3_context_body_T_cam)
            .between(current_pose_it->second.compose(packet.body_T_cam));
    const double optimized_displacement =
        optimized_context_T_current.translation().norm();
    result.metrics["da3_camera_displacement"] = da3_displacement;
    result.metrics["odometry_camera_displacement"] = optimized_displacement;

    if (!std::isfinite(da3_displacement) ||
        da3_displacement <= kMinDisplacement) {
      result.failure_reason =
          "DA3 camera-center displacement is invalid or zero";
      return result;
    }
    if (!std::isfinite(optimized_displacement) ||
        optimized_displacement <= kMinDisplacement) {
      result.failure_reason =
          "odometry endpoint camera-center displacement is invalid or zero";
      return result;
    }

    result.absolute_scale = optimized_displacement / da3_displacement;
    if (!std::isfinite(result.absolute_scale) || result.absolute_scale <= 0.0) {
      result.absolute_scale = 1.0;
      result.failure_reason = "relative-pose depth scale is invalid";
      return result;
    }
    result.inlier_count = 1u;
    result.valid = true;
    return result;
  }
};

class LandmarkScaleAligner final : public MonoDepthScaleAligner {
 public:
  explicit LandmarkScaleAligner(const MonoDepthParams& params)
      : min_depth_m_(params.min_depth_m),
        max_depth_m_(params.max_depth_m),
        flatness_radius_(params.landmark_scale_flatness_radius),
        max_relative_depth_variation_(
            params.landmark_scale_max_relative_depth_variation) {
    if (flatness_radius_ < 1) {
      throw std::invalid_argument(
          "mono_depth.landmark_scale_flatness_radius must be at least 1");
    }
    if (!std::isfinite(max_relative_depth_variation_) ||
        max_relative_depth_variation_ <= 0.0) {
      throw std::invalid_argument(
          "mono_depth.landmark_scale_max_relative_depth_variation must be "
          "finite and positive");
    }
  }

  MonoDepthScaleAlignmentMethod method() const override {
    return MonoDepthScaleAlignmentMethod::kLandmarks;
  }

  bool requiresOptimizedLandmarks() const override { return true; }

  MonoDepthScaleAlignmentResult align(
      const MonoDepthScaleAlignmentInput& input) const override {
    for (auto it = frozen_results_.begin(); it != frozen_results_.end();) {
      if (input.optimized_body_poses.find(it->first) ==
          input.optimized_body_poses.end()) {
        it = frozen_results_.erase(it);
      } else {
        ++it;
      }
    }

    const FrameId frame_id = input.canonical_packet.keyframe_id;
    const auto frozen_it = frozen_results_.find(frame_id);
    if (frozen_it != frozen_results_.end()) {
      return frozen_it->second;
    }

    MonoDepthScaleAlignmentResult result = makeResult(method());
    const MonoDepthRawPacket& packet = input.canonical_packet;
    if (packet.depth.empty() || packet.depth.type() != CV_32FC1) {
      result.failure_reason = "canonical mono-depth image is malformed";
      return result;
    }
    if (packet.valid_mask.empty() || packet.valid_mask.type() != CV_8UC1) {
      result.failure_reason = "canonical mono-depth valid mask is malformed";
      return result;
    }
    if (packet.keypoints.empty() || packet.landmark_ids.empty()) {
      result.failure_reason = "keyframe feature metadata is missing";
      return result;
    }
    if (input.optimized_landmarks.empty()) {
      result.failure_reason = "optimized landmarks are unavailable";
      return result;
    }

    const auto pose_it = input.optimized_body_poses.find(packet.keyframe_id);
    if (pose_it == input.optimized_body_poses.end()) {
      result.failure_reason = "optimized keyframe pose is unavailable";
      return result;
    }
    const gtsam::Pose3 cam_T_smoother =
        pose_it->second.compose(packet.body_T_cam).inverse();

    std::vector<WeightedLogDepthRatio> candidates;
    const std::size_t feature_count =
        std::min(packet.keypoints.size(), packet.landmark_ids.size());
    candidates.reserve(feature_count);
    result.landmark_samples.reserve(feature_count);
    std::size_t raw_candidate_count = 0u;
    std::size_t flatness_support_rejected_count = 0u;
    std::size_t depth_edge_rejected_count = 0u;
    double candidate_weight_sum = 0.0;
    double candidate_squared_weight_sum = 0.0;
    double variation_sum = 0.0;
    double max_observed_variation = 0.0;
    for (std::size_t i = 0u; i < feature_count; ++i) {
      const LandmarkId landmark_id = packet.landmark_ids[i];
      if (landmark_id == -1) {
        continue;
      }
      const auto landmark_it = input.optimized_landmarks.find(landmark_id);
      if (landmark_it == input.optimized_landmarks.end()) {
        continue;
      }

      const Point3 landmark_cam =
          cam_T_smoother.transformFrom(landmark_it->second);
      const double landmark_depth = landmark_cam.z();
      if (!std::isfinite(landmark_depth) || landmark_depth < min_depth_m_ ||
          landmark_depth > max_depth_m_) {
        continue;
      }

      float predicted_depth = 0.0f;
      if (!sampleDepthBilinear(packet.depth,
                               packet.valid_mask,
                               packet.keypoints[i],
                               &predicted_depth)) {
        continue;
      }
      ++raw_candidate_count;
      const double log_depth_ratio =
          std::log(landmark_depth) -
          std::log(static_cast<double>(predicted_depth));

      double relative_depth_variation = 0.0;
      double flatness_weight = 0.0;
      if (!computeDepthFlatnessWeight(packet.depth,
                                      packet.valid_mask,
                                      packet.keypoints[i],
                                      predicted_depth,
                                      flatness_radius_,
                                      max_relative_depth_variation_,
                                      &relative_depth_variation,
                                      &flatness_weight)) {
        ++flatness_support_rejected_count;
        result.landmark_samples.push_back(
            {landmark_id,
             packet.keypoints[i],
             -1.0,
             0.0,
             log_depth_ratio,
             MonoDepthLandmarkScaleSampleStatus::kFlatnessSupportRejected});
        continue;
      }
      max_observed_variation =
          std::max(max_observed_variation, relative_depth_variation);
      if (flatness_weight <= 0.0) {
        ++depth_edge_rejected_count;
        result.landmark_samples.push_back(
            {landmark_id,
             packet.keypoints[i],
             relative_depth_variation,
             0.0,
             log_depth_ratio,
             MonoDepthLandmarkScaleSampleStatus::kDepthEdgeRejected});
        continue;
      }

      const std::size_t diagnostic_index = result.landmark_samples.size();
      result.landmark_samples.push_back(
          {landmark_id,
           packet.keypoints[i],
           relative_depth_variation,
           flatness_weight,
           log_depth_ratio,
           MonoDepthLandmarkScaleSampleStatus::kFlatInlier});
      candidates.push_back(
          {log_depth_ratio, flatness_weight, diagnostic_index});
      candidate_weight_sum += flatness_weight;
      candidate_squared_weight_sum += flatness_weight * flatness_weight;
      variation_sum += relative_depth_variation;
    }

    result.candidate_count = candidates.size();
    result.metrics["landmark_raw_candidate_count"] =
        static_cast<double>(raw_candidate_count);
    result.metrics["flatness_support_rejected_count"] =
        static_cast<double>(flatness_support_rejected_count);
    result.metrics["depth_edge_rejected_count"] =
        static_cast<double>(depth_edge_rejected_count);
    result.metrics["flatness_weight_sum"] = candidate_weight_sum;
    result.metrics["flatness_effective_sample_size"] =
        candidate_squared_weight_sum > 0.0
            ? candidate_weight_sum * candidate_weight_sum /
                  candidate_squared_weight_sum
            : 0.0;
    result.metrics["mean_relative_depth_variation"] =
        candidates.empty()
            ? 0.0
            : variation_sum / static_cast<double>(candidates.size());
    result.metrics["max_relative_depth_variation"] = max_observed_variation;
    result.metrics["flatness_radius_pixels"] =
        static_cast<double>(flatness_radius_);
    result.metrics["flatness_cutoff_relative_depth_variation"] =
        max_relative_depth_variation_;
    if (candidates.size() < kMinLandmarkPairs) {
      if (raw_candidate_count < kMinLandmarkPairs) {
        result.failure_reason = "insufficient landmark depth pairs";
      } else if (depth_edge_rejected_count > 0u) {
        result.failure_reason =
            "insufficient flat-surface landmark depth pairs";
      } else {
        result.failure_reason = "insufficient landmark depth flatness support";
      }
      return result;
    }

    const double median_log_ratio = weightedMedianLogRatio(candidates);
    result.metrics["median_log_depth_ratio"] = median_log_ratio;
    result.metrics["weighted_median_log_depth_ratio"] = median_log_ratio;
    if (!std::isfinite(median_log_ratio)) {
      result.failure_reason =
          "weighted median landmark log-depth ratio is invalid";
      return result;
    }

    const double log_inlier_threshold = std::log(kLandmarkRatioInlierFactor);
    double inlier_weight_sum = 0.0;
    double inlier_squared_weight_sum = 0.0;
    double weighted_log_ratio_sum = 0.0;
    for (const auto& candidate : candidates) {
      if (std::abs(candidate.log_ratio - median_log_ratio) >
          log_inlier_threshold) {
        result.landmark_samples[candidate.diagnostic_index].status =
            MonoDepthLandmarkScaleSampleStatus::kLogRatioOutlier;
        continue;
      }
      weighted_log_ratio_sum += candidate.flatness_weight * candidate.log_ratio;
      inlier_weight_sum += candidate.flatness_weight;
      inlier_squared_weight_sum +=
          candidate.flatness_weight * candidate.flatness_weight;
      ++result.inlier_count;
    }
    result.metrics["inlier_weight_sum"] = inlier_weight_sum;
    result.metrics["inlier_effective_sample_size"] =
        inlier_squared_weight_sum > 0.0
            ? inlier_weight_sum * inlier_weight_sum / inlier_squared_weight_sum
            : 0.0;
    if (result.inlier_count < kMinLandmarkPairs ||
        !std::isfinite(inlier_weight_sum) || inlier_weight_sum <= 0.0) {
      result.failure_reason = "insufficient inlier landmark depth pairs";
      return result;
    }

    const double log_scale = weighted_log_ratio_sum / inlier_weight_sum;
    result.absolute_scale = std::exp(log_scale);
    result.metrics["estimated_absolute_scale"] = result.absolute_scale;
    // DA3 depth is scale-ambiguous, so the absolute multiplier has no useful
    // fixed range.  Reject only estimates that cannot be applied
    // numerically; metric depth bounds are enforced during backprojection.
    if (!std::isfinite(result.absolute_scale) || result.absolute_scale <= 0.0) {
      result.absolute_scale = 1.0;
      result.failure_reason = "landmark depth scale is numerically invalid";
      return result;
    }

    double weighted_squared_log_error = 0.0;
    for (const auto& candidate : candidates) {
      if (std::abs(candidate.log_ratio - median_log_ratio) >
          log_inlier_threshold) {
        continue;
      }
      const double residual = log_scale - candidate.log_ratio;
      weighted_squared_log_error +=
          candidate.flatness_weight * residual * residual;
    }
    result.log_rmse = std::sqrt(weighted_squared_log_error / inlier_weight_sum);
    result.valid = true;
    result.metrics["scale_frozen"] = 1.0;
    frozen_results_[frame_id] = result;
    return result;
  }

 private:
  double min_depth_m_;
  double max_depth_m_;
  int flatness_radius_;
  double max_relative_depth_variation_;
  mutable std::map<FrameId, MonoDepthScaleAlignmentResult> frozen_results_;
};

cv::Size failClosedMaskSize(const MonoDepthRawPacket& packet) {
  if (!packet.valid_mask.empty()) {
    return packet.valid_mask.size();
  }
  return packet.depth.size();
}

cv::Size failClosedWeightSize(const MonoDepthRawPacket& packet) {
  if (!packet.weight_image.empty()) {
    return packet.weight_image.size();
  }
  return packet.metadata.model_size;
}

}  // namespace

MonoDepthScaleAligner::UniquePtr makeMonoDepthScaleAligner(
    const MonoDepthParams& params) {
  validateMonoDepthScaleAlignmentConfiguration(params.mode,
                                               params.scale_alignment_method);
  switch (params.scale_alignment_method) {
    case MonoDepthScaleAlignmentMethod::kNone:
      return std::make_unique<NoScaleAligner>();
    case MonoDepthScaleAlignmentMethod::kRelativePose:
      return std::make_unique<RelativePoseScaleAligner>();
    case MonoDepthScaleAlignmentMethod::kLandmarks:
      return std::make_unique<LandmarkScaleAligner>(params);
  }
  throw std::invalid_argument("Unknown mono-depth scale alignment method");
}

MonoDepthRawPacket::ConstPtr applyMonoDepthScaleAlignment(
    const MonoDepthRawPacket& canonical_packet,
    const MonoDepthScaleAlignmentResult& result) {
  auto aligned_packet = std::make_shared<MonoDepthRawPacket>(canonical_packet);
  aligned_packet->scale_alignment = result;

  const auto fail_closed = [&](const std::string& reason) {
    aligned_packet->scale_alignment.valid = false;
    aligned_packet->scale_alignment.absolute_scale = 1.0;
    if (aligned_packet->scale_alignment.failure_reason.empty()) {
      aligned_packet->scale_alignment.failure_reason = reason;
    }
    aligned_packet->depth = canonical_packet.depth.clone();
    const cv::Size mask_size = failClosedMaskSize(canonical_packet);
    if (mask_size.width > 0 && mask_size.height > 0) {
      aligned_packet->valid_mask = cv::Mat(mask_size, CV_8UC1, cv::Scalar(0u));
    } else {
      aligned_packet->valid_mask.release();
    }
    const cv::Size weight_size = failClosedWeightSize(canonical_packet);
    if (weight_size.width > 0 && weight_size.height > 0) {
      aligned_packet->weight_image =
          cv::Mat(weight_size, CV_32FC1, cv::Scalar(0.0f));
    } else {
      aligned_packet->weight_image.release();
    }
    return MonoDepthRawPacket::ConstPtr(aligned_packet);
  };

  if (!canonical_packet.source_image_is_undistorted) {
    return fail_closed(
        "mono-depth packet is not in undistorted pinhole geometry");
  }
  if (!result.valid) {
    return fail_closed("mono-depth scale alignment failed");
  }
  if (!std::isfinite(result.absolute_scale) || result.absolute_scale <= 0.0) {
    return fail_closed("mono-depth scale alignment returned an invalid scale");
  }

  aligned_packet->depth =
      scaleMonoDepthImage(canonical_packet.depth, result.absolute_scale);
  if (aligned_packet->depth.empty()) {
    return fail_closed("canonical mono-depth image is missing or malformed");
  }
  return aligned_packet;
}

}  // namespace VIO
