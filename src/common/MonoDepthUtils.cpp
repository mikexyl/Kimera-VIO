#include "kimera-vio/common/MonoDepthUtils.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace VIO {
namespace {

constexpr std::size_t kMinDa3OverlapSamples = 32u;

double median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t middle = values.size() / 2u;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2u != 0u) {
    return upper;
  }
  const double lower =
      *std::max_element(values.begin(), values.begin() + middle);
  return 0.5 * (lower + upper);
}

Eigen::Vector3d backprojectDepthPixel(const int u,
                                      const int v,
                                      const float z,
                                      const MonoDepthIntrinsics& intrinsics) {
  return Eigen::Vector3d((static_cast<double>(u) - intrinsics.cx) *
                             static_cast<double>(z) / intrinsics.fx,
                         (static_cast<double>(v) - intrinsics.cy) *
                             static_cast<double>(z) / intrinsics.fy,
                         static_cast<double>(z));
}

}  // namespace

Da3OverlapScaleEstimate estimateDa3OverlapScale(
    const MonoDepthRawPacket& previous_current,
    const MonoDepthRawPacket& next_context,
    const int point_stride) {
  Da3OverlapScaleEstimate result;
  if (previous_current.keyframe_id != next_context.keyframe_id) {
    result.failure_reason = "overlap packets do not describe the same image";
    return result;
  }
  if (previous_current.depth.empty() || next_context.depth.empty() ||
      previous_current.depth.type() != CV_32FC1 ||
      next_context.depth.type() != CV_32FC1 ||
      previous_current.depth.size() != next_context.depth.size() ||
      previous_current.depth_support_mask.empty() ||
      next_context.depth_support_mask.empty() ||
      previous_current.depth_support_mask.type() != CV_8UC1 ||
      next_context.depth_support_mask.type() != CV_8UC1 ||
      previous_current.depth_support_mask.size() !=
          previous_current.depth.size() ||
      next_context.depth_support_mask.size() != next_context.depth.size()) {
    result.failure_reason = "overlap depth or support-mask geometry is invalid";
    return result;
  }

  const int stride = std::max(1, point_stride);
  std::vector<double> log_ratios;
  const std::size_t reserve = static_cast<std::size_t>(
      ((previous_current.depth.rows + stride - 1) / stride) *
      ((previous_current.depth.cols + stride - 1) / stride));
  log_ratios.reserve(reserve);
  for (int v = 0; v < previous_current.depth.rows; v += stride) {
    const float* previous_depth = previous_current.depth.ptr<float>(v);
    const float* context_depth = next_context.depth.ptr<float>(v);
    const uint8_t* previous_support =
        previous_current.depth_support_mask.ptr<uint8_t>(v);
    const uint8_t* context_support =
        next_context.depth_support_mask.ptr<uint8_t>(v);
    for (int u = 0; u < previous_current.depth.cols; u += stride) {
      if (previous_support[u] == 0u || context_support[u] == 0u) {
        continue;
      }
      const double previous_z = static_cast<double>(previous_depth[u]);
      const double context_z = static_cast<double>(context_depth[u]);
      if (!std::isfinite(previous_z) || !std::isfinite(context_z) ||
          previous_z <= 0.0 || context_z <= 0.0) {
        continue;
      }
      log_ratios.push_back(std::log(previous_z / context_z));
    }
  }
  result.candidate_count = log_ratios.size();
  if (log_ratios.size() < kMinDa3OverlapSamples) {
    result.failure_reason = "insufficient same-image depth overlap";
    return result;
  }

  const double initial_median = median(log_ratios);
  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(log_ratios.size());
  for (const double log_ratio : log_ratios) {
    absolute_residuals.push_back(std::abs(log_ratio - initial_median));
  }
  const double mad = median(absolute_residuals);
  const double inlier_threshold = std::max(1e-3, 3.0 * 1.4826 * mad);
  std::vector<double> inliers;
  inliers.reserve(log_ratios.size());
  for (const double log_ratio : log_ratios) {
    if (std::abs(log_ratio - initial_median) <= inlier_threshold) {
      inliers.push_back(log_ratio);
    }
  }
  result.inlier_count = inliers.size();
  if (inliers.size() < kMinDa3OverlapSamples) {
    result.failure_reason = "insufficient robust same-image depth overlap";
    return result;
  }

  const double robust_log_ratio = median(inliers);
  double squared_error_sum = 0.0;
  for (const double log_ratio : inliers) {
    const double residual = log_ratio - robust_log_ratio;
    squared_error_sum += residual * residual;
  }
  result.log_rmse =
      std::sqrt(squared_error_sum / static_cast<double>(inliers.size()));
  result.scale_ratio = std::exp(robust_log_ratio);
  result.valid = std::isfinite(result.scale_ratio) &&
                 result.scale_ratio > 0.0 && std::isfinite(result.log_rmse);
  if (!result.valid) {
    result.scale_ratio = 1.0;
    result.failure_reason = "robust overlap scale is invalid";
  }
  return result;
}

cv::Mat scaleMonoDepthImage(const cv::Mat& canonical_depth,
                            const double depth_scale) {
  if (canonical_depth.empty() || canonical_depth.type() != CV_32FC1 ||
      !std::isfinite(depth_scale) || depth_scale <= 0.0) {
    return {};
  }
  cv::Mat scaled_depth;
  canonical_depth.convertTo(scaled_depth, CV_32FC1, depth_scale);
  return scaled_depth;
}

cv::Mat makeMonoDepthWeightImage(const cv::Mat& depth,
                                 const cv::Mat& valid_mask,
                                 const MonoDepthIntrinsics& intrinsics,
                                 const MonoDepthParams& params) {
  CHECK(!depth.empty());
  CHECK_EQ(depth.type(), CV_32FC1);

  cv::Mat weights(depth.rows, depth.cols, CV_32FC1, cv::Scalar(0.0f));
  if (valid_mask.empty() || valid_mask.type() != CV_8UC1 ||
      intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
    return weights;
  }

  if (!params.depth_weighting_enabled) {
    for (int v = 0; v < std::min(depth.rows, valid_mask.rows); ++v) {
      const uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
      float* weight_row = weights.ptr<float>(v);
      for (int u = 0; u < std::min(depth.cols, valid_mask.cols); ++u) {
        weight_row[u] = valid_row[u] == 0u ? 0.0f : 1.0f;
      }
    }
    return weights;
  }

  const int radius = std::max(1, params.depth_weight_normal_radius);
  const bool use_range_weight = params.depth_weight_range_ref > 0.0;
  const int rows = std::min(depth.rows, valid_mask.rows);
  const int cols = std::min(depth.cols, valid_mask.cols);
  for (int v = radius; v < rows - radius; ++v) {
    const float* depth_row = depth.ptr<float>(v);
    const uint8_t* valid_row = valid_mask.ptr<uint8_t>(v);
    float* weight_row = weights.ptr<float>(v);
    for (int u = radius; u < cols - radius; ++u) {
      if (valid_row[u] == 0u || valid_mask.at<uint8_t>(v, u - radius) == 0u ||
          valid_mask.at<uint8_t>(v, u + radius) == 0u ||
          valid_mask.at<uint8_t>(v - radius, u) == 0u ||
          valid_mask.at<uint8_t>(v + radius, u) == 0u) {
        continue;
      }

      const float z = depth_row[u];
      const float z_l = depth.at<float>(v, u - radius);
      const float z_r = depth.at<float>(v, u + radius);
      const float z_u = depth.at<float>(v - radius, u);
      const float z_d = depth.at<float>(v + radius, u);
      const auto depth_is_valid = [&params](const float depth_value) {
        return std::isfinite(depth_value) &&
               depth_value >= params.min_depth_m &&
               depth_value <= params.max_depth_m;
      };
      if (!depth_is_valid(z) || !depth_is_valid(z_l) || !depth_is_valid(z_r) ||
          !depth_is_valid(z_u) || !depth_is_valid(z_d)) {
        continue;
      }

      const Eigen::Vector3d p = backprojectDepthPixel(u, v, z, intrinsics);
      const Eigen::Vector3d p_l =
          backprojectDepthPixel(u - radius, v, z_l, intrinsics);
      const Eigen::Vector3d p_r =
          backprojectDepthPixel(u + radius, v, z_r, intrinsics);
      const Eigen::Vector3d p_u =
          backprojectDepthPixel(u, v - radius, z_u, intrinsics);
      const Eigen::Vector3d p_d =
          backprojectDepthPixel(u, v + radius, z_d, intrinsics);

      Eigen::Vector3d normal = (p_r - p_l).cross(p_d - p_u);
      const double normal_norm = normal.norm();
      const double point_norm = p.norm();
      if (normal_norm < 1e-9 || point_norm < 1e-9) {
        continue;
      }

      normal /= normal_norm;
      const Eigen::Vector3d view_to_camera = -p / point_norm;
      const double cos_theta =
          std::clamp(std::abs(normal.dot(view_to_camera)), 0.0, 1.0);
      const double grazing_confidence =
          std::pow(cos_theta, params.depth_weight_grazing_power);
      const double grazing_weight =
          params.depth_weight_min +
          (1.0 - params.depth_weight_min) * grazing_confidence;
      const double range_weight =
          use_range_weight
              ? std::clamp(std::pow(params.depth_weight_range_ref / point_norm,
                                    params.depth_weight_range_power),
                           params.depth_weight_range_min,
                           1.0)
              : 1.0;
      weight_row[u] = static_cast<float>(
          std::clamp(grazing_weight * range_weight, 0.0, 1.0));
    }
  }
  return weights;
}

cv::Mat makeMonoDepthWeightImageAtSize(const cv::Mat& depth,
                                       const cv::Mat& valid_mask,
                                       const MonoDepthIntrinsics& intrinsics,
                                       const cv::Size& weight_size,
                                       const MonoDepthParams& params) {
  CHECK(!depth.empty());
  CHECK_EQ(depth.type(), CV_32FC1);

  const cv::Size bounded_size(std::min(depth.cols, weight_size.width),
                              std::min(depth.rows, weight_size.height));
  if (bounded_size.width <= 0 || bounded_size.height <= 0 ||
      bounded_size == depth.size()) {
    return makeMonoDepthWeightImage(depth, valid_mask, intrinsics, params);
  }

  cv::Mat resized_depth;
  cv::resize(depth, resized_depth, bounded_size, 0.0, 0.0, cv::INTER_AREA);

  cv::Mat resized_valid_mask;
  if (!valid_mask.empty() && valid_mask.type() == CV_8UC1) {
    cv::resize(
        valid_mask, resized_valid_mask, bounded_size, 0.0, 0.0, cv::INTER_AREA);
    // A low-resolution depth sample is valid only when its complete source
    // footprint is valid. This avoids mixing rejected confidence or sky pixels
    // into the surface normal used for weighting.
    cv::threshold(resized_valid_mask,
                  resized_valid_mask,
                  254.0,
                  255.0,
                  cv::THRESH_BINARY);
  }

  const double scale_x = static_cast<double>(bounded_size.width) / depth.cols;
  const double scale_y = static_cast<double>(bounded_size.height) / depth.rows;
  MonoDepthIntrinsics resized_intrinsics = intrinsics;
  resized_intrinsics.fx *= scale_x;
  resized_intrinsics.fy *= scale_y;
  resized_intrinsics.cx = (intrinsics.cx + 0.5) * scale_x - 0.5;
  resized_intrinsics.cy = (intrinsics.cy + 0.5) * scale_y - 0.5;
  resized_intrinsics.width = bounded_size.width;
  resized_intrinsics.height = bounded_size.height;

  return makeMonoDepthWeightImage(
      resized_depth, resized_valid_mask, resized_intrinsics, params);
}

float sampleMonoDepthWeight(const cv::Mat& weight_image,
                            const cv::Size& depth_size,
                            const int u,
                            const int v) {
  if (weight_image.empty() || weight_image.type() != CV_32FC1) {
    return 1.0f;
  }
  if (depth_size.width <= 0 || depth_size.height <= 0 || u < 0 || v < 0 ||
      u >= depth_size.width || v >= depth_size.height) {
    return 0.0f;
  }

  // Map pixel centers between grids. Nearest-neighbor lookup is intentional:
  // the weight grid is already smooth, and this keeps point-cloud sampling
  // cheap while preserving exact zero-weight regions.
  const std::int64_t weight_u_numerator =
      (2 * static_cast<std::int64_t>(u) + 1) * weight_image.cols;
  const std::int64_t weight_v_numerator =
      (2 * static_cast<std::int64_t>(v) + 1) * weight_image.rows;
  const int weight_u = std::clamp(
      static_cast<int>(weight_u_numerator /
                       (2 * static_cast<std::int64_t>(depth_size.width))),
      0,
      weight_image.cols - 1);
  const int weight_v = std::clamp(
      static_cast<int>(weight_v_numerator /
                       (2 * static_cast<std::int64_t>(depth_size.height))),
      0,
      weight_image.rows - 1);
  return weight_image.at<float>(weight_v, weight_u);
}

}  // namespace VIO
