#include "kimera-vio/backend/MonoDepthAlignment.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace VIO {

MonoDepthAlignment::MonoDepthAlignment(const MonoDepthParams& params)
    : params_(params) {}

MonoDepthMapOutput::ConstPtr MonoDepthAlignment::process(
    const MonoDepthRawPacket::ConstPtr& raw_packet,
    const gtsam::Values& state,
    const PointsWithIdMap& landmarks,
    const gtsam::Pose3& world_T_smoother) {
  if (!params_.enabled) {
    return nullptr;
  }

  cacheRawPacket(raw_packet);

  const std::optional<FrameId> target_frame_id =
      findOldestSmootherPoseFrameId(state);
  if (!target_frame_id.has_value()) {
    return nullptr;
  }

  if (!last_oldest_frame_id_.has_value()) {
    last_oldest_frame_id_ = *target_frame_id;
    return nullptr;
  }
  if (*last_oldest_frame_id_ == *target_frame_id) {
    return nullptr;
  }
  last_oldest_frame_id_ = *target_frame_id;

  if (last_processed_frame_id_.has_value() &&
      *last_processed_frame_id_ == *target_frame_id) {
    return nullptr;
  }

  const auto raw_it = raw_packet_cache_.find(*target_frame_id);
  if (raw_it == raw_packet_cache_.end() || !raw_it->second) {
    return nullptr;
  }

  const gtsam::Symbol target_pose_key(kPoseSymbolChar, *target_frame_id);
  if (state.find(target_pose_key) == state.end()) {
    return nullptr;
  }

  const MonoDepthRawPacket& target_packet = *raw_it->second;
  if (target_packet.depth.empty() || target_packet.depth.type() != CV_32FC1 ||
      target_packet.valid_mask.empty() ||
      target_packet.valid_mask.type() != CV_8UC1 ||
      target_packet.source_image_bgr.empty()) {
    return nullptr;
  }

  const gtsam::Pose3 smoother_T_body = state.at<Pose3>(target_pose_key);
  const gtsam::Pose3 smoother_T_cam =
      smoother_T_body.compose(target_packet.body_T_cam);
  const gtsam::Pose3 world_T_cam = world_T_smoother.compose(smoother_T_cam);
  const ScaleEstimate scale_estimate = estimateScale(
      target_packet, landmarks, target_packet.depth, smoother_T_cam.inverse());

  MonoDepthMapOutput::ConstPtr output =
      buildMapOutput(target_packet, world_T_cam, scale_estimate);
  if (output) {
    last_processed_frame_id_ = *target_frame_id;
  }
  return output;
}

void MonoDepthAlignment::cacheRawPacket(
    const MonoDepthRawPacket::ConstPtr& raw_packet) {
  if (!raw_packet) {
    return;
  }
  raw_packet_cache_[raw_packet->keyframe_id] = raw_packet;
  while (raw_packet_cache_.size() > kRawPacketCacheSize) {
    raw_packet_cache_.erase(raw_packet_cache_.begin());
  }
}

std::optional<FrameId> MonoDepthAlignment::findOldestSmootherPoseFrameId(
    const gtsam::Values& state) {
  std::optional<FrameId> oldest_frame_id = std::nullopt;
  for (auto key : state.keys()) {
    const gtsam::Symbol symbol(key);
    if (symbol.chr() != kPoseSymbolChar) {
      continue;
    }
    const FrameId frame_id = symbol.index();
    if (!oldest_frame_id.has_value() || frame_id < *oldest_frame_id) {
      oldest_frame_id = frame_id;
    }
  }
  return oldest_frame_id;
}

bool MonoDepthAlignment::sampleDepthBilinear(const cv::Mat& depth,
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

  const float wx = px.x - static_cast<float>(x0);
  const float wy = px.y - static_cast<float>(y0);
  const float z00 = depth.at<float>(y0, x0);
  const float z01 = depth.at<float>(y0, x1);
  const float z10 = depth.at<float>(y1, x0);
  const float z11 = depth.at<float>(y1, x1);
  if (!std::isfinite(z00) || !std::isfinite(z01) || !std::isfinite(z10) ||
      !std::isfinite(z11) || z00 <= 0.0f || z01 <= 0.0f || z10 <= 0.0f ||
      z11 <= 0.0f) {
    return false;
  }

  *sampled_depth = (1.0f - wx) * (1.0f - wy) * z00 +
                   wx * (1.0f - wy) * z01 +
                   (1.0f - wx) * wy * z10 + wx * wy * z11;
  return true;
}

double MonoDepthAlignment::medianValue(std::vector<double> values) {
  CHECK(!values.empty());
  const std::size_t middle = values.size() / 2u;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double median = values[middle];
  if (values.size() % 2u == 0u) {
    std::nth_element(values.begin(), values.begin() + middle - 1u,
                     values.end());
    median = 0.5 * (median + values[middle - 1u]);
  }
  return median;
}

MonoDepthAlignment::ScaleEstimate MonoDepthAlignment::estimateScale(
    const MonoDepthRawPacket& raw_packet,
    const PointsWithIdMap& landmarks,
    const cv::Mat& depth,
    const gtsam::Pose3& cam_T_smoother) const {
  static constexpr std::size_t kMinPairs = 8u;
  static constexpr double kMinScale = 0.05;
  static constexpr double kMaxScale = 20.0;
  static constexpr double kRatioInlierFactor = 2.0;
  const double log_ratio_inlier_threshold = std::log(kRatioInlierFactor);

  ScaleEstimate estimate;
  estimate.scale = scale_valid_ ? scale_ : 1.0;
  if (raw_packet.keypoints.empty() || raw_packet.landmark_ids.empty() ||
      landmarks.empty() || depth.empty() || depth.type() != CV_32FC1) {
    return estimate;
  }

  std::vector<double> log_ratios;
  const std::size_t feature_count =
      std::min(raw_packet.keypoints.size(), raw_packet.landmark_ids.size());
  log_ratios.reserve(feature_count);

  for (std::size_t i = 0u; i < feature_count; ++i) {
    const LandmarkId lmk_id = raw_packet.landmark_ids[i];
    if (lmk_id == -1) {
      continue;
    }
    const auto lmk_it = landmarks.find(lmk_id);
    if (lmk_it == landmarks.end()) {
      continue;
    }

    const Point3 landmark_cam = cam_T_smoother.transformFrom(lmk_it->second);
    const double landmark_depth = landmark_cam.z();
    if (!std::isfinite(landmark_depth) ||
        landmark_depth < params_.min_depth_m ||
        landmark_depth > params_.max_depth_m) {
      continue;
    }

    float da3_depth = 0.0f;
    if (!sampleDepthBilinear(depth,
                             raw_packet.valid_mask,
                             raw_packet.keypoints[i],
                             &da3_depth) ||
        !std::isfinite(da3_depth) || da3_depth <= 0.0f) {
      continue;
    }

    const double da3_depth_d = static_cast<double>(da3_depth);
    log_ratios.push_back(std::log(landmark_depth) - std::log(da3_depth_d));
  }

  estimate.candidate_pairs = log_ratios.size();
  if (log_ratios.size() < kMinPairs) {
    return estimate;
  }

  const double median_log_ratio = medianValue(log_ratios);
  if (!std::isfinite(median_log_ratio)) {
    return estimate;
  }

  double sum_log_ratio = 0.0;
  for (const double log_ratio : log_ratios) {
    if (std::abs(log_ratio - median_log_ratio) >
        log_ratio_inlier_threshold) {
      continue;
    }
    sum_log_ratio += log_ratio;
    ++estimate.inlier_pairs;
  }

  if (estimate.inlier_pairs < kMinPairs) {
    return estimate;
  }

  const double log_scale =
      sum_log_ratio / static_cast<double>(estimate.inlier_pairs);
  const double scale = std::exp(log_scale);
  if (!std::isfinite(scale) || scale < kMinScale || scale > kMaxScale) {
    return estimate;
  }

  double sum_squared_log_error = 0.0;
  for (const double log_ratio : log_ratios) {
    if (std::abs(log_ratio - median_log_ratio) >
        log_ratio_inlier_threshold) {
      continue;
    }
    const double residual = log_scale - log_ratio;
    sum_squared_log_error += residual * residual;
  }

  estimate.scale = scale;
  estimate.log_rmse =
      std::sqrt(sum_squared_log_error /
                static_cast<double>(estimate.inlier_pairs));
  estimate.updated = true;
  return estimate;
}

MonoDepthMapOutput::ConstPtr MonoDepthAlignment::buildMapOutput(
    const MonoDepthRawPacket& raw_packet,
    const gtsam::Pose3& world_T_cam,
    const ScaleEstimate& scale_estimate) {
  if (scale_estimate.updated) {
    scale_ = scale_estimate.scale;
    scale_valid_ = true;
  }
  const double depth_scale = scale_valid_ ? scale_ : 1.0;

  const int stride = std::max(1, params_.point_stride);
  const int max_points = std::max(0, params_.max_points_per_keyframe);

  std::vector<Point3> candidate_points;
  std::vector<Eigen::Vector4f> candidate_colors;
  if (max_points > 0) {
    const int rows =
        std::min({raw_packet.depth.rows,
                  raw_packet.valid_mask.rows,
                  raw_packet.source_image_bgr.rows});
    const int cols =
        std::min({raw_packet.depth.cols,
                  raw_packet.valid_mask.cols,
                  raw_packet.source_image_bgr.cols});
    const std::size_t candidate_reserve =
        static_cast<std::size_t>(((rows + stride - 1) / stride) *
                                 ((cols + stride - 1) / stride));
    candidate_points.reserve(candidate_reserve);
    candidate_colors.reserve(candidate_reserve);

    const double fx = raw_packet.intrinsics.fx;
    const double fy = raw_packet.intrinsics.fy;
    const double cx = raw_packet.intrinsics.cx;
    const double cy = raw_packet.intrinsics.cy;
    if (fx <= 0.0 || fy <= 0.0) {
      LOG_EVERY_N(WARNING, 30)
          << "Skipping mono depth map backprojection because intrinsics are "
             "invalid.";
    } else {
      for (int v = 0; v < rows; v += stride) {
        const float* depth_row = raw_packet.depth.ptr<float>(v);
        const uint8_t* valid_row = raw_packet.valid_mask.ptr<uint8_t>(v);
        const cv::Vec3b* color_row =
            raw_packet.source_image_bgr.ptr<cv::Vec3b>(v);
        for (int u = 0; u < cols; u += stride) {
          if (valid_row[u] == 0u) {
            continue;
          }
          const float raw_z = depth_row[u];
          if (!std::isfinite(raw_z) || raw_z <= 0.0f) {
            continue;
          }
          const double z = static_cast<double>(raw_z) * depth_scale;
          if (!std::isfinite(z) || z < params_.min_depth_m ||
              z > params_.max_depth_m) {
            continue;
          }

          const double x = (static_cast<double>(u) - cx) * z / fx;
          const double y = (static_cast<double>(v) - cy) * z / fy;
          candidate_points.push_back(
              world_T_cam.transformFrom(Point3(x, y, z)));
          const cv::Vec3b& bgr = color_row[u];
          candidate_colors.emplace_back(static_cast<float>(bgr[2]),
                                        static_cast<float>(bgr[1]),
                                        static_cast<float>(bgr[0]),
                                        180.0f);
        }
      }
    }
  }

  const std::size_t candidate_count = candidate_points.size();
  std::vector<Point3> keyframe_points;
  std::vector<Eigen::Vector4f> keyframe_colors;
  if (static_cast<int>(candidate_count) <= max_points) {
    keyframe_points = std::move(candidate_points);
    keyframe_colors = std::move(candidate_colors);
  } else if (max_points > 0) {
    keyframe_points.reserve(static_cast<std::size_t>(max_points));
    keyframe_colors.reserve(static_cast<std::size_t>(max_points));
    for (int i = 0; i < max_points; ++i) {
      const std::size_t idx =
          std::min(candidate_count - 1,
                   (static_cast<std::size_t>(i) * candidate_count) /
                       static_cast<std::size_t>(max_points));
      keyframe_points.push_back(candidate_points[idx]);
      keyframe_colors.push_back(candidate_colors[idx]);
    }
  }

  accumulated_map_.insert(accumulated_map_.end(),
                          keyframe_points.begin(),
                          keyframe_points.end());
  accumulated_colors_.insert(accumulated_colors_.end(),
                             keyframe_colors.begin(),
                             keyframe_colors.end());

  auto output = std::make_shared<MonoDepthMapOutput>();
  output->target_frame_id = raw_packet.keyframe_id;
  output->target_timestamp = raw_packet.timestamp;
  output->scale = depth_scale;
  output->scale_log_rmse = scale_estimate.log_rmse;
  output->scale_candidate_pairs = scale_estimate.candidate_pairs;
  output->scale_inlier_pairs = scale_estimate.inlier_pairs;
  output->keyframe_cloud = std::move(keyframe_points);
  output->keyframe_colors = std::move(keyframe_colors);
  output->accumulated_map = accumulated_map_;
  output->accumulated_colors = accumulated_colors_;
  output->point_radius = params_.point_radius;

  LOG_EVERY_N(INFO, 10)
      << "Mono depth map points: " << accumulated_map_.size() << " ("
      << output->keyframe_cloud.size() << "/" << candidate_count
      << " keyframe points for delayed frame " << raw_packet.keyframe_id
      << "), scale: " << output->scale << " from "
      << output->scale_inlier_pairs << "/" << output->scale_candidate_pairs
      << " landmark depth pairs, log rmse: " << output->scale_log_rmse;

  return output;
}

}  // namespace VIO
