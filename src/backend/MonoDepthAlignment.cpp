#include "kimera-vio/backend/MonoDepthAlignment.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace VIO {
namespace {

Eigen::Vector4f weightToColor(const double weight) {
  const double w = std::clamp(weight, 0.0, 1.0);
  const float low = static_cast<float>(255.0 * (1.0 - w));
  const float high = static_cast<float>(255.0 * w);
  return Eigen::Vector4f(low, high, 0.0f, 210.0f);
}

}  // namespace

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

  const std::vector<FrameId> smoother_frame_ids =
      findSmootherPoseFrameIds(state);
  if (smoother_frame_ids.empty()) {
    return nullptr;
  }
  pruneRawPacketCache(smoother_frame_ids);

  const FrameId oldest_frame_id = smoother_frame_ids.front();
  std::optional<FrameId> insert_frame_id = std::nullopt;
  if (!last_oldest_frame_id_.has_value()) {
    last_oldest_frame_id_ = oldest_frame_id;
  } else if (*last_oldest_frame_id_ != oldest_frame_id) {
    last_oldest_frame_id_ = oldest_frame_id;
    if (!last_processed_frame_id_.has_value() ||
        *last_processed_frame_id_ != oldest_frame_id) {
      insert_frame_id = oldest_frame_id;
    }
  }

  MonoDepthMapOutput::ConstPtr output = buildMapOutput(smoother_frame_ids,
                                                       state,
                                                       landmarks,
                                                       world_T_smoother,
                                                       insert_frame_id);
  if (insert_frame_id.has_value() && output &&
      !output->keyframe_cloud.empty()) {
    last_processed_frame_id_ = *insert_frame_id;
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

std::vector<FrameId> MonoDepthAlignment::findSmootherPoseFrameIds(
    const gtsam::Values& state) {
  std::vector<FrameId> frame_ids;
  for (auto key : state.keys()) {
    const gtsam::Symbol symbol(key);
    if (symbol.chr() != kPoseSymbolChar) {
      continue;
    }
    frame_ids.push_back(symbol.index());
  }
  std::sort(frame_ids.begin(), frame_ids.end());
  frame_ids.erase(std::unique(frame_ids.begin(), frame_ids.end()),
                  frame_ids.end());
  return frame_ids;
}

void MonoDepthAlignment::pruneRawPacketCache(
    const std::vector<FrameId>& smoother_frame_ids) {
  if (smoother_frame_ids.empty()) {
    return;
  }

  const FrameId oldest_frame_id = smoother_frame_ids.front();
  const FrameId newest_frame_id = smoother_frame_ids.back();
  for (auto it = raw_packet_cache_.begin(); it != raw_packet_cache_.end();) {
    const FrameId frame_id = it->first;
    const bool inside_smoother_range =
        frame_id >= oldest_frame_id && frame_id <= newest_frame_id;
    const bool in_smoother_window =
        inside_smoother_range &&
        std::binary_search(smoother_frame_ids.begin(),
                           smoother_frame_ids.end(),
                           frame_id);
    const bool newer_than_smoother = frame_id > newest_frame_id;
    if (in_smoother_window || newer_than_smoother) {
      ++it;
    } else {
      it = raw_packet_cache_.erase(it);
    }
  }

  while (raw_packet_cache_.size() > kRawPacketCacheSize) {
    raw_packet_cache_.erase(raw_packet_cache_.begin());
  }
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

std::size_t MonoDepthAlignment::backprojectPacket(
    const MonoDepthRawPacket& raw_packet,
    const gtsam::Pose3& world_T_cam,
    double depth_scale,
    Point3Vector* points,
    RgbaColorVector* colors,
    RgbaColorVector* weight_colors) const {
  CHECK(points != nullptr);
  CHECK(colors != nullptr);
  if (raw_packet.depth.empty() || raw_packet.depth.type() != CV_32FC1 ||
      raw_packet.valid_mask.empty() ||
      raw_packet.valid_mask.type() != CV_8UC1 ||
      raw_packet.source_image_bgr.empty()) {
    return 0u;
  }

  const int stride = std::max(1, params_.visualization_point_stride);
  const int max_points =
      std::max(0, params_.visualization_max_points_per_keyframe);
  if (max_points == 0) {
    return 0u;
  }

  Point3Vector candidate_points;
  RgbaColorVector candidate_colors;
  RgbaColorVector candidate_weight_colors;
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
  if (params_.visualize_weights) {
    candidate_weight_colors.reserve(candidate_reserve);
  }

  const double fx = raw_packet.intrinsics.fx;
  const double fy = raw_packet.intrinsics.fy;
  const double cx = raw_packet.intrinsics.cx;
  const double cy = raw_packet.intrinsics.cy;
  if (fx <= 0.0 || fy <= 0.0) {
    LOG_EVERY_N(WARNING, 30)
        << "Skipping mono depth map backprojection because intrinsics are "
           "invalid.";
    return 0u;
  }

  for (int v = 0; v < rows; v += stride) {
    const float* depth_row = raw_packet.depth.ptr<float>(v);
    const uint8_t* valid_row = raw_packet.valid_mask.ptr<uint8_t>(v);
    const cv::Vec3b* color_row = raw_packet.source_image_bgr.ptr<cv::Vec3b>(v);
    const float* weight_row =
        !raw_packet.weight_image.empty() &&
                raw_packet.weight_image.type() == CV_32FC1 &&
                raw_packet.weight_image.rows > v
            ? raw_packet.weight_image.ptr<float>(v)
            : nullptr;
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
      candidate_points.push_back(world_T_cam.transformFrom(Point3(x, y, z)));
      const cv::Vec3b& bgr = color_row[u];
      candidate_colors.emplace_back(static_cast<float>(bgr[2]),
                                    static_cast<float>(bgr[1]),
                                    static_cast<float>(bgr[0]),
                                    180.0f);
      if (params_.visualize_weights) {
        const double weight =
            weight_row && raw_packet.weight_image.cols > u
                ? static_cast<double>(weight_row[u])
                : 1.0;
        candidate_weight_colors.emplace_back(weightToColor(weight));
      }
    }
  }

  const std::size_t candidate_count = candidate_points.size();
  if (static_cast<int>(candidate_count) <= max_points) {
    points->insert(points->end(),
                   std::make_move_iterator(candidate_points.begin()),
                   std::make_move_iterator(candidate_points.end()));
    colors->insert(colors->end(),
                   std::make_move_iterator(candidate_colors.begin()),
                   std::make_move_iterator(candidate_colors.end()));
    if (weight_colors && params_.visualize_weights) {
      weight_colors->insert(
          weight_colors->end(),
          std::make_move_iterator(candidate_weight_colors.begin()),
          std::make_move_iterator(candidate_weight_colors.end()));
    }
  } else {
    points->reserve(points->size() + static_cast<std::size_t>(max_points));
    colors->reserve(colors->size() + static_cast<std::size_t>(max_points));
    if (weight_colors && params_.visualize_weights) {
      weight_colors->reserve(
          weight_colors->size() + static_cast<std::size_t>(max_points));
    }
    for (int i = 0; i < max_points; ++i) {
      const std::size_t idx =
          std::min(candidate_count - 1,
                   (static_cast<std::size_t>(i) * candidate_count) /
                       static_cast<std::size_t>(max_points));
      points->push_back(candidate_points[idx]);
      colors->push_back(candidate_colors[idx]);
      if (weight_colors && params_.visualize_weights) {
        weight_colors->push_back(candidate_weight_colors[idx]);
      }
    }
  }
  return candidate_count;
}

MonoDepthMapOutput::ConstPtr MonoDepthAlignment::buildMapOutput(
    const std::vector<FrameId>& smoother_frame_ids,
    const gtsam::Values& state,
    const PointsWithIdMap& landmarks,
    const gtsam::Pose3& world_T_smoother,
    const std::optional<FrameId>& insert_frame_id) {
  ScaleEstimate scale_estimate;
  if (params_.align_scale_with_landmarks && insert_frame_id.has_value()) {
    const auto raw_it = raw_packet_cache_.find(*insert_frame_id);
    const gtsam::Symbol target_pose_key(kPoseSymbolChar, *insert_frame_id);
    if (raw_it != raw_packet_cache_.end() && raw_it->second &&
        state.find(target_pose_key) != state.end()) {
      const MonoDepthRawPacket& target_packet = *raw_it->second;
      const gtsam::Pose3 smoother_T_body = state.at<Pose3>(target_pose_key);
      const gtsam::Pose3 smoother_T_cam =
          smoother_T_body.compose(target_packet.body_T_cam);
      scale_estimate = estimateScale(target_packet,
                                     landmarks,
                                     target_packet.depth,
                                     smoother_T_cam.inverse());
      if (scale_estimate.updated) {
        scale_ = scale_estimate.scale;
        scale_valid_ = true;
      }
    }
  }
  const double depth_scale =
      params_.align_scale_with_landmarks && scale_valid_ ? scale_ : 1.0;

  auto output = std::make_shared<MonoDepthMapOutput>();
  output->scale = depth_scale;
  output->scale_log_rmse = scale_estimate.log_rmse;
  output->scale_candidate_pairs = scale_estimate.candidate_pairs;
  output->scale_inlier_pairs = scale_estimate.inlier_pairs;
  output->point_radius = params_.point_radius;

  std::size_t window_candidate_points = 0u;
  std::size_t keyframe_candidate_points = 0u;
  bool has_window_timestamp = false;
  FrameId latest_window_frame_id = 0u;
  Timestamp latest_window_timestamp = 0u;

  for (const FrameId frame_id : smoother_frame_ids) {
    const auto raw_it = raw_packet_cache_.find(frame_id);
    if (raw_it == raw_packet_cache_.end() || !raw_it->second) {
      continue;
    }

    const gtsam::Symbol pose_key(kPoseSymbolChar, frame_id);
    if (state.find(pose_key) == state.end()) {
      continue;
    }

    const MonoDepthRawPacket& packet = *raw_it->second;
    const gtsam::Pose3 smoother_T_body = state.at<Pose3>(pose_key);
    const gtsam::Pose3 smoother_T_cam =
        smoother_T_body.compose(packet.body_T_cam);
    const gtsam::Pose3 world_T_cam = world_T_smoother.compose(smoother_T_cam);

    Point3Vector frame_points;
    RgbaColorVector frame_colors;
    RgbaColorVector frame_weight_colors;
    const std::size_t candidate_points =
        backprojectPacket(packet, world_T_cam, depth_scale, &frame_points,
                          &frame_colors, &frame_weight_colors);
    window_candidate_points += candidate_points;
    if (frame_points.empty()) {
      continue;
    }

    ++output->window_keyframes;
    latest_window_frame_id = packet.keyframe_id;
    latest_window_timestamp = packet.timestamp;
    has_window_timestamp = true;

    if (insert_frame_id.has_value() && frame_id == *insert_frame_id) {
      output->target_frame_id = packet.keyframe_id;
      output->target_timestamp = packet.timestamp;
      output->keyframe_cloud = frame_points;
      output->keyframe_colors = frame_colors;
      keyframe_candidate_points = candidate_points;
    }

    output->window_cloud.insert(output->window_cloud.end(),
                                std::make_move_iterator(frame_points.begin()),
                                std::make_move_iterator(frame_points.end()));
    output->window_colors.insert(output->window_colors.end(),
                                 std::make_move_iterator(frame_colors.begin()),
                                 std::make_move_iterator(frame_colors.end()));
    if (params_.visualize_weights) {
      output->window_weight_colors.insert(
          output->window_weight_colors.end(),
          std::make_move_iterator(frame_weight_colors.begin()),
          std::make_move_iterator(frame_weight_colors.end()));
    }
  }

  if (output->keyframe_cloud.empty() && has_window_timestamp) {
    output->target_frame_id = latest_window_frame_id;
    output->target_timestamp = latest_window_timestamp;
  }

  if (output->keyframe_cloud.empty() && output->window_cloud.empty()) {
    return nullptr;
  }

  VLOG(1) << "Mono depth window points: " << output->window_cloud.size()
          << "/" << window_candidate_points << " across "
          << output->window_keyframes << " smoother keyframes, delayed insert "
          << output->keyframe_cloud.size() << "/" << keyframe_candidate_points
          << ", scale: " << output->scale << " from "
          << output->scale_inlier_pairs << "/" << output->scale_candidate_pairs
          << " landmark depth pairs, log rmse: " << output->scale_log_rmse;

  return output;
}

}  // namespace VIO
