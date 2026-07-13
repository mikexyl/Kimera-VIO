#include "kimera-vio/backend/MonoDepthAlignment.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "kimera-vio/common/MonoDepthUtils.h"

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
    const gtsam::Values& state,
    const gtsam::Pose3& world_T_smoother,
    const MonoDepthICPOnlyResult* icp_only_result) {
  if (!params_.enabled) {
    return nullptr;
  }

  const std::vector<FrameId> smoother_frame_ids =
      findSmootherPoseFrameIds(state);
  if (smoother_frame_ids.empty()) {
    return nullptr;
  }
  pruneRawPacketCache(smoother_frame_ids);

  const FrameId oldest_frame_id = smoother_frame_ids.front();
  if (!last_oldest_frame_id_.has_value()) {
    last_oldest_frame_id_ = oldest_frame_id;
  } else if (*last_oldest_frame_id_ != oldest_frame_id) {
    last_oldest_frame_id_ = oldest_frame_id;
    if (!last_processed_frame_id_.has_value() ||
        *last_processed_frame_id_ != oldest_frame_id) {
      pending_insert_frame_id_ = oldest_frame_id;
    }
  }
  if (pending_insert_frame_id_.has_value() &&
      *pending_insert_frame_id_ != oldest_frame_id) {
    pending_insert_frame_id_.reset();
  }
  const std::optional<FrameId> insert_frame_id = pending_insert_frame_id_;

  MonoDepthMapOutput::ConstPtr output = buildMapOutput(
      smoother_frame_ids,
      state,
      world_T_smoother,
      insert_frame_id,
      icp_only_result);
  if (insert_frame_id.has_value() && output &&
      !output->keyframe_cloud.empty()) {
    last_processed_frame_id_ = *insert_frame_id;
    pending_insert_frame_id_.reset();
  }
  return output;
}

void MonoDepthAlignment::replaceRawPackets(
    const std::map<FrameId, MonoDepthRawPacket::ConstPtr>& raw_packets) {
  raw_packet_cache_ = raw_packets;
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
        std::binary_search(
            smoother_frame_ids.begin(), smoother_frame_ids.end(), frame_id);
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

std::size_t MonoDepthAlignment::backprojectPacket(
    const MonoDepthRawPacket& raw_packet,
    const gtsam::Pose3& world_T_cam,
    Point3Vector* points,
    RgbaColorVector* colors,
    RgbaColorVector* weight_colors) const {
  CHECK(points != nullptr);
  CHECK(colors != nullptr);
  if (!raw_packet.source_image_is_undistorted) {
    LOG_EVERY_N(WARNING, 30)
        << "Skipping mono-depth map backprojection because the packet is not "
           "in undistorted pinhole geometry.";
    return 0u;
  }
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
  const int rows = std::min({raw_packet.depth.rows,
                             raw_packet.valid_mask.rows,
                             raw_packet.source_image_bgr.rows});
  const int cols = std::min({raw_packet.depth.cols,
                             raw_packet.valid_mask.cols,
                             raw_packet.source_image_bgr.cols});
  const std::size_t candidate_reserve = static_cast<std::size_t>(
      ((rows + stride - 1) / stride) * ((cols + stride - 1) / stride));
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
    for (int u = 0; u < cols; u += stride) {
      if (valid_row[u] == 0u) {
        continue;
      }
      const float raw_z = depth_row[u];
      if (!std::isfinite(raw_z) || raw_z <= 0.0f) {
        continue;
      }
      const double z = static_cast<double>(raw_z);
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
        const double weight = static_cast<double>(sampleMonoDepthWeight(
            raw_packet.weight_image, raw_packet.depth.size(), u, v));
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
      weight_colors->reserve(weight_colors->size() +
                             static_cast<std::size_t>(max_points));
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
    const gtsam::Pose3& world_T_smoother,
    const std::optional<FrameId>& insert_frame_id,
    const MonoDepthICPOnlyResult* icp_only_result) {
  auto output = std::make_shared<MonoDepthMapOutput>();
  output->point_radius = params_.point_radius;
  if (icp_only_result) {
    output->icp_only = *icp_only_result;
  }

  std::size_t window_candidate_points = 0u;
  std::size_t keyframe_candidate_points = 0u;
  bool has_window_packet = false;
  FrameId latest_window_frame_id = 0u;
  Timestamp latest_window_timestamp = 0u;

  for (const FrameId frame_id : smoother_frame_ids) {
    const auto raw_it = raw_packet_cache_.find(frame_id);
    if (raw_it == raw_packet_cache_.end() || !raw_it->second) {
      continue;
    }

    const MonoDepthRawPacket& packet = *raw_it->second;
    output->scale_alignments[frame_id] = packet.scale_alignment;
    if (packet.scale_alignment.valid) {
      ++output->valid_scale_alignment_packets;
    } else {
      ++output->rejected_scale_alignment_packets;
    }
    output->selected_scale_alignment_frame_id = frame_id;
    output->selected_scale_alignment = packet.scale_alignment;
    latest_window_frame_id = packet.keyframe_id;
    latest_window_timestamp = packet.timestamp;
    has_window_packet = true;

    const gtsam::Symbol pose_key(kPoseSymbolChar, frame_id);
    if (state.find(pose_key) == state.end()) {
      continue;
    }

    const gtsam::Pose3 smoother_T_body = state.at<Pose3>(pose_key);
    const gtsam::Pose3 smoother_T_cam =
        smoother_T_body.compose(packet.body_T_cam);
    const gtsam::Pose3 world_T_cam = world_T_smoother.compose(smoother_T_cam);

    Point3Vector frame_points;
    RgbaColorVector frame_colors;
    RgbaColorVector frame_weight_colors;
    const std::size_t candidate_points =
        backprojectPacket(packet,
                          world_T_cam,
                          &frame_points,
                          &frame_colors,
                          &frame_weight_colors);
    window_candidate_points += candidate_points;
    if (frame_points.empty()) {
      continue;
    }

    ++output->window_keyframes;

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

  // This is a diagnostic cloud, so retain a finite optimizer candidate even
  // when the post-update correspondence refresh makes the objective worse.
  // The validity scalar remains false in that case.
  if (output->icp_only.enabled && output->icp_only.solution_available) {
    for (const FrameId frame_id : smoother_frame_ids) {
      const auto raw_it = raw_packet_cache_.find(frame_id);
      const auto pose_it = output->icp_only.body_poses.find(frame_id);
      if (raw_it == raw_packet_cache_.end() || !raw_it->second ||
          pose_it == output->icp_only.body_poses.end()) {
        continue;
      }

      const MonoDepthRawPacket& packet = *raw_it->second;
      const gtsam::Pose3 smoother_T_cam =
          pose_it->second.compose(packet.body_T_cam);
      const gtsam::Pose3 world_T_cam =
          world_T_smoother.compose(smoother_T_cam);
      Point3Vector frame_points;
      RgbaColorVector frame_colors;
      backprojectPacket(
          packet, world_T_cam, &frame_points, &frame_colors, nullptr);
      if (frame_points.empty()) {
        continue;
      }
      ++output->icp_only_window_keyframes;
      output->icp_only_window_cloud.insert(
          output->icp_only_window_cloud.end(),
          std::make_move_iterator(frame_points.begin()),
          std::make_move_iterator(frame_points.end()));
      output->icp_only_window_colors.insert(
          output->icp_only_window_colors.end(),
          std::make_move_iterator(frame_colors.begin()),
          std::make_move_iterator(frame_colors.end()));
    }
  }

  if (output->keyframe_cloud.empty() && has_window_packet) {
    output->target_frame_id = latest_window_frame_id;
    output->target_timestamp = latest_window_timestamp;
  }

  if (output->scale_alignments.empty() && output->keyframe_cloud.empty() &&
      output->window_cloud.empty()) {
    return nullptr;
  }

  const MonoDepthScaleAlignmentResult& selected =
      output->selected_scale_alignment;
  VLOG(1) << "Mono depth window points: " << output->window_cloud.size() << "/"
          << window_candidate_points << " across " << output->window_keyframes
          << " smoother keyframes, delayed insert "
          << output->keyframe_cloud.size() << "/" << keyframe_candidate_points
          << ", scale alignment method: "
          << monoDepthScaleAlignmentMethodToString(selected.method)
          << ", selected frame: " << output->selected_scale_alignment_frame_id
          << ", valid: " << selected.valid
          << ", absolute scale: " << selected.absolute_scale << " from "
          << selected.inlier_count << "/" << selected.candidate_count
          << " candidates, log rmse: " << selected.log_rmse
          << ", valid/rejected window packets: "
          << output->valid_scale_alignment_packets << "/"
          << output->rejected_scale_alignment_packets
          << ", ICP-only valid/factors/poses: " << output->icp_only.valid << "/"
          << output->icp_only.factor_count << "/"
          << output->icp_only.pose_count;

  return output;
}

}  // namespace VIO
