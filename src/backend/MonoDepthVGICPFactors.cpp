#include "kimera-vio/backend/MonoDepthVGICPFactors.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_weighted_icp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <numeric>
#include <queue>

#include "kimera-vio/common/MonoDepthUtils.h"
#include "kimera-vio/utils/Timer.h"

namespace VIO {
namespace {

double elapsedMs(
    const std::chrono::high_resolution_clock::time_point& start_time) {
  return static_cast<double>(
             utils::Timer::toc<std::chrono::microseconds>(start_time).count()) /
         1000.0;
}

constexpr int kVgicpLogEveryN = 10;

bool isFinitePose(const gtsam::Pose3& pose) {
  return pose.rotation().matrix().allFinite() && pose.translation().allFinite();
}

gtsam::Pose3 scalePoseTranslation(const gtsam::Pose3& pose,
                                  const double scale) {
  return gtsam::Pose3(pose.rotation(), scale * pose.translation());
}

}  // namespace

MonoDepthVGICPFactors::MonoDepthVGICPFactors(
    const BackendParams& backend_params,
    const MonoDepthParams& mono_depth_params)
    : backend_params_(backend_params), mono_depth_params_(mono_depth_params) {
  CHECK(backend_params_.vgicp_factors_enabled_);
  CHECK_GT(backend_params_.vgicp_downsample_resolution_, 0.0);
  CHECK_GT(backend_params_.vgicp_voxel_resolution_, 0.0);
  CHECK_GT(backend_params_.vgicp_covariance_neighbors_, 0);
  CHECK_GT(backend_params_.vgicp_num_threads_, 0);
  CHECK_GT(backend_params_.vgicp_max_correspondence_distance_, 0.0);
  CHECK_GT(backend_params_.vgicp_factor_weight_, 0.0);
  CHECK_GT(backend_params_.vgicp_icp_only_max_iterations_, 0);
}

void MonoDepthVGICPFactors::addFactors(
    const MonoDepthRawPacket::ConstPtr& raw_packet,
    const gtsam::Values& state,
    const gtsam::Values& new_values,
    const FeatureTracks& feature_tracks,
    const gtsam::NonlinearFactorGraph& current_factors,
    gtsam::FactorIndices* delete_slots,
    gtsam::NonlinearFactorGraph* new_factors) {
  CHECK_NOTNULL(delete_slots);
  CHECK_NOTNULL(new_factors);
  if (!backend_params_.vgicp_factors_enabled_) {
    return;
  }

  if (backend_params_.vgicp_icp_only_da3_overlap_fusion_) {
    cacheDa3OverlapPair(raw_packet);
    pruneDa3OverlapWindow(collectActivePoseFrameIds(state, new_values));
    LOG_EVERY_N(INFO, kVgicpLogEveryN)
        << "DA3 overlap-only diagnostic: pairs="
        << da3_overlap_fusion_.pair_count
        << ", fused_views_total=" << da3_overlap_fusion_.fused_view_count
        << ", retained_views=" << da3_overlap_fusion_.window_views.size()
        << ", window_points=" << da3OverlapWindowPointCount()
        << ", component_resets=" << da3_overlap_fusion_.component_reset_count
        << ". No ICP factors were constructed or added.";
    return;
  }

  const auto total_tic = utils::Timer::tic();
  cacheRawPacket(raw_packet);

  const std::vector<FrameId> active_frame_ids =
      collectActivePoseFrameIds(state, new_values);
  if (active_frame_ids.empty()) {
    return;
  }
  pruneCaches(active_frame_ids);

  const PairTrackCounts shared_track_counts =
      countSharedTracks(active_frame_ids, feature_tracks);

  std::vector<CandidatePair> refresh_pairs;
  std::size_t stale_factor_slots = 0u;
  if (!changed_frame_ids_.empty()) {
    std::set<FramePair> refresh_pair_set;
    for (const FramePair& pair : accepted_factor_pairs_) {
      if (changed_frame_ids_.find(pair.first) == changed_frame_ids_.end() &&
          changed_frame_ids_.find(pair.second) == changed_frame_ids_.end()) {
        continue;
      }
      const auto count_it = shared_track_counts.find(pair);
      refresh_pairs.push_back(
          {pair,
           count_it == shared_track_counts.end() ? 0u : count_it->second});
      refresh_pair_set.insert(pair);
    }
    if (!backend_params_.vgicp_icp_only_enabled_) {
      stale_factor_slots = appendAcceptedFactorSlotsToDelete(
          current_factors, refresh_pair_set, delete_slots);
    }
  }

  const std::vector<CandidatePair> track_gated_pairs =
      selectCandidatePairs(active_frame_ids, shared_track_counts, false);
  std::size_t max_shared_tracks = 0u;
  for (const auto& pair_and_count : shared_track_counts) {
    max_shared_tracks = std::max(max_shared_tracks, pair_and_count.second);
  }

  std::set<FrameId> frames_to_preprocess;
  for (const CandidatePair& candidate : refresh_pairs) {
    frames_to_preprocess.insert(candidate.pair.first);
    frames_to_preprocess.insert(candidate.pair.second);
  }
  for (const CandidatePair& candidate : track_gated_pairs) {
    frames_to_preprocess.insert(candidate.pair.first);
    frames_to_preprocess.insert(candidate.pair.second);
  }
  for (const FrameId frame_id : frames_to_preprocess) {
    ensureDenseFrame(frame_id);
  }

  const std::vector<CandidatePair> candidate_pairs =
      track_gated_pairs.empty()
          ? std::vector<CandidatePair>{}
          : selectCandidatePairs(active_frame_ids, shared_track_counts, true);

  if (backend_params_.vgicp_icp_only_enabled_) {
    for (const CandidatePair& candidate : candidate_pairs) {
      pending_factor_pairs_.insert(candidate.pair);
    }
    LOG_EVERY_N(INFO, kVgicpLogEveryN)
        << "Mono-depth ICP-only graph discovery: active_frames="
        << active_frame_ids.size()
        << ", raw_packets=" << raw_packet_cache_.size()
        << ", cached_clouds=" << dense_frames_.size()
        << ", shared_track_pairs=" << shared_track_counts.size()
        << ", new_pairs=" << candidate_pairs.size()
        << ", accepted_pairs=" << accepted_factor_pairs_.size()
        << ", preprocess_ms=" << elapsedMs(total_tic)
        << ". No ICP factors were added to the VIO smoother.";
    return;
  }

  std::size_t added_factors = 0u;
  std::size_t refreshed_factors = 0u;
  std::size_t unavailable_refresh_pairs = 0u;
  const auto add_factor = [&](const CandidatePair& candidate,
                              const bool is_refresh) {
    const FrameId target_id = candidate.pair.first;
    const FrameId source_id = candidate.pair.second;
    const auto target_it = dense_frames_.find(target_id);
    const auto source_it = dense_frames_.find(source_id);
    if (target_it == dense_frames_.end() || source_it == dense_frames_.end() ||
        !hasUsableDenseFrame(target_id) || !hasUsableDenseFrame(source_id)) {
      return false;
    }
    const gtsam::NonlinearFactor::shared_ptr factor =
        makeMatchingFactor(candidate.pair);
    if (!factor) {
      return false;
    }
    new_factors->push_back(factor);
    const double information_scale =
        backend_params_.vgicp_factor_weight_ /
        static_cast<double>(source_it->second.cloud->size());
    if (!is_refresh) {
      pending_factor_pairs_.insert(candidate.pair);
    }
    ++added_factors;
    if (is_refresh) {
      ++refreshed_factors;
    }

    VLOG(1) << "Added mono-depth "
            << (backend_params_.vgicp_use_weighted_icp_factor_ ? "weighted ICP"
                                                               : "VGICP")
            << " factor x" << target_id << " -> x" << source_id << " with "
            << candidate.shared_tracks << " shared tracks, target points="
            << target_it->second.downsampled_points
            << ", source points=" << source_it->second.downsampled_points
            << ", source mean weight=" << source_it->second.mean_weight
            << ", total factor weight=" << backend_params_.vgicp_factor_weight_
            << ", information scale=" << information_scale
            << ", refreshed=" << is_refresh;
    return true;
  };

  for (const CandidatePair& candidate : refresh_pairs) {
    if (!add_factor(candidate, true)) {
      ++unavailable_refresh_pairs;
      VLOG(1) << "Removed stale mono-depth ICP factor x" << candidate.pair.first
              << " -> x" << candidate.pair.second
              << " without replacement because at least one refreshed cloud "
                 "is unavailable.";
    }
  }
  for (const CandidatePair& candidate : candidate_pairs) {
    add_factor(candidate, false);
  }

  const double total_ms = elapsedMs(total_tic);
  LOG_EVERY_N(INFO, kVgicpLogEveryN)
      << "Mono-depth VGICP factors: active_frames=" << active_frame_ids.size()
      << ", raw_packets=" << raw_packet_cache_.size()
      << ", cached_clouds=" << dense_frames_.size()
      << ", shared_track_pairs=" << shared_track_counts.size()
      << ", max_shared_tracks=" << max_shared_tracks
      << ", min_shared_tracks=" << backend_params_.vgicp_min_shared_tracks_
      << ", track_gated_pairs=" << track_gated_pairs.size()
      << ", candidate_pairs=" << candidate_pairs.size()
      << ", added_factors=" << added_factors
      << ", stale_factor_slots=" << stale_factor_slots
      << ", refreshed_factors=" << refreshed_factors
      << ", unavailable_refresh_pairs=" << unavailable_refresh_pairs
      << ", accepted_pairs=" << accepted_factor_pairs_.size()
      << ", preprocess_ms=" << total_ms;
}

void MonoDepthVGICPFactors::notifySmootherUpdateResult(
    const bool update_succeeded) {
  if (update_succeeded) {
    accepted_factor_pairs_.insert(pending_factor_pairs_.begin(),
                                  pending_factor_pairs_.end());
    changed_frame_ids_.clear();
  }
  pending_factor_pairs_.clear();
}

MonoDepthICPOnlyResult MonoDepthVGICPFactors::optimizeIcpOnly(
    const gtsam::Values& state) {
  MonoDepthICPOnlyResult result;
  result.enabled = backend_params_.vgicp_icp_only_enabled_;
  if (!result.enabled) {
    result.failure_reason = "ICP-only optimization is disabled";
    return result;
  }

  if (backend_params_.vgicp_icp_only_da3_overlap_fusion_) {
    const gtsam::Values no_new_values;
    pruneDa3OverlapWindow(
        collectActivePoseFrameIds(state, no_new_values));
    return makeDa3OverlapResult();
  }

  const auto total_tic = utils::Timer::tic();
  const gtsam::Values no_new_values;
  const std::vector<FrameId> active_frame_ids =
      collectActivePoseFrameIds(state, no_new_values);
  pruneCaches(active_frame_ids);
  for (const FrameId frame_id : active_frame_ids) {
    const gtsam::Symbol key(kPoseSymbolChar, frame_id);
    result.body_poses[frame_id] = state.at<gtsam::Pose3>(key);
  }

  std::vector<FramePair> usable_pairs;
  usable_pairs.reserve(accepted_factor_pairs_.size());
  std::map<FrameId, std::set<FrameId>> adjacency;
  for (const FramePair& pair : accepted_factor_pairs_) {
    if (!ensureDenseFrame(pair.first) || !ensureDenseFrame(pair.second) ||
        !hasUsableDenseFrame(pair.first) || !hasUsableDenseFrame(pair.second)) {
      continue;
    }
    const gtsam::Symbol target_key(kPoseSymbolChar, pair.first);
    const gtsam::Symbol source_key(kPoseSymbolChar, pair.second);
    if (!state.exists(target_key) || !state.exists(source_key)) {
      continue;
    }
    usable_pairs.push_back(pair);
    adjacency[pair.first].insert(pair.second);
    adjacency[pair.second].insert(pair.first);
  }

  result.factor_count = usable_pairs.size();
  result.pose_count = adjacency.size();
  if (usable_pairs.empty()) {
    result.failure_reason = "no usable accepted ICP pairs";
    result.optimization_ms = elapsedMs(total_tic);
    return result;
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  for (const auto& frame_and_neighbors : adjacency) {
    const gtsam::Symbol key(kPoseSymbolChar, frame_and_neighbors.first);
    initial_values.insert(key, state.at<gtsam::Pose3>(key));
  }
  for (const FramePair& pair : usable_pairs) {
    const gtsam::NonlinearFactor::shared_ptr factor = makeMatchingFactor(pair);
    if (!factor) {
      result.failure_reason = "failed to construct an accepted ICP factor";
      result.optimization_ms = elapsedMs(total_tic);
      return result;
    }
    graph.push_back(factor);
  }

  // Each connected component has an independent SE(3) gauge.  Pin its oldest
  // pose to the VIO initialization, while leaving every other pose constrained
  // only by ICP.
  const gtsam::SharedNoiseModel anchor_noise =
      gtsam::noiseModel::Isotropic::Sigma(6u, 1e-6);
  std::set<FrameId> visited;
  for (const auto& frame_and_neighbors : adjacency) {
    const FrameId seed = frame_and_neighbors.first;
    if (visited.find(seed) != visited.end()) {
      continue;
    }
    FrameId anchor_id = seed;
    std::queue<FrameId> frontier;
    frontier.push(seed);
    visited.insert(seed);
    while (!frontier.empty()) {
      const FrameId frame_id = frontier.front();
      frontier.pop();
      anchor_id = std::min(anchor_id, frame_id);
      for (const FrameId neighbor : adjacency.at(frame_id)) {
        if (visited.insert(neighbor).second) {
          frontier.push(neighbor);
        }
      }
    }
    const gtsam::Symbol anchor_key(kPoseSymbolChar, anchor_id);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        anchor_key, initial_values.at<gtsam::Pose3>(anchor_key), anchor_noise);
    ++result.anchor_count;
  }

  try {
    result.initial_error = graph.error(initial_values);
    gtsam::LevenbergMarquardtParams params =
        gtsam::LevenbergMarquardtParams::CeresDefaults();
    params.setMaxIterations(backend_params_.vgicp_icp_only_max_iterations_);
    params.setVerbosity("SILENT");
    params.setVerbosityLM("SILENT");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);
    const gtsam::Values optimized_values = optimizer.optimize();
    result.iterations = optimizer.iterations();
    result.final_error = graph.error(optimized_values);
    result.error_ratio = result.initial_error > 1e-12
                             ? result.final_error / result.initial_error
                             : 1.0;

    constexpr double kRadiansToDegrees =
        180.0 / 3.141592653589793238462643383279502884;
    for (const auto& frame_and_neighbors : adjacency) {
      const FrameId frame_id = frame_and_neighbors.first;
      const gtsam::Symbol key(kPoseSymbolChar, frame_id);
      const gtsam::Pose3& initial_pose = initial_values.at<gtsam::Pose3>(key);
      const gtsam::Pose3& optimized_pose =
          optimized_values.at<gtsam::Pose3>(key);
      const gtsam::Pose3 correction = initial_pose.between(optimized_pose);
      result.max_translation_delta_m = std::max(
          result.max_translation_delta_m, correction.translation().norm());
      result.max_rotation_delta_deg =
          std::max(result.max_rotation_delta_deg,
                   gtsam::Rot3::Logmap(correction.rotation()).norm() *
                       kRadiansToDegrees);
      result.body_poses[frame_id] = optimized_pose;
    }

    result.solution_available = true;
    result.valid = std::isfinite(result.initial_error) &&
                   std::isfinite(result.final_error) &&
                   std::isfinite(result.error_ratio) &&
                   result.final_error <= result.initial_error + 1e-9;
    if (!result.valid) {
      result.failure_reason =
          "ICP-only optimizer produced a non-finite or increasing objective";
    }
  } catch (const std::exception& exception) {
    result.failure_reason = exception.what();
  }
  result.optimization_ms = elapsedMs(total_tic);

  LOG_EVERY_N(INFO, kVgicpLogEveryN)
      << "Mono-depth ICP-only optimization: valid=" << result.valid
      << ", solution_available=" << result.solution_available
      << ", factors=" << result.factor_count << ", poses=" << result.pose_count
      << ", anchors=" << result.anchor_count
      << ", iterations=" << result.iterations
      << ", error=" << result.initial_error << " -> " << result.final_error
      << ", ratio=" << result.error_ratio
      << ", max translation correction=" << result.max_translation_delta_m
      << " m"
      << ", max rotation correction=" << result.max_rotation_delta_deg
      << " deg, optimization_ms=" << result.optimization_ms
      << (result.failure_reason.empty() ? std::string()
                                        : ", failure=" + result.failure_reason);
  return result;
}

void MonoDepthVGICPFactors::cacheDa3OverlapPair(
    const MonoDepthRawPacket::ConstPtr& raw_packet) {
  if (!raw_packet || !raw_packet->da3_context_packet ||
      !raw_packet->da3_context_keyframe_id.has_value() ||
      !raw_packet->da3_context_cam_T_current_cam.has_value()) {
    return;
  }

  const MonoDepthRawPacket::ConstPtr& context_packet =
      raw_packet->da3_context_packet;
  const FramePair pair{*raw_packet->da3_context_keyframe_id,
                       raw_packet->keyframe_id};
  if (context_packet->keyframe_id != pair.first || pair.first == pair.second) {
    LOG(ERROR) << "Rejecting malformed DA3 overlap pair [" << pair.first << ", "
               << pair.second << "]";
    return;
  }
  if (da3_overlap_fusion_.initialized &&
      da3_overlap_fusion_.last_pair == pair) {
    return;
  }
  if (!isFinitePose(*raw_packet->da3_context_cam_T_current_cam)) {
    LOG(ERROR) << "Rejecting DA3 overlap pair [" << pair.first << ", "
               << pair.second << "] because its predicted pose is invalid.";
    return;
  }

  bool continue_component =
      da3_overlap_fusion_.initialized &&
      da3_overlap_fusion_.last_current_frame_id == pair.first &&
      da3_overlap_fusion_.last_current_packet;
  Da3OverlapScaleEstimate overlap;
  if (continue_component) {
    overlap = VIO::estimateDa3OverlapScale(
        *da3_overlap_fusion_.last_current_packet,
        *context_packet,
        mono_depth_params_.visualization_point_stride);
    continue_component = overlap.valid;
  }

  if (!continue_component && da3_overlap_fusion_.initialized) {
    const std::size_t reset_count =
        da3_overlap_fusion_.component_reset_count + 1u;
    LOG(WARNING) << "Resetting DA3 overlap component at pair [" << pair.first
                 << ", " << pair.second << "]"
                 << (overlap.failure_reason.empty()
                         ? ": pair is not consecutive"
                         : ": " + overlap.failure_reason);
    da3_overlap_fusion_ = Da3OverlapFusionState();
    da3_overlap_fusion_.component_reset_count = reset_count;
  }

  const double pair_scale =
      continue_component
          ? da3_overlap_fusion_.last_pair_scale * overlap.scale_ratio
          : 1.0;
  if (!std::isfinite(pair_scale) || pair_scale <= 0.0) {
    LOG(ERROR) << "Rejecting DA3 overlap pair [" << pair.first << ", "
               << pair.second << "] because its chained scale is invalid.";
    return;
  }

  const gtsam::Pose3 chain_T_context =
      continue_component ? da3_overlap_fusion_.chain_T_last_current
                         : gtsam::Pose3();
  const gtsam::Pose3 context_T_current = scalePoseTranslation(
      *raw_packet->da3_context_cam_T_current_cam, pair_scale);
  const gtsam::Pose3 chain_T_current =
      chain_T_context.compose(context_T_current);
  if (!isFinitePose(chain_T_current)) {
    LOG(ERROR) << "Rejecting DA3 overlap pair [" << pair.first << ", "
               << pair.second
               << "] because its chained camera pose is invalid.";
    return;
  }

  Da3OverlapViewCloud context_view;
  context_view.frame_id = pair.first;
  const std::size_t context_candidates = appendDa3OverlapView(
      *context_packet,
      pair_scale,
      chain_T_context,
      &context_view.points,
      &context_view.colors);
  Da3OverlapViewCloud current_view;
  current_view.frame_id = pair.second;
  const std::size_t current_candidates = appendDa3OverlapView(
      *raw_packet,
      pair_scale,
      chain_T_current,
      &current_view.points,
      &current_view.colors);
  if (context_candidates == 0u && current_candidates == 0u) {
    LOG(ERROR) << "Rejecting DA3 overlap pair [" << pair.first << ", "
               << pair.second << "] because neither view produced points.";
    return;
  }

  if (context_candidates > 0u) {
    da3_overlap_fusion_.window_views.push_back(std::move(context_view));
  }
  if (current_candidates > 0u) {
    da3_overlap_fusion_.window_views.push_back(std::move(current_view));
  }

  da3_overlap_fusion_.initialized = true;
  da3_overlap_fusion_.last_pair = pair;
  da3_overlap_fusion_.last_current_frame_id = pair.second;
  da3_overlap_fusion_.last_current_packet = raw_packet;
  da3_overlap_fusion_.chain_T_last_current = chain_T_current;
  da3_overlap_fusion_.last_pair_scale = pair_scale;
  ++da3_overlap_fusion_.pair_count;
  da3_overlap_fusion_.fused_view_count +=
      static_cast<std::size_t>(context_candidates > 0u) +
      static_cast<std::size_t>(current_candidates > 0u);
  da3_overlap_fusion_.last_overlap_candidate_count = overlap.candidate_count;
  da3_overlap_fusion_.last_overlap_inlier_count = overlap.inlier_count;
  da3_overlap_fusion_.last_overlap_log_rmse = overlap.log_rmse;

  LOG(INFO) << "DA3 overlap fused pair [" << pair.first << ", " << pair.second
            << "]: scale=" << pair_scale
            << (continue_component
                    ? ", overlap_ratio=" + std::to_string(overlap.scale_ratio)
                    : ", component_anchor=identity")
            << ", overlap_inliers=" << overlap.inlier_count << "/"
            << overlap.candidate_count
            << ", overlap_log_rmse=" << overlap.log_rmse << ", appended_views="
            << static_cast<std::size_t>(context_candidates > 0u) +
                   static_cast<std::size_t>(current_candidates > 0u)
            << ", retained_views=" << da3_overlap_fusion_.window_views.size()
            << ", window_points=" << da3OverlapWindowPointCount()
            << ". ICP disabled.";
}

std::size_t MonoDepthVGICPFactors::appendDa3OverlapView(
    const MonoDepthRawPacket& packet,
    const double scale,
    const gtsam::Pose3& chain_T_cam,
    Point3Vector* points,
    RgbaColorVector* colors) const {
  CHECK_NOTNULL(points);
  CHECK_NOTNULL(colors);
  if (!std::isfinite(scale) || scale <= 0.0 ||
      !packet.source_image_is_undistorted || packet.depth.empty() ||
      packet.depth.type() != CV_32FC1 || packet.depth_support_mask.empty() ||
      packet.depth_support_mask.type() != CV_8UC1 ||
      packet.source_image_bgr.empty() ||
      packet.source_image_bgr.type() != CV_8UC3) {
    return 0u;
  }
  const double fx = packet.intrinsics.fx;
  const double fy = packet.intrinsics.fy;
  const double cx = packet.intrinsics.cx;
  const double cy = packet.intrinsics.cy;
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
    return 0u;
  }
  const int max_points =
      std::max(0, mono_depth_params_.visualization_max_points_per_keyframe);
  if (max_points == 0) {
    return 0u;
  }
  const int stride = std::max(1, mono_depth_params_.visualization_point_stride);
  const int rows = std::min({packet.depth.rows,
                             packet.depth_support_mask.rows,
                             packet.source_image_bgr.rows});
  const int cols = std::min({packet.depth.cols,
                             packet.depth_support_mask.cols,
                             packet.source_image_bgr.cols});

  Point3Vector candidates;
  RgbaColorVector candidate_colors;
  candidates.reserve(static_cast<std::size_t>(((rows + stride - 1) / stride) *
                                              ((cols + stride - 1) / stride)));
  candidate_colors.reserve(candidates.capacity());
  for (int v = 0; v < rows; v += stride) {
    const float* depth_row = packet.depth.ptr<float>(v);
    const uint8_t* support_row = packet.depth_support_mask.ptr<uint8_t>(v);
    const cv::Vec3b* color_row = packet.source_image_bgr.ptr<cv::Vec3b>(v);
    for (int u = 0; u < cols; u += stride) {
      if (support_row[u] == 0u) {
        continue;
      }
      const double z = scale * static_cast<double>(depth_row[u]);
      if (!std::isfinite(z) || z <= 0.0) {
        continue;
      }
      const Point3 chain_point = chain_T_cam.transformFrom(
          Point3((static_cast<double>(u) - cx) * z / fx,
                 (static_cast<double>(v) - cy) * z / fy,
                 z));
      if (!chain_point.allFinite()) {
        continue;
      }
      candidates.push_back(chain_point);
      const cv::Vec3b& bgr = color_row[u];
      candidate_colors.emplace_back(static_cast<float>(bgr[2]),
                                    static_cast<float>(bgr[1]),
                                    static_cast<float>(bgr[0]),
                                    180.0f);
    }
  }

  const std::size_t candidate_count = candidates.size();
  const std::size_t selected_count =
      std::min(candidate_count, static_cast<std::size_t>(max_points));
  points->reserve(points->size() + selected_count);
  colors->reserve(colors->size() + selected_count);
  for (std::size_t i = 0u; i < selected_count; ++i) {
    const std::size_t index =
        selected_count == candidate_count
            ? i
            : std::min(candidate_count - 1u,
                       (i * candidate_count) / selected_count);
    points->push_back(candidates[index]);
    colors->push_back(candidate_colors[index]);
  }
  return candidate_count;
}

MonoDepthICPOnlyResult MonoDepthVGICPFactors::makeDa3OverlapResult() const {
  MonoDepthICPOnlyResult result;
  result.enabled = backend_params_.vgicp_icp_only_enabled_;
  result.da3_overlap_fusion = true;
  const std::size_t window_point_count = da3OverlapWindowPointCount();
  result.solution_available =
      da3_overlap_fusion_.initialized && window_point_count > 0u;
  result.valid = result.solution_available;
  result.failure_reason = result.valid
                              ? std::string()
                              : "no chained DA3 overlap component is available";
  result.factor_count = 0u;
  std::set<FrameId> retained_keyframe_ids;
  for (const Da3OverlapViewCloud& view :
       da3_overlap_fusion_.window_views) {
    retained_keyframe_ids.insert(view.frame_id);
  }
  result.pose_count = retained_keyframe_ids.size();
  result.anchor_count = result.solution_available ? 1u : 0u;
  result.iterations = 0u;
  result.initial_error = 0.0;
  result.final_error = 0.0;
  result.error_ratio = 1.0;
  result.da3_pair_count = da3_overlap_fusion_.pair_count;
  result.da3_fused_view_count = da3_overlap_fusion_.fused_view_count;
  result.da3_retained_view_count = da3_overlap_fusion_.window_views.size();
  result.da3_retained_keyframe_count = retained_keyframe_ids.size();
  result.da3_component_reset_count = da3_overlap_fusion_.component_reset_count;
  result.da3_overlap_candidate_count =
      da3_overlap_fusion_.last_overlap_candidate_count;
  result.da3_overlap_inlier_count =
      da3_overlap_fusion_.last_overlap_inlier_count;
  result.da3_overlap_log_rmse = da3_overlap_fusion_.last_overlap_log_rmse;
  result.da3_last_pair_scale = da3_overlap_fusion_.last_pair_scale;
  result.da3_overlap_cloud.reserve(window_point_count);
  result.da3_overlap_colors.reserve(window_point_count);
  for (const Da3OverlapViewCloud& view :
       da3_overlap_fusion_.window_views) {
    result.da3_overlap_cloud.insert(result.da3_overlap_cloud.end(),
                                    view.points.begin(),
                                    view.points.end());
    result.da3_overlap_colors.insert(result.da3_overlap_colors.end(),
                                     view.colors.begin(),
                                     view.colors.end());
  }
  return result;
}

void MonoDepthVGICPFactors::pruneDa3OverlapWindow(
    const std::vector<FrameId>& active_frame_ids) {
  // An empty key set is useful in pose-independence unit tests and does not
  // provide a meaningful smoother window to prune against. Production
  // smoother updates always carry active pose keys.
  if (active_frame_ids.empty()) {
    return;
  }
  const std::set<FrameId> active_frames(active_frame_ids.begin(),
                                        active_frame_ids.end());
  auto& views = da3_overlap_fusion_.window_views;
  views.erase(
      std::remove_if(
          views.begin(),
          views.end(),
          [&active_frames](const Da3OverlapViewCloud& view) {
            return active_frames.find(view.frame_id) == active_frames.end();
          }),
      views.end());
}

std::size_t MonoDepthVGICPFactors::da3OverlapWindowPointCount() const {
  return std::accumulate(
      da3_overlap_fusion_.window_views.begin(),
      da3_overlap_fusion_.window_views.end(),
      std::size_t{0u},
      [](const std::size_t count, const Da3OverlapViewCloud& view) {
        return count + view.points.size();
      });
}

void MonoDepthVGICPFactors::replaceRawPackets(
    const std::map<FrameId, MonoDepthRawPacket::ConstPtr>& raw_packets) {
  std::set<FrameId> changed_frame_ids;
  for (const auto& frame_and_packet : raw_packet_cache_) {
    const auto replacement_it = raw_packets.find(frame_and_packet.first);
    if (replacement_it == raw_packets.end() ||
        replacement_it->second.get() != frame_and_packet.second.get()) {
      changed_frame_ids.insert(frame_and_packet.first);
    }
  }
  for (const auto& frame_and_packet : raw_packets) {
    const auto previous_it = raw_packet_cache_.find(frame_and_packet.first);
    if (previous_it == raw_packet_cache_.end() ||
        previous_it->second.get() != frame_and_packet.second.get()) {
      changed_frame_ids.insert(frame_and_packet.first);
    }
  }

  raw_packet_cache_ = raw_packets;
  for (const FrameId frame_id : changed_frame_ids) {
    dense_frames_.erase(frame_id);
  }
  changed_frame_ids_.insert(changed_frame_ids.begin(), changed_frame_ids.end());
  while (raw_packet_cache_.size() > kRawPacketCacheSize) {
    changed_frame_ids_.insert(raw_packet_cache_.begin()->first);
    dense_frames_.erase(raw_packet_cache_.begin()->first);
    raw_packet_cache_.erase(raw_packet_cache_.begin());
  }
}

void MonoDepthVGICPFactors::cacheRawPacket(
    const MonoDepthRawPacket::ConstPtr& raw_packet) {
  if (!raw_packet) {
    return;
  }

  const auto previous_it = raw_packet_cache_.find(raw_packet->keyframe_id);
  if (previous_it != raw_packet_cache_.end() &&
      previous_it->second.get() == raw_packet.get()) {
    return;
  }
  raw_packet_cache_[raw_packet->keyframe_id] = raw_packet;
  dense_frames_.erase(raw_packet->keyframe_id);
  changed_frame_ids_.insert(raw_packet->keyframe_id);
  while (raw_packet_cache_.size() > kRawPacketCacheSize) {
    dense_frames_.erase(raw_packet_cache_.begin()->first);
    raw_packet_cache_.erase(raw_packet_cache_.begin());
  }
}

std::vector<FrameId> MonoDepthVGICPFactors::collectActivePoseFrameIds(
    const gtsam::Values& state,
    const gtsam::Values& new_values) {
  std::vector<FrameId> frame_ids;
  const auto collect_from_values = [&frame_ids](const gtsam::Values& values) {
    for (const gtsam::Key key : values.keys()) {
      const gtsam::Symbol symbol(key);
      if (symbol.chr() == kPoseSymbolChar) {
        frame_ids.push_back(symbol.index());
      }
    }
  };
  collect_from_values(state);
  collect_from_values(new_values);
  std::sort(frame_ids.begin(), frame_ids.end());
  frame_ids.erase(std::unique(frame_ids.begin(), frame_ids.end()),
                  frame_ids.end());
  return frame_ids;
}

void MonoDepthVGICPFactors::pruneCaches(
    const std::vector<FrameId>& active_frame_ids) {
  const std::set<FrameId> active_frames(active_frame_ids.begin(),
                                        active_frame_ids.end());
  const auto frame_is_active = [&active_frames](const FrameId frame_id) {
    return active_frames.find(frame_id) != active_frames.end();
  };

  for (auto it = raw_packet_cache_.begin(); it != raw_packet_cache_.end();) {
    if (frame_is_active(it->first)) {
      ++it;
    } else {
      it = raw_packet_cache_.erase(it);
    }
  }
  for (auto it = dense_frames_.begin(); it != dense_frames_.end();) {
    if (frame_is_active(it->first)) {
      ++it;
    } else {
      it = dense_frames_.erase(it);
    }
  }

  const auto pair_is_active = [&frame_is_active](const FramePair& pair) {
    return frame_is_active(pair.first) && frame_is_active(pair.second);
  };
  for (auto it = accepted_factor_pairs_.begin();
       it != accepted_factor_pairs_.end();) {
    if (pair_is_active(*it)) {
      ++it;
    } else {
      it = accepted_factor_pairs_.erase(it);
    }
  }
  for (auto it = pending_factor_pairs_.begin();
       it != pending_factor_pairs_.end();) {
    if (pair_is_active(*it)) {
      ++it;
    } else {
      it = pending_factor_pairs_.erase(it);
    }
  }
}

bool MonoDepthVGICPFactors::ensureDenseFrame(const FrameId& frame_id) {
  if (dense_frames_.find(frame_id) != dense_frames_.end()) {
    return true;
  }

  const auto raw_it = raw_packet_cache_.find(frame_id);
  if (raw_it == raw_packet_cache_.end() || !raw_it->second) {
    return false;
  }

  const auto total_tic = utils::Timer::tic();
  std::size_t candidate_points = 0u;
  std::size_t sampled_points = 0u;
  std::shared_ptr<gtsam_points::PointCloudCPU> cloud =
      buildBodyFrameCloud(*raw_it->second, &candidate_points, &sampled_points);
  if (!cloud || cloud->size() == 0u) {
    VLOG(1) << "Skipping mono-depth VGICP cloud for keyframe " << frame_id
            << ": no valid depth points.";
    return false;
  }

  double covariance_ms = 0.0;
  double voxel_ms = 0.0;
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> voxelmap = nullptr;
  if (!backend_params_.vgicp_use_weighted_icp_factor_) {
    const auto cov_tic = utils::Timer::tic();
    const auto covariances = gtsam_points::estimate_covariances(
        *cloud,
        backend_params_.vgicp_covariance_neighbors_,
        backend_params_.vgicp_num_threads_);
    cloud->add_covs(covariances);
    covariance_ms = elapsedMs(cov_tic);

    const auto voxel_tic = utils::Timer::tic();
    voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(
        backend_params_.vgicp_voxel_resolution_);
    voxelmap->insert(*cloud);
    voxel_ms = elapsedMs(voxel_tic);
  }

  DenseFrame dense_frame;
  dense_frame.cloud = cloud;
  dense_frame.voxelmap = voxelmap;
  dense_frame.candidate_points = candidate_points;
  dense_frame.sampled_points = sampled_points;
  dense_frame.downsampled_points = cloud->size();
  dense_frame.voxels = voxelmap ? voxelmap->num_voxels() : 0u;
  if (cloud->has_intensities()) {
    for (std::size_t i = 0u; i < cloud->size(); ++i) {
      dense_frame.mean_weight += cloud->intensities[i];
    }
    dense_frame.mean_weight /= static_cast<double>(cloud->size());
  }
  dense_frames_[frame_id] = dense_frame;

  const double total_ms = elapsedMs(total_tic);
  LOG_EVERY_N(INFO, kVgicpLogEveryN)
      << "Cached mono-depth VGICP cloud: keyframe_id=" << frame_id
      << ", scale_alignment_method="
      << monoDepthScaleAlignmentMethodToString(
             raw_it->second->scale_alignment.method)
      << ", absolute_scale=" << raw_it->second->scale_alignment.absolute_scale
      << ", candidate_points=" << candidate_points
      << ", sampled_points=" << sampled_points
      << ", downsampled_points=" << cloud->size()
      << ", mean_weight=" << dense_frame.mean_weight
      << ", voxels=" << dense_frame.voxels
      << ", covariance_ms=" << covariance_ms << ", voxel_ms=" << voxel_ms
      << ", preprocess_ms=" << total_ms;
  return true;
}

std::shared_ptr<gtsam_points::PointCloudCPU>
MonoDepthVGICPFactors::buildBodyFrameCloud(const MonoDepthRawPacket& raw_packet,
                                           std::size_t* candidate_points,
                                           std::size_t* sampled_points) const {
  CHECK_NOTNULL(candidate_points);
  CHECK_NOTNULL(sampled_points);
  *candidate_points = 0u;
  *sampled_points = 0u;

  if (!raw_packet.source_image_is_undistorted) {
    LOG_EVERY_N(WARNING, 30)
        << "Skipping mono-depth VGICP backprojection because the packet is "
           "not in undistorted pinhole geometry.";
    return nullptr;
  }
  if (raw_packet.depth.empty() || raw_packet.depth.type() != CV_32FC1 ||
      raw_packet.valid_mask.empty() ||
      raw_packet.valid_mask.type() != CV_8UC1) {
    return nullptr;
  }
  const double fx = raw_packet.intrinsics.fx;
  const double fy = raw_packet.intrinsics.fy;
  const double cx = raw_packet.intrinsics.cx;
  const double cy = raw_packet.intrinsics.cy;
  if (fx <= 0.0 || fy <= 0.0) {
    LOG_EVERY_N(WARNING, 30)
        << "Skipping mono-depth VGICP backprojection because intrinsics are "
           "invalid.";
    return nullptr;
  }

  const int stride = std::max(1, mono_depth_params_.point_stride);
  const int max_points =
      std::max(0, mono_depth_params_.max_points_per_keyframe);
  if (max_points == 0) {
    return nullptr;
  }

  int rows = std::min(raw_packet.depth.rows, raw_packet.valid_mask.rows);
  int cols = std::min(raw_packet.depth.cols, raw_packet.valid_mask.cols);

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      body_points;
  std::vector<double> intensities;
  const std::size_t candidate_reserve = static_cast<std::size_t>(
      ((rows + stride - 1) / stride) * ((cols + stride - 1) / stride));
  body_points.reserve(candidate_reserve);
  intensities.reserve(candidate_reserve);

  for (int v = 0; v < rows; v += stride) {
    const float* depth_row = raw_packet.depth.ptr<float>(v);
    const uint8_t* valid_row = raw_packet.valid_mask.ptr<uint8_t>(v);
    for (int u = 0; u < cols; u += stride) {
      if (valid_row[u] == 0u) {
        continue;
      }

      const float raw_z = depth_row[u];
      if (!std::isfinite(raw_z) || raw_z <= 0.0f) {
        continue;
      }

      const double z = static_cast<double>(raw_z);
      if (!std::isfinite(z) || z < mono_depth_params_.min_depth_m ||
          z > mono_depth_params_.max_depth_m) {
        continue;
      }

      const double x = (static_cast<double>(u) - cx) * z / fx;
      const double y = (static_cast<double>(v) - cy) * z / fy;
      const Point3 body_point =
          raw_packet.body_T_cam.transformFrom(Point3(x, y, z));
      if (!std::isfinite(body_point.x()) || !std::isfinite(body_point.y()) ||
          !std::isfinite(body_point.z())) {
        continue;
      }
      const double weight = static_cast<double>(sampleMonoDepthWeight(
          raw_packet.weight_image, raw_packet.depth.size(), u, v));
      if (!std::isfinite(weight) || weight <= 0.0) {
        continue;
      }
      body_points.emplace_back(
          body_point.x(), body_point.y(), body_point.z(), 1.0);
      intensities.push_back(std::clamp(weight, 0.0, 1.0));
    }
  }

  *candidate_points = body_points.size();
  if (body_points.empty()) {
    return nullptr;
  }

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      sampled_body_points;
  std::vector<double> sampled_intensities;
  if (static_cast<int>(body_points.size()) <= max_points) {
    sampled_body_points = std::move(body_points);
    sampled_intensities = std::move(intensities);
  } else {
    sampled_body_points.reserve(static_cast<std::size_t>(max_points));
    sampled_intensities.reserve(static_cast<std::size_t>(max_points));
    for (int i = 0; i < max_points; ++i) {
      const std::size_t idx =
          std::min(body_points.size() - 1u,
                   (static_cast<std::size_t>(i) * body_points.size()) /
                       static_cast<std::size_t>(max_points));
      sampled_body_points.push_back(body_points[idx]);
      sampled_intensities.push_back(intensities[idx]);
    }
  }
  *sampled_points = sampled_body_points.size();

  auto raw_cloud =
      std::make_shared<gtsam_points::PointCloudCPU>(sampled_body_points);
  raw_cloud->add_intensities(sampled_intensities);

  std::shared_ptr<gtsam_points::PointCloudCPU> downsampled =
      gtsam_points::voxelgrid_sampling(
          raw_cloud,
          backend_params_.vgicp_downsample_resolution_,
          backend_params_.vgicp_num_threads_);
  if (!downsampled || downsampled->size() == 0u) {
    return nullptr;
  }
  return downsampled;
}

bool MonoDepthVGICPFactors::hasUsableDenseFrame(const FrameId& frame_id) const {
  const auto frame_it = dense_frames_.find(frame_id);
  if (frame_it == dense_frames_.end()) {
    return false;
  }
  const DenseFrame& dense_frame = frame_it->second;
  if (!dense_frame.cloud ||
      dense_frame.cloud->size() <
          static_cast<std::size_t>(
              backend_params_.vgicp_min_points_per_keyframe_)) {
    return false;
  }
  if (backend_params_.vgicp_use_weighted_icp_factor_) {
    return dense_frame.cloud->has_intensities();
  }
  return dense_frame.voxelmap && dense_frame.cloud->has_covs();
}

MonoDepthVGICPFactors::PairTrackCounts MonoDepthVGICPFactors::countSharedTracks(
    const std::vector<FrameId>& active_frame_ids,
    const FeatureTracks& feature_tracks) const {
  const std::set<FrameId> active_frames(active_frame_ids.begin(),
                                        active_frame_ids.end());
  PairTrackCounts shared_track_counts;

  for (const auto& track_entry : feature_tracks) {
    std::vector<FrameId> observed_active_frames;
    observed_active_frames.reserve(track_entry.second.obs_.size());
    for (const FeatureObs& observation : track_entry.second.obs_) {
      if (active_frames.find(observation.first) != active_frames.end()) {
        observed_active_frames.push_back(observation.first);
      }
    }

    std::sort(observed_active_frames.begin(), observed_active_frames.end());
    observed_active_frames.erase(std::unique(observed_active_frames.begin(),
                                             observed_active_frames.end()),
                                 observed_active_frames.end());
    if (observed_active_frames.size() < 2u) {
      continue;
    }

    for (std::size_t i = 0u; i + 1u < observed_active_frames.size(); ++i) {
      for (std::size_t j = i + 1u; j < observed_active_frames.size(); ++j) {
        ++shared_track_counts[orderedPair(observed_active_frames[i],
                                          observed_active_frames[j])];
      }
    }
  }

  return shared_track_counts;
}

std::vector<MonoDepthVGICPFactors::CandidatePair>
MonoDepthVGICPFactors::selectCandidatePairs(
    const std::vector<FrameId>& active_frame_ids,
    const PairTrackCounts& shared_track_counts,
    const bool require_dense_frames) const {
  if (backend_params_.vgicp_max_edges_per_keyframe_ <= 0) {
    return {};
  }

  const std::set<FrameId> active_frames(active_frame_ids.begin(),
                                        active_frame_ids.end());
  const auto pair_is_active = [&active_frames](const FramePair& pair) {
    return active_frames.find(pair.first) != active_frames.end() &&
           active_frames.find(pair.second) != active_frames.end();
  };

  std::map<FrameId, int> degree;
  for (const FramePair& pair : accepted_factor_pairs_) {
    if (pair_is_active(pair)) {
      ++degree[pair.first];
      ++degree[pair.second];
    }
  }
  for (const FramePair& pair : pending_factor_pairs_) {
    if (pair_is_active(pair)) {
      ++degree[pair.first];
      ++degree[pair.second];
    }
  }

  std::vector<CandidatePair> candidates;
  candidates.reserve(shared_track_counts.size());
  for (const auto& pair_and_count : shared_track_counts) {
    const FramePair& pair = pair_and_count.first;
    const std::size_t shared_tracks = pair_and_count.second;
    if (shared_tracks < static_cast<std::size_t>(
                            backend_params_.vgicp_min_shared_tracks_) ||
        isPairAlreadyTracked(pair) ||
        (require_dense_frames && (!hasUsableDenseFrame(pair.first) ||
                                  !hasUsableDenseFrame(pair.second)))) {
      continue;
    }
    candidates.push_back({pair, shared_tracks});
  }

  std::sort(candidates.begin(),
            candidates.end(),
            [](const CandidatePair& lhs, const CandidatePair& rhs) {
              if (lhs.shared_tracks != rhs.shared_tracks) {
                return lhs.shared_tracks > rhs.shared_tracks;
              }
              return lhs.pair < rhs.pair;
            });

  std::vector<CandidatePair> selected;
  selected.reserve(candidates.size());
  for (const CandidatePair& candidate : candidates) {
    int& degree_first = degree[candidate.pair.first];
    int& degree_second = degree[candidate.pair.second];
    if (degree_first >= backend_params_.vgicp_max_edges_per_keyframe_ ||
        degree_second >= backend_params_.vgicp_max_edges_per_keyframe_) {
      continue;
    }
    selected.push_back(candidate);
    ++degree_first;
    ++degree_second;
  }

  return selected;
}

bool MonoDepthVGICPFactors::isPairAlreadyTracked(const FramePair& pair) const {
  return accepted_factor_pairs_.find(pair) != accepted_factor_pairs_.end() ||
         pending_factor_pairs_.find(pair) != pending_factor_pairs_.end();
}

gtsam::NonlinearFactor::shared_ptr MonoDepthVGICPFactors::makeMatchingFactor(
    const FramePair& pair) const {
  const auto target_it = dense_frames_.find(pair.first);
  const auto source_it = dense_frames_.find(pair.second);
  if (target_it == dense_frames_.end() || source_it == dense_frames_.end() ||
      !hasUsableDenseFrame(pair.first) || !hasUsableDenseFrame(pair.second)) {
    return nullptr;
  }

  const gtsam::Symbol target_key(kPoseSymbolChar, pair.first);
  const gtsam::Symbol source_key(kPoseSymbolChar, pair.second);
  const double information_scale =
      backend_params_.vgicp_factor_weight_ /
      static_cast<double>(source_it->second.cloud->size());
  if (backend_params_.vgicp_use_weighted_icp_factor_) {
    auto factor = std::make_shared<gtsam_points::IntegratedWeightedICPFactor>(
        target_key,
        source_key,
        target_it->second.cloud,
        source_it->second.cloud);
    factor->set_num_threads(backend_params_.vgicp_num_threads_);
    factor->set_max_correspondence_distance(
        backend_params_.vgicp_max_correspondence_distance_);
    factor->set_information_scale(information_scale);
    return factor;
  }

  auto factor = std::make_shared<gtsam_points::IntegratedVGICPFactor>(
      target_key,
      source_key,
      target_it->second.voxelmap,
      source_it->second.cloud);
  factor->set_num_threads(backend_params_.vgicp_num_threads_);
  factor->set_fused_cov_cache_mode(gtsam_points::FusedCovCacheMode::COMPACT);
  factor->set_information_scale(information_scale);
  return factor;
}

std::size_t MonoDepthVGICPFactors::appendAcceptedFactorSlotsToDelete(
    const gtsam::NonlinearFactorGraph& current_factors,
    const std::set<FramePair>& refresh_pairs,
    gtsam::FactorIndices* delete_slots) const {
  CHECK_NOTNULL(delete_slots);
  std::size_t appended_slots = 0u;
  for (std::size_t slot = 0u; slot < current_factors.size(); ++slot) {
    if (!current_factors.exists(slot)) {
      continue;
    }
    const gtsam::NonlinearFactor::shared_ptr& factor = current_factors.at(slot);
    if (!factor ||
        (dynamic_cast<const gtsam_points::IntegratedVGICPFactor*>(
             factor.get()) == nullptr &&
         dynamic_cast<const gtsam_points::IntegratedWeightedICPFactor*>(
             factor.get()) == nullptr)) {
      continue;
    }

    const gtsam::KeyVector& keys = factor->keys();
    if (keys.size() != 2u) {
      continue;
    }
    const gtsam::Symbol first(keys[0]);
    const gtsam::Symbol second(keys[1]);
    if (first.chr() != kPoseSymbolChar || second.chr() != kPoseSymbolChar) {
      continue;
    }
    const FramePair pair = orderedPair(first.index(), second.index());
    if (refresh_pairs.find(pair) == refresh_pairs.end() ||
        std::find(delete_slots->begin(), delete_slots->end(), slot) !=
            delete_slots->end()) {
      continue;
    }
    delete_slots->push_back(slot);
    ++appended_slots;
  }
  return appended_slots;
}

MonoDepthVGICPFactors::FramePair MonoDepthVGICPFactors::orderedPair(
    const FrameId& frame_id_a,
    const FrameId& frame_id_b) {
  return frame_id_a < frame_id_b ? FramePair(frame_id_a, frame_id_b)
                                 : FramePair(frame_id_b, frame_id_a);
}

}  // namespace VIO
