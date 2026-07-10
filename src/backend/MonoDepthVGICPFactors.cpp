#include "kimera-vio/backend/MonoDepthVGICPFactors.h"

#include <glog/logging.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_weighted_icp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

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
}

void MonoDepthVGICPFactors::addFactors(
    const MonoDepthRawPacket::ConstPtr& raw_packet,
    const gtsam::Values& state,
    const gtsam::Values& new_values,
    const FeatureTracks& feature_tracks,
    gtsam::NonlinearFactorGraph* new_factors) {
  CHECK_NOTNULL(new_factors);
  if (!backend_params_.vgicp_factors_enabled_) {
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
  const std::vector<CandidatePair> track_gated_pairs =
      selectCandidatePairs(active_frame_ids, shared_track_counts, false);
  std::size_t max_shared_tracks = 0u;
  for (const auto& pair_and_count : shared_track_counts) {
    max_shared_tracks = std::max(max_shared_tracks, pair_and_count.second);
  }

  std::set<FrameId> frames_to_preprocess;
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

  std::size_t added_factors = 0u;
  for (const CandidatePair& candidate : candidate_pairs) {
    const FrameId target_id = candidate.pair.first;
    const FrameId source_id = candidate.pair.second;
    const auto target_it = dense_frames_.find(target_id);
    const auto source_it = dense_frames_.find(source_id);
    if (target_it == dense_frames_.end() || source_it == dense_frames_.end()) {
      continue;
    }

    if (backend_params_.vgicp_use_weighted_icp_factor_) {
      auto factor =
          std::make_shared<gtsam_points::IntegratedWeightedICPFactor>(
              gtsam::Symbol(kPoseSymbolChar, target_id),
              gtsam::Symbol(kPoseSymbolChar, source_id),
              target_it->second.cloud,
              source_it->second.cloud);
      factor->set_num_threads(backend_params_.vgicp_num_threads_);
      factor->set_max_correspondence_distance(
          backend_params_.vgicp_max_correspondence_distance_);
      new_factors->push_back(factor);
    } else {
      auto factor = std::make_shared<gtsam_points::IntegratedVGICPFactor>(
          gtsam::Symbol(kPoseSymbolChar, target_id),
          gtsam::Symbol(kPoseSymbolChar, source_id),
          target_it->second.voxelmap,
          source_it->second.cloud);
      factor->set_num_threads(backend_params_.vgicp_num_threads_);
      factor->set_fused_cov_cache_mode(
          gtsam_points::FusedCovCacheMode::COMPACT);
      new_factors->push_back(factor);
    }
    pending_factor_pairs_.insert(candidate.pair);
    ++added_factors;

    VLOG(1) << "Added mono-depth "
            << (backend_params_.vgicp_use_weighted_icp_factor_
                    ? "weighted ICP"
                    : "VGICP")
            << " factor x" << target_id << " -> x" << source_id << " with "
            << candidate.shared_tracks << " shared tracks, target points="
            << target_it->second.downsampled_points << ", source points="
            << source_it->second.downsampled_points
            << ", source mean weight=" << source_it->second.mean_weight;
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
      << ", accepted_pairs=" << accepted_factor_pairs_.size()
      << ", preprocess_ms=" << total_ms;
}

void MonoDepthVGICPFactors::notifySmootherUpdateResult(
    const bool update_succeeded) {
  if (update_succeeded) {
    accepted_factor_pairs_.insert(pending_factor_pairs_.begin(),
                                  pending_factor_pairs_.end());
  }
  pending_factor_pairs_.clear();
}

void MonoDepthVGICPFactors::replaceRawPackets(
    const std::map<FrameId, MonoDepthRawPacket::ConstPtr>& raw_packets) {
  raw_packet_cache_ = raw_packets;
  dense_frames_.clear();
  while (raw_packet_cache_.size() > kRawPacketCacheSize) {
    raw_packet_cache_.erase(raw_packet_cache_.begin());
  }
}

void MonoDepthVGICPFactors::cacheRawPacket(
    const MonoDepthRawPacket::ConstPtr& raw_packet) {
  if (!raw_packet) {
    return;
  }

  raw_packet_cache_[raw_packet->keyframe_id] = raw_packet;
  dense_frames_.erase(raw_packet->keyframe_id);
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
      << ", da3_pose_depth_scale="
      << raw_it->second->da3_pose_depth_scale
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
MonoDepthVGICPFactors::buildBodyFrameCloud(
    const MonoDepthRawPacket& raw_packet,
    std::size_t* candidate_points,
    std::size_t* sampled_points) const {
  CHECK_NOTNULL(candidate_points);
  CHECK_NOTNULL(sampled_points);
  *candidate_points = 0u;
  *sampled_points = 0u;

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
  const std::size_t candidate_reserve =
      static_cast<std::size_t>(((rows + stride - 1) / stride) *
                               ((cols + stride - 1) / stride));
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
      body_points.emplace_back(body_point.x(), body_point.y(), body_point.z(),
                               1.0);
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

bool MonoDepthVGICPFactors::hasUsableDenseFrame(
    const FrameId& frame_id) const {
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

MonoDepthVGICPFactors::PairTrackCounts
MonoDepthVGICPFactors::countSharedTracks(
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
    if (shared_tracks <
            static_cast<std::size_t>(
                backend_params_.vgicp_min_shared_tracks_) ||
        isPairAlreadyTracked(pair) ||
        (require_dense_frames &&
         (!hasUsableDenseFrame(pair.first) ||
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

bool MonoDepthVGICPFactors::isPairAlreadyTracked(
    const FramePair& pair) const {
  return accepted_factor_pairs_.find(pair) != accepted_factor_pairs_.end() ||
         pending_factor_pairs_.find(pair) != pending_factor_pairs_.end();
}

MonoDepthVGICPFactors::FramePair MonoDepthVGICPFactors::orderedPair(
    const FrameId& frame_id_a,
    const FrameId& frame_id_b) {
  return frame_id_a < frame_id_b ? FramePair(frame_id_a, frame_id_b)
                                 : FramePair(frame_id_b, frame_id_a);
}

}  // namespace VIO
