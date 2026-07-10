#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/backend/VioBackendParams.h"
#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/utils/Macros.h"

namespace gtsam_points {
class GaussianVoxelMapCPU;
struct PointCloudCPU;
}  // namespace gtsam_points

namespace VIO {

class MonoDepthVGICPFactors {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDepthVGICPFactors);
  KIMERA_POINTER_TYPEDEFS(MonoDepthVGICPFactors);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonoDepthVGICPFactors(const BackendParams& backend_params,
                        const MonoDepthParams& mono_depth_params);
  ~MonoDepthVGICPFactors() = default;

  void addFactors(const MonoDepthRawPacket::ConstPtr& raw_packet,
                  const gtsam::Values& state,
                  const gtsam::Values& new_values,
                  const FeatureTracks& feature_tracks,
                  gtsam::NonlinearFactorGraph* new_factors);

  void notifySmootherUpdateResult(bool update_succeeded);

 private:
  struct DenseFrame {
    std::shared_ptr<gtsam_points::PointCloudCPU> cloud;
    std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> voxelmap;
    std::size_t candidate_points = 0u;
    std::size_t sampled_points = 0u;
    std::size_t downsampled_points = 0u;
    std::size_t voxels = 0u;
    double mean_weight = 0.0;
  };

  struct CandidatePair {
    std::pair<FrameId, FrameId> pair;
    std::size_t shared_tracks = 0u;
  };

  using FramePair = std::pair<FrameId, FrameId>;
  using PairTrackCounts = std::map<FramePair, std::size_t>;

  void cacheRawPacket(const MonoDepthRawPacket::ConstPtr& raw_packet);

  static std::vector<FrameId> collectActivePoseFrameIds(
      const gtsam::Values& state,
      const gtsam::Values& new_values);

  void pruneCaches(const std::vector<FrameId>& active_frame_ids);

  bool ensureDenseFrame(const FrameId& frame_id);

  std::shared_ptr<gtsam_points::PointCloudCPU> buildBodyFrameCloud(
      const MonoDepthRawPacket& raw_packet,
      std::size_t* candidate_points,
      std::size_t* sampled_points) const;

  bool hasUsableDenseFrame(const FrameId& frame_id) const;

  PairTrackCounts countSharedTracks(
      const std::vector<FrameId>& active_frame_ids,
      const FeatureTracks& feature_tracks) const;

  std::vector<CandidatePair> selectCandidatePairs(
      const std::vector<FrameId>& active_frame_ids,
      const PairTrackCounts& shared_track_counts,
      bool require_dense_frames) const;

  bool isPairAlreadyTracked(const FramePair& pair) const;

  static FramePair orderedPair(const FrameId& frame_id_a,
                               const FrameId& frame_id_b);

 private:
  const BackendParams& backend_params_;
  const MonoDepthParams& mono_depth_params_;
  std::map<FrameId, MonoDepthRawPacket::ConstPtr> raw_packet_cache_;
  std::map<FrameId, DenseFrame> dense_frames_;
  std::set<FramePair> accepted_factor_pairs_;
  std::set<FramePair> pending_factor_pairs_;

  static constexpr std::size_t kRawPacketCacheSize = 256u;
};

}  // namespace VIO
