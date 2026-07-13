#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <string>
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
                  const gtsam::NonlinearFactorGraph& current_factors,
                  gtsam::FactorIndices* delete_slots,
                  gtsam::NonlinearFactorGraph* new_factors);

  void replaceRawPackets(
      const std::map<FrameId, MonoDepthRawPacket::ConstPtr>& raw_packets);

  void notifySmootherUpdateResult(bool update_succeeded);

  /**
   * Optimize active poses using only the cached mono-depth ICP factors.
   * The returned poses are diagnostic and never modify the supplied state.
   */
  MonoDepthICPOnlyResult optimizeIcpOnly(const gtsam::Values& state);

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

  struct Da3OverlapScaleEstimate {
    bool valid = false;
    double scale_ratio = 1.0;
    std::size_t candidate_count = 0u;
    std::size_t inlier_count = 0u;
    double log_rmse = 0.0;
    std::string failure_reason;
  };

  struct Da3OverlapViewCloud {
    FrameId frame_id = 0u;
    Point3Vector points;
    RgbaColorVector colors;
  };

  struct Da3OverlapFusionState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool initialized = false;
    FramePair last_pair{0u, 0u};
    FrameId last_current_frame_id = 0u;
    MonoDepthRawPacket::ConstPtr last_current_packet;
    gtsam::Pose3 chain_T_last_current;
    double last_pair_scale = 1.0;
    std::size_t pair_count = 0u;
    std::size_t fused_view_count = 0u;
    std::size_t component_reset_count = 0u;
    std::size_t last_overlap_candidate_count = 0u;
    std::size_t last_overlap_inlier_count = 0u;
    double last_overlap_log_rmse = 0.0;
    std::vector<Da3OverlapViewCloud> window_views;
  };

  void cacheRawPacket(const MonoDepthRawPacket::ConstPtr& raw_packet);

  void cacheDa3OverlapPair(const MonoDepthRawPacket::ConstPtr& raw_packet);

  Da3OverlapScaleEstimate estimateDa3OverlapScale(
      const MonoDepthRawPacket& previous_current,
      const MonoDepthRawPacket& next_context) const;

  std::size_t appendDa3OverlapView(const MonoDepthRawPacket& packet,
                                   double scale,
                                   const gtsam::Pose3& chain_T_cam,
                                   Point3Vector* points,
                                   RgbaColorVector* colors) const;

  void pruneDa3OverlapWindow(
      const std::vector<FrameId>& active_frame_ids);

  std::size_t da3OverlapWindowPointCount() const;

  MonoDepthICPOnlyResult makeDa3OverlapResult() const;

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

  gtsam::NonlinearFactor::shared_ptr makeMatchingFactor(
      const FramePair& pair) const;

  std::size_t appendAcceptedFactorSlotsToDelete(
      const gtsam::NonlinearFactorGraph& current_factors,
      const std::set<FramePair>& refresh_pairs,
      gtsam::FactorIndices* delete_slots) const;

  static FramePair orderedPair(const FrameId& frame_id_a,
                               const FrameId& frame_id_b);

 private:
  const BackendParams& backend_params_;
  const MonoDepthParams& mono_depth_params_;
  std::map<FrameId, MonoDepthRawPacket::ConstPtr> raw_packet_cache_;
  std::map<FrameId, DenseFrame> dense_frames_;
  std::set<FramePair> accepted_factor_pairs_;
  std::set<FramePair> pending_factor_pairs_;
  std::set<FrameId> changed_frame_ids_;
  Da3OverlapFusionState da3_overlap_fusion_;

  static constexpr std::size_t kRawPacketCacheSize = 256u;
};

}  // namespace VIO
