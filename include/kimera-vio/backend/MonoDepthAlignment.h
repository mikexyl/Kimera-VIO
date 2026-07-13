#pragma once

#include <gtsam/nonlinear/Values.h>

#include <map>
#include <optional>
#include <vector>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MonoDepthAlignment {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDepthAlignment);
  KIMERA_POINTER_TYPEDEFS(MonoDepthAlignment);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MonoDepthAlignment(const MonoDepthParams& params);
  ~MonoDepthAlignment() = default;

  MonoDepthMapOutput::ConstPtr process(const gtsam::Values& state,
                                       const gtsam::Pose3& world_T_smoother,
                                       const MonoDepthICPOnlyResult*
                                           icp_only_result = nullptr);

  void replaceRawPackets(
      const std::map<FrameId, MonoDepthRawPacket::ConstPtr>& raw_packets);

 private:
  static std::vector<FrameId> findSmootherPoseFrameIds(
      const gtsam::Values& state);

  void pruneRawPacketCache(const std::vector<FrameId>& smoother_frame_ids);

  std::size_t backprojectPacket(const MonoDepthRawPacket& raw_packet,
                                const gtsam::Pose3& world_T_cam,
                                Point3Vector* points,
                                RgbaColorVector* colors,
                                RgbaColorVector* weight_colors) const;

  MonoDepthMapOutput::ConstPtr buildMapOutput(
      const std::vector<FrameId>& smoother_frame_ids,
      const gtsam::Values& state,
      const gtsam::Pose3& world_T_smoother,
      const std::optional<FrameId>& insert_frame_id,
      const MonoDepthICPOnlyResult* icp_only_result);

 private:
  MonoDepthParams params_;
  std::map<FrameId, MonoDepthRawPacket::ConstPtr> raw_packet_cache_;
  std::optional<FrameId> last_oldest_frame_id_;
  std::optional<FrameId> last_processed_frame_id_;
  std::optional<FrameId> pending_insert_frame_id_;

  static constexpr std::size_t kRawPacketCacheSize = 256u;
};

}  // namespace VIO
