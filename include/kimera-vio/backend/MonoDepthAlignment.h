#pragma once

#include <gtsam/nonlinear/Values.h>

#include <map>
#include <optional>

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

  MonoDepthMapOutput::ConstPtr process(
      const MonoDepthRawPacket::ConstPtr& raw_packet,
      const gtsam::Values& state,
      const PointsWithIdMap& landmarks,
      const gtsam::Pose3& world_T_smoother);

 private:
  struct ScaleEstimate {
    double scale = 1.0;
    double log_rmse = 0.0;
    std::size_t candidate_pairs = 0u;
    std::size_t inlier_pairs = 0u;
    bool updated = false;
  };

  void cacheRawPacket(const MonoDepthRawPacket::ConstPtr& raw_packet);

  static std::optional<FrameId> findOldestSmootherPoseFrameId(
      const gtsam::Values& state);

  static bool sampleDepthBilinear(const cv::Mat& depth,
                                  const cv::Mat& valid_mask,
                                  const cv::Point2f& px,
                                  float* sampled_depth);

  static double medianValue(std::vector<double> values);

  ScaleEstimate estimateScale(const MonoDepthRawPacket& raw_packet,
                              const PointsWithIdMap& landmarks,
                              const cv::Mat& depth,
                              const gtsam::Pose3& cam_T_smoother) const;

  MonoDepthMapOutput::ConstPtr buildMapOutput(
      const MonoDepthRawPacket& raw_packet,
      const gtsam::Pose3& world_T_cam,
      const ScaleEstimate& scale_estimate);

 private:
  MonoDepthParams params_;
  std::map<FrameId, MonoDepthRawPacket::ConstPtr> raw_packet_cache_;
  std::optional<FrameId> last_oldest_frame_id_;
  std::optional<FrameId> last_processed_frame_id_;
  double scale_ = 1.0;
  bool scale_valid_ = false;
  Point3Vector accumulated_map_;
  RgbaColorVector accumulated_colors_;

  static constexpr std::size_t kRawPacketCacheSize = 256u;
};

}  // namespace VIO
