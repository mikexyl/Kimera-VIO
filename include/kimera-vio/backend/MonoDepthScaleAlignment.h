#pragma once

#include <gtsam/geometry/Pose3.h>

#include <map>
#include <memory>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct MonoDepthScaleAlignmentInput {
  const MonoDepthRawPacket& canonical_packet;
  const std::map<FrameId, gtsam::Pose3>& optimized_body_poses;
  const PointsWithIdMap& optimized_landmarks;
};

class MonoDepthScaleAligner {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDepthScaleAligner);
  KIMERA_POINTER_TYPEDEFS(MonoDepthScaleAligner);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonoDepthScaleAligner() = default;
  virtual ~MonoDepthScaleAligner() = default;

  virtual MonoDepthScaleAlignmentMethod method() const = 0;

  virtual bool requiresOptimizedLandmarks() const { return false; }

  virtual MonoDepthScaleAlignmentResult align(
      const MonoDepthScaleAlignmentInput& input) const = 0;
};

MonoDepthScaleAligner::UniquePtr makeMonoDepthScaleAligner(
    const MonoDepthParams& params);

MonoDepthRawPacket::ConstPtr applyMonoDepthScaleAlignment(
    const MonoDepthRawPacket& canonical_packet,
    const MonoDepthScaleAlignmentResult& result);

}  // namespace VIO
