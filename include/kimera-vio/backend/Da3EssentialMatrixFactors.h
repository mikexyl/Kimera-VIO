#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <optional>
#include <set>
#include <string>
#include <utility>

#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

struct Da3EssentialFactorAddResult {
  bool added = false;
  std::optional<std::pair<FrameId, FrameId>> pair;
  std::string diagnostic;
  double initial_rotation_residual_norm = 0.0;
  double initial_direction_residual_norm = 0.0;
};

/** Adds at most one camera-aware essential factor for each canonical DA3 pair.
 */
class Da3EssentialMatrixFactors {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(Da3EssentialMatrixFactors);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double kRotationSigmaDegrees = 10.0;
  static constexpr double kDirectionSigmaDegrees = 25.0;
  static constexpr double kHuberThreshold = 1.345;

  explicit Da3EssentialMatrixFactors(bool enabled);

  Da3EssentialFactorAddResult addFactor(
      const MonoDepthRawPacket::ConstPtr& raw_packet,
      const gtsam::Values& state,
      const gtsam::Values& new_values,
      gtsam::NonlinearFactorGraph* new_factors);

  void notifySmootherUpdateResult(bool update_succeeded);

  bool enabled() const { return enabled_; }
  std::size_t committedPairCount() const { return committed_pairs_.size(); }

 private:
  using FramePair = std::pair<FrameId, FrameId>;

  Da3EssentialFactorAddResult skip(const std::optional<FramePair>& pair,
                                   const std::string& diagnostic,
                                   bool warning = true) const;

  bool enabled_ = false;
  gtsam::SharedNoiseModel noise_model_;
  std::set<FramePair> committed_pairs_;
  std::set<FramePair> pending_pairs_;
};

}  // namespace VIO
