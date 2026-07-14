#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

using Da3FrameTriple = std::array<FrameId, 3>;

struct Da3BaselineRatioFactorAddResult {
  bool added = false;
  std::optional<Da3FrameTriple> triple;
  std::string diagnostic;
  double overlap_scale_ratio = 1.0;
  double first_da3_translation_norm = 0.0;
  double second_da3_translation_norm = 0.0;
  double measured_baseline_ratio = 0.0;
  std::size_t overlap_candidate_count = 0u;
  std::size_t overlap_inlier_count = 0u;
  double overlap_log_rmse = 0.0;
  double initial_log_ratio_residual = 0.0;
};

/** Adds one scale-free baseline-ratio factor for each consecutive DA3 triple.
 */
class Da3BaselineRatioFactors {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(Da3BaselineRatioFactors);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double kHuberThreshold = 1.345;

  Da3BaselineRatioFactors(bool enabled, double log_sigma, int point_stride);

  Da3BaselineRatioFactorAddResult addFactor(
      const MonoDepthRawPacket::ConstPtr& raw_packet,
      const gtsam::Values& state,
      const gtsam::Values& new_values,
      gtsam::NonlinearFactorGraph* new_factors);

  void notifySmootherUpdateResult(bool update_succeeded);

  bool enabled() const { return enabled_; }
  std::size_t committedTripleCount() const { return committed_triples_.size(); }

 private:
  using FramePair = std::pair<FrameId, FrameId>;

  struct CachedPair {
    FramePair ids;
    MonoDepthRawPacket::ConstPtr packet;
  };

  Da3BaselineRatioFactorAddResult skip(
      const std::optional<Da3FrameTriple>& triple,
      const std::string& diagnostic,
      bool warning = true) const;

  bool enabled_ = false;
  int point_stride_ = 1;
  gtsam::SharedNoiseModel noise_model_;
  std::optional<CachedPair> anchor_pair_;
  std::set<Da3FrameTriple> committed_triples_;
  std::set<Da3FrameTriple> pending_triples_;
  std::map<Da3FrameTriple, CachedPair> pending_predecessors_;
};

}  // namespace VIO
