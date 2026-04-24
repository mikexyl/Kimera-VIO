#include "kimera-vio/backend/CbsLocalBeliefCovariance.h"

#include <cbs/bpsam/bpsam.h>

namespace VIO {

std::optional<gtsam::Matrix> computePoseBeliefCovarianceWithBpsamSnapshot(
    const gtsam::NonlinearFactorGraph& local_graph,
    const gtsam::Values& local_values,
    gtsam::Key pose_key,
    const gtsam::ISAM2Params& isam2_params,
    char robot_id) {
  if (local_graph.empty() || !local_values.exists(pose_key)) {
    return std::nullopt;
  }

  cbs::BPSAM::Params bpsam_params;
  bpsam_params.robot_id = static_cast<cbs::AgentId>(robot_id);
  bpsam_params.sam_params_ = isam2_params;
  bpsam_params.enable_gkcm = false;
  bpsam_params.enable_belief_dcs = false;

  try {
    cbs::BPSAM bpsam_snapshot(bpsam_params);
    cbs::BPSAM::UpdateParams update_params;
    bpsam_snapshot.update(local_graph, local_values, update_params);
    bpsam_snapshot.setMarginalizationGraph(
        cbs::BPSAM::MarginalizationType::LOCAL);
    const gtsam::Matrix pose_cov = bpsam_snapshot.marginalCovariance(pose_key);
    if (pose_cov.rows() != 6 || pose_cov.cols() != 6 || !pose_cov.allFinite()) {
      return std::nullopt;
    }
    return pose_cov;
  } catch (...) {
    return std::nullopt;
  }
}

}  // namespace VIO
