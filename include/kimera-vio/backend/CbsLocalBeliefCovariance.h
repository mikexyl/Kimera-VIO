#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <optional>

namespace VIO {

std::optional<gtsam::Matrix> computePoseBeliefCovarianceWithBpsamSnapshot(
    const gtsam::NonlinearFactorGraph& local_graph,
    const gtsam::Values& local_values,
    gtsam::Key pose_key,
    const gtsam::ISAM2Params& isam2_params,
    char robot_id);

}  // namespace VIO
