#include "kimera-vio/backend/Da3EssentialMatrixFactors.h"

#include <glog/logging.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>

#include <cmath>
#include <exception>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/factors/CameraAwareEssentialMatrixFactor.h"

namespace VIO {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kMinimumBaseline = 1e-9;

bool isFinitePose(const gtsam::Pose3& pose) {
  return pose.rotation().matrix().allFinite() && pose.translation().allFinite();
}

std::optional<gtsam::Pose3> findPose(const gtsam::Values& values,
                                     const gtsam::Key key) {
  const auto value = values.find(key);
  if (value == values.end()) {
    return std::nullopt;
  }
  try {
    return values.at<gtsam::Pose3>(key);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

}  // namespace

Da3EssentialMatrixFactors::Da3EssentialMatrixFactors(const bool enabled)
    : enabled_(enabled) {
  gtsam::Vector5 sigmas;
  sigmas.head<3>().setConstant(kRotationSigmaDegrees * kDegreesToRadians);
  sigmas.tail<2>().setConstant(kDirectionSigmaDegrees * kDegreesToRadians);
  const auto diagonal = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  noise_model_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(kHuberThreshold), diagonal);
}

Da3EssentialFactorAddResult Da3EssentialMatrixFactors::skip(
    const std::optional<FramePair>& pair,
    const std::string& diagnostic,
    const bool warning) const {
  Da3EssentialFactorAddResult result;
  result.pair = pair;
  result.diagnostic = diagnostic;
  if (warning) {
    if (pair) {
      LOG(WARNING) << "Skipping DA3 essential pair [" << pair->first << ", "
                   << pair->second << "]: " << diagnostic;
    } else {
      LOG(WARNING) << "Skipping DA3 essential factor: " << diagnostic;
    }
  } else {
    VLOG(2) << "Skipping DA3 essential factor: " << diagnostic;
  }
  return result;
}

Da3EssentialFactorAddResult Da3EssentialMatrixFactors::addFactor(
    const MonoDepthRawPacket::ConstPtr& raw_packet,
    const gtsam::Values& state,
    const gtsam::Values& new_values,
    gtsam::NonlinearFactorGraph* new_factors) {
  CHECK_NOTNULL(new_factors);
  if (!enabled_) {
    return skip(std::nullopt, "disabled", false);
  }
  if (!raw_packet) {
    return skip(std::nullopt, "no canonical DA3 packet", false);
  }

  const std::optional<FramePair> partial_pair =
      raw_packet->da3_context_keyframe_id
          ? std::make_optional(FramePair(*raw_packet->da3_context_keyframe_id,
                                         raw_packet->keyframe_id))
          : std::nullopt;
  if (!raw_packet->da3_context_keyframe_id ||
      !raw_packet->da3_context_body_T_cam ||
      !raw_packet->da3_context_cam_T_current_cam) {
    return skip(partial_pair, "required pair pose metadata is missing");
  }

  const FramePair pair(*raw_packet->da3_context_keyframe_id,
                       raw_packet->keyframe_id);
  if (pair.first >= pair.second) {
    return skip(pair, "pair IDs are malformed");
  }
  if (committed_pairs_.count(pair) || pending_pairs_.count(pair)) {
    return skip(pair, "duplicate pair");
  }
  if (!isFinitePose(*raw_packet->da3_context_body_T_cam) ||
      !isFinitePose(raw_packet->body_T_cam) ||
      !isFinitePose(*raw_packet->da3_context_cam_T_current_cam)) {
    return skip(pair, "pair pose metadata is non-finite");
  }
  const double measured_baseline =
      raw_packet->da3_context_cam_T_current_cam->translation().norm();
  if (!std::isfinite(measured_baseline) ||
      measured_baseline <= kMinimumBaseline) {
    return skip(pair, "measured camera baseline is invalid or zero");
  }

  const gtsam::Key context_key = gtsam::Symbol(kPoseSymbolChar, pair.first);
  const gtsam::Key current_key = gtsam::Symbol(kPoseSymbolChar, pair.second);
  const std::optional<gtsam::Pose3> context_body_pose =
      findPose(state, context_key);
  if (!context_body_pose) {
    return skip(pair, "context pose is outside the fixed-lag window");
  }
  std::optional<gtsam::Pose3> current_body_pose = findPose(state, current_key);
  if (!current_body_pose) {
    current_body_pose = findPose(new_values, current_key);
  }
  if (!current_body_pose) {
    return skip(pair, "current pose is unavailable");
  }

  try {
#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
    boost::shared_ptr<CameraAwareEssentialMatrixFactor> factor(
        new CameraAwareEssentialMatrixFactor(
            context_key,
            current_key,
            *raw_packet->da3_context_cam_T_current_cam,
            *raw_packet->da3_context_body_T_cam,
            raw_packet->body_T_cam,
            noise_model_));
#else
    auto factor = std::make_shared<CameraAwareEssentialMatrixFactor>(
        context_key,
        current_key,
        *raw_packet->da3_context_cam_T_current_cam,
        *raw_packet->da3_context_body_T_cam,
        raw_packet->body_T_cam,
        noise_model_);
#endif
    const gtsam::Vector5 initial_error =
        factor->evaluateError(*context_body_pose, *current_body_pose);
    new_factors->push_back(factor);
    pending_pairs_.insert(pair);

    Da3EssentialFactorAddResult result;
    result.added = true;
    result.pair = pair;
    result.diagnostic = "added";
    result.initial_rotation_residual_norm = initial_error.head<3>().norm();
    result.initial_direction_residual_norm = initial_error.tail<2>().norm();
    LOG(INFO) << "Added DA3 essential pair [" << pair.first << ", "
              << pair.second << "] with initial rotation residual "
              << result.initial_rotation_residual_norm
              << " rad and direction residual "
              << result.initial_direction_residual_norm << " rad";
    return result;
  } catch (const std::exception& e) {
    return skip(pair, std::string("invalid factor: ") + e.what());
  }
}

void Da3EssentialMatrixFactors::notifySmootherUpdateResult(
    const bool update_succeeded) {
  if (update_succeeded) {
    committed_pairs_.insert(pending_pairs_.begin(), pending_pairs_.end());
  }
  pending_pairs_.clear();
}

}  // namespace VIO
