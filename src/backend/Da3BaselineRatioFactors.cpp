#include "kimera-vio/backend/Da3BaselineRatioFactors.h"

#include <glog/logging.h>
#include <gtsam/nonlinear/Symbol.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/common/MonoDepthUtils.h"
#include "kimera-vio/factors/CameraAwareBaselineRatioFactor.h"

namespace VIO {
namespace {

constexpr double kMinimumBaseline = 1e-9;
constexpr double kExtrinsicTolerance = 1e-6;

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

std::string tripleString(const Da3FrameTriple& triple) {
  return "[" + std::to_string(triple[0]) + ", " + std::to_string(triple[1]) +
         ", " + std::to_string(triple[2]) + "]";
}

}  // namespace

Da3BaselineRatioFactors::Da3BaselineRatioFactors(const bool enabled,
                                                 const double log_sigma,
                                                 const int point_stride)
    : enabled_(enabled), point_stride_(std::max(1, point_stride)) {
  if (!std::isfinite(log_sigma) || log_sigma <= 0.0) {
    throw std::invalid_argument(
        "DA3 baseline-ratio log sigma must be finite and positive");
  }
  const auto isotropic = gtsam::noiseModel::Isotropic::Sigma(1u, log_sigma);
  noise_model_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(kHuberThreshold), isotropic);
}

Da3BaselineRatioFactorAddResult Da3BaselineRatioFactors::skip(
    const std::optional<Da3FrameTriple>& triple,
    const std::string& diagnostic,
    const bool warning) const {
  Da3BaselineRatioFactorAddResult result;
  result.triple = triple;
  result.diagnostic = diagnostic;
  if (warning) {
    if (triple) {
      LOG(WARNING) << "Skipping DA3 baseline-ratio triple "
                   << tripleString(*triple) << ": " << diagnostic;
    } else {
      LOG(WARNING) << "Skipping DA3 baseline-ratio factor: " << diagnostic;
    }
  } else {
    VLOG(2) << "Skipping DA3 baseline-ratio factor: " << diagnostic;
  }
  return result;
}

Da3BaselineRatioFactorAddResult Da3BaselineRatioFactors::addFactor(
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

  if (!raw_packet->da3_context_keyframe_id ||
      !raw_packet->da3_context_body_T_cam ||
      !raw_packet->da3_context_cam_T_current_cam) {
    return skip(std::nullopt, "required pair pose metadata is missing");
  }
  const FramePair pair(*raw_packet->da3_context_keyframe_id,
                       raw_packet->keyframe_id);
  if (pair.first >= pair.second) {
    return skip(std::nullopt, "pair IDs are malformed");
  }
  if (!isFinitePose(*raw_packet->da3_context_body_T_cam) ||
      !isFinitePose(raw_packet->body_T_cam) ||
      !isFinitePose(*raw_packet->da3_context_cam_T_current_cam)) {
    return skip(std::nullopt, "pair pose metadata is non-finite");
  }
  const double current_da3_baseline =
      raw_packet->da3_context_cam_T_current_cam->translation().norm();
  if (!std::isfinite(current_da3_baseline) ||
      current_da3_baseline <= kMinimumBaseline) {
    return skip(std::nullopt, "DA3 pair baseline is invalid or zero");
  }

  if (anchor_pair_ && anchor_pair_->ids == pair) {
    return skip(std::nullopt, "duplicate pair packet", false);
  }

  const CachedPair current_pair{pair, raw_packet};
  if (!anchor_pair_) {
    anchor_pair_ = current_pair;
    return skip(std::nullopt, "first valid DA3 pair anchors the chain", false);
  }

  const CachedPair previous_pair = *anchor_pair_;
  anchor_pair_ = current_pair;
  const Da3FrameTriple candidate_triple{
      previous_pair.ids.first, previous_pair.ids.second, pair.second};
  if (previous_pair.ids.second != pair.first) {
    return skip(candidate_triple,
                "pair chain is not consecutive; re-anchored at newest pair");
  }
  const Da3FrameTriple triple{previous_pair.ids.first, pair.first, pair.second};
  if (committed_triples_.count(triple) || pending_triples_.count(triple)) {
    return skip(triple, "duplicate triple", false);
  }

  if (!raw_packet->da3_context_packet) {
    return skip(triple,
                "shared-image overlap packet is missing; chain re-anchored");
  }
  if (raw_packet->da3_context_packet->keyframe_id != pair.first) {
    return skip(triple,
                "shared-image overlap packet ID is malformed; chain "
                "re-anchored");
  }
  if (!previous_pair.packet->body_T_cam.equals(
          *raw_packet->da3_context_body_T_cam, kExtrinsicTolerance)) {
    return skip(triple,
                "shared-frame camera extrinsics disagree; chain re-anchored");
  }

  const Da3OverlapScaleEstimate overlap = VIO::estimateDa3OverlapScale(
      *previous_pair.packet, *raw_packet->da3_context_packet, point_stride_);
  if (!overlap.valid) {
    auto result =
        skip(triple,
             overlap.failure_reason + "; chain re-anchored (candidates=" +
                 std::to_string(overlap.candidate_count) +
                 ", inliers=" + std::to_string(overlap.inlier_count) +
                 ", log_rmse=" + std::to_string(overlap.log_rmse) + ")");
    result.overlap_candidate_count = overlap.candidate_count;
    result.overlap_inlier_count = overlap.inlier_count;
    result.overlap_log_rmse = overlap.log_rmse;
    return result;
  }

  const double previous_da3_baseline =
      previous_pair.packet->da3_context_cam_T_current_cam->translation().norm();
  if (!std::isfinite(previous_da3_baseline) ||
      previous_da3_baseline <= kMinimumBaseline) {
    return skip(triple, "previous DA3 pair baseline is invalid or zero");
  }
  const double measured_ratio =
      overlap.scale_ratio * current_da3_baseline / previous_da3_baseline;
  if (!std::isfinite(measured_ratio) || measured_ratio <= 0.0) {
    return skip(triple, "overlap-derived measured ratio is invalid");
  }

  const gtsam::Key first_key = gtsam::Symbol(kPoseSymbolChar, triple[0]);
  const gtsam::Key middle_key = gtsam::Symbol(kPoseSymbolChar, triple[1]);
  const gtsam::Key last_key = gtsam::Symbol(kPoseSymbolChar, triple[2]);
  const std::optional<gtsam::Pose3> first_body_pose =
      findPose(state, first_key);
  if (!first_body_pose) {
    return skip(triple, "first pose is outside the fixed-lag window");
  }
  const std::optional<gtsam::Pose3> middle_body_pose =
      findPose(state, middle_key);
  if (!middle_body_pose) {
    return skip(triple, "middle pose is outside the fixed-lag window");
  }
  std::optional<gtsam::Pose3> last_body_pose = findPose(state, last_key);
  if (!last_body_pose) {
    last_body_pose = findPose(new_values, last_key);
  }
  if (!last_body_pose) {
    return skip(triple, "last pose is unavailable");
  }

  try {
    auto factor = std::make_shared<CameraAwareBaselineRatioFactor>(
        first_key,
        middle_key,
        last_key,
        *previous_pair.packet->da3_context_body_T_cam,
        previous_pair.packet->body_T_cam,
        raw_packet->body_T_cam,
        measured_ratio,
        noise_model_);
    const double initial_error = factor->evaluateError(
        *first_body_pose, *middle_body_pose, *last_body_pose)(0);
    new_factors->push_back(factor);
    pending_triples_.insert(triple);
    pending_predecessors_.emplace(triple, previous_pair);

    Da3BaselineRatioFactorAddResult result;
    result.added = true;
    result.triple = triple;
    result.diagnostic = "added";
    result.overlap_scale_ratio = overlap.scale_ratio;
    result.first_da3_translation_norm = previous_da3_baseline;
    result.second_da3_translation_norm = current_da3_baseline;
    result.measured_baseline_ratio = measured_ratio;
    result.overlap_candidate_count = overlap.candidate_count;
    result.overlap_inlier_count = overlap.inlier_count;
    result.overlap_log_rmse = overlap.log_rmse;
    result.initial_log_ratio_residual = initial_error;
    LOG(INFO) << "Added DA3 baseline-ratio triple " << tripleString(triple)
              << ": overlap_scale=" << result.overlap_scale_ratio
              << ", da3_translation_norms=["
              << result.first_da3_translation_norm << ", "
              << result.second_da3_translation_norm
              << "], measured_ratio=" << result.measured_baseline_ratio
              << ", overlap_inliers=" << result.overlap_inlier_count << "/"
              << result.overlap_candidate_count
              << ", overlap_log_rmse=" << result.overlap_log_rmse
              << ", initial_log_ratio_residual="
              << result.initial_log_ratio_residual;
    return result;
  } catch (const std::exception& e) {
    return skip(triple, std::string("invalid factor: ") + e.what());
  }
}

void Da3BaselineRatioFactors::notifySmootherUpdateResult(
    const bool update_succeeded) {
  if (update_succeeded) {
    committed_triples_.insert(pending_triples_.begin(), pending_triples_.end());
  } else if (!pending_predecessors_.empty()) {
    // Restore the predecessor so resubmitting the same canonical packet can
    // reconstruct the exact triple that the smoother rejected.
    anchor_pair_ = pending_predecessors_.begin()->second;
  }
  pending_triples_.clear();
  pending_predecessors_.clear();
}

}  // namespace VIO
