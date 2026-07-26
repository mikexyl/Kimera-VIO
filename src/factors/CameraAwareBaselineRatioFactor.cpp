/* ----------------------------------------------------------------------------
 * Copyright 2026
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "kimera-vio/factors/CameraAwareBaselineRatioFactor.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace VIO {
namespace {

constexpr double kMinimumBaseline = 1e-9;

}  // namespace

CameraAwareBaselineRatioFactor::CameraAwareBaselineRatioFactor(
    const gtsam::Key first_body_key,
    const gtsam::Key middle_body_key,
    const gtsam::Key last_body_key,
    const gtsam::Pose3& first_body_T_cam,
    const gtsam::Pose3& middle_body_T_cam,
    const gtsam::Pose3& last_body_T_cam,
    const double measured_baseline_ratio,
    const gtsam::SharedNoiseModel& noise_model)
    : Base(noise_model, first_body_key, middle_body_key, last_body_key),
      first_body_T_cam_(first_body_T_cam),
      middle_body_T_cam_(middle_body_T_cam),
      last_body_T_cam_(last_body_T_cam),
      measured_baseline_ratio_(measured_baseline_ratio) {
  if (!noise_model || noise_model->dim() != 1u) {
    throw std::invalid_argument(
        "CameraAwareBaselineRatioFactor requires a 1D noise model");
  }
  if (!isFinite(first_body_T_cam_) || !isFinite(middle_body_T_cam_) ||
      !isFinite(last_body_T_cam_)) {
    throw std::invalid_argument(
        "CameraAwareBaselineRatioFactor extrinsics must be finite");
  }
  if (!std::isfinite(measured_baseline_ratio_) ||
      measured_baseline_ratio_ <= 0.0) {
    throw std::invalid_argument(
        "CameraAwareBaselineRatioFactor measured ratio must be finite and "
        "positive");
  }
}

bool CameraAwareBaselineRatioFactor::isFinite(const gtsam::Pose3& pose) {
  return pose.rotation().matrix().allFinite() && pose.translation().allFinite();
}

gtsam::Vector CameraAwareBaselineRatioFactor::evaluateError(
    const gtsam::Pose3& world_T_first_body,
    const gtsam::Pose3& world_T_middle_body,
    const gtsam::Pose3& world_T_last_body,
    CameraAwareBaselineRatioJacobian H_first_body,
    CameraAwareBaselineRatioJacobian H_middle_body,
    CameraAwareBaselineRatioJacobian H_last_body) const {
  if (!isFinite(world_T_first_body) || !isFinite(world_T_middle_body) ||
      !isFinite(world_T_last_body)) {
    throw std::domain_error(
        "CameraAwareBaselineRatioFactor body poses must be finite");
  }

  gtsam::Matrix36 H_first_center;
  gtsam::Matrix36 H_middle_center;
  gtsam::Matrix36 H_last_center;
  const gtsam::Point3 first_center = world_T_first_body.transformFrom(
      first_body_T_cam_.translation(),
      H_first_body ? &H_first_center : nullptr);
  const gtsam::Point3 middle_center = world_T_middle_body.transformFrom(
      middle_body_T_cam_.translation(),
      H_middle_body ? &H_middle_center : nullptr);
  const gtsam::Point3 last_center = world_T_last_body.transformFrom(
      last_body_T_cam_.translation(), H_last_body ? &H_last_center : nullptr);

  const gtsam::Vector3 first_baseline = middle_center - first_center;
  const gtsam::Vector3 second_baseline = last_center - middle_center;
  const double first_squared_norm = first_baseline.squaredNorm();
  const double second_squared_norm = second_baseline.squaredNorm();
  const double first_norm = std::sqrt(first_squared_norm);
  const double second_norm = std::sqrt(second_squared_norm);
  if (!first_baseline.allFinite() || !second_baseline.allFinite() ||
      !std::isfinite(first_norm) || !std::isfinite(second_norm) ||
      first_norm <= kMinimumBaseline || second_norm <= kMinimumBaseline) {
    throw std::domain_error(
        "CameraAwareBaselineRatioFactor prediction must have two nonzero "
        "baselines");
  }

  const Eigen::RowVector3d H_error_first_center =
      first_baseline.transpose() / first_squared_norm;
  const Eigen::RowVector3d H_error_middle_center =
      -first_baseline.transpose() / first_squared_norm -
      second_baseline.transpose() / second_squared_norm;
  const Eigen::RowVector3d H_error_last_center =
      second_baseline.transpose() / second_squared_norm;
  if (H_first_body) {
    *H_first_body = H_error_first_center * H_first_center;
  }
  if (H_middle_body) {
    *H_middle_body = H_error_middle_center * H_middle_center;
  }
  if (H_last_body) {
    *H_last_body = H_error_last_center * H_last_center;
  }

  gtsam::Vector1 error;
  error << std::log(second_norm) - std::log(first_norm) -
               std::log(measured_baseline_ratio_);
  return error;
}

gtsam::NonlinearFactor::shared_ptr CameraAwareBaselineRatioFactor::clone()
    const {
  return gtsam::NonlinearFactor::shared_ptr(new This(*this));
}

bool CameraAwareBaselineRatioFactor::equals(
    const gtsam::NonlinearFactor& expected,
    const double tol) const {
  const auto* other = dynamic_cast<const This*>(&expected);
  return other && Base::equals(*other, tol) &&
         first_body_T_cam_.equals(other->first_body_T_cam_, tol) &&
         middle_body_T_cam_.equals(other->middle_body_T_cam_, tol) &&
         last_body_T_cam_.equals(other->last_body_T_cam_, tol) &&
         std::abs(measured_baseline_ratio_ - other->measured_baseline_ratio_) <=
             tol;
}

void CameraAwareBaselineRatioFactor::print(
    const std::string& s,
    const gtsam::KeyFormatter& key_formatter) const {
  std::cout << s << "CameraAwareBaselineRatioFactor(" << key_formatter(key1())
            << ", " << key_formatter(key2()) << ", " << key_formatter(key3())
            << ")\n"
            << "  measured baseline ratio: " << measured_baseline_ratio_
            << "\n";
  first_body_T_cam_.print("  first body_T_cam: ");
  middle_body_T_cam_.print("  middle body_T_cam: ");
  last_body_T_cam_.print("  last body_T_cam: ");
  if (noiseModel_) {
    noiseModel_->print("  noise model: ");
  }
}

}  // namespace VIO
