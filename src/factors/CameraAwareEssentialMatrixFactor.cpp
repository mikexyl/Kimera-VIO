/* ----------------------------------------------------------------------------
 * Copyright 2026
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "kimera-vio/factors/CameraAwareEssentialMatrixFactor.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace VIO {
namespace {

constexpr double kMinimumBaseline = 1e-9;
constexpr double kDirectionSingularityTolerance = 1e-12;

}  // namespace

CameraAwareEssentialMatrixFactor::CameraAwareEssentialMatrixFactor(
    const gtsam::Key context_body_key,
    const gtsam::Key current_body_key,
    const gtsam::Pose3& context_cam_T_current_cam,
    const gtsam::Pose3& context_body_T_cam,
    const gtsam::Pose3& current_body_T_cam,
    const gtsam::SharedNoiseModel& noise_model)
    : Base(noise_model, context_body_key, current_body_key),
      context_cam_T_current_cam_(context_cam_T_current_cam),
      context_body_T_cam_(context_body_T_cam),
      current_body_T_cam_(current_body_T_cam) {
  if (!noise_model || noise_model->dim() != 5u) {
    throw std::invalid_argument(
        "CameraAwareEssentialMatrixFactor requires a 5D noise model");
  }
  if (!isFinite(context_cam_T_current_cam_) || !isFinite(context_body_T_cam_) ||
      !isFinite(current_body_T_cam_)) {
    throw std::invalid_argument(
        "CameraAwareEssentialMatrixFactor poses must be finite");
  }
  const double measurement_baseline =
      context_cam_T_current_cam_.translation().norm();
  if (!std::isfinite(measurement_baseline) ||
      measurement_baseline <= kMinimumBaseline) {
    throw std::invalid_argument(
        "CameraAwareEssentialMatrixFactor measurement must have nonzero "
        "translation");
  }
  measured_ = gtsam::EssentialMatrix::FromPose3(context_cam_T_current_cam_);
}

bool CameraAwareEssentialMatrixFactor::isFinite(const gtsam::Pose3& pose) {
  return pose.rotation().matrix().allFinite() && pose.translation().allFinite();
}

gtsam::Vector5 CameraAwareEssentialMatrixFactor::localCoordinates(
    const gtsam::EssentialMatrix& origin,
    const gtsam::EssentialMatrix& other,
    gtsam::OptionalJacobian<5, 5> H_other) {
  gtsam::Vector5 error;

  gtsam::Matrix3 H_rotation;
  const gtsam::Rot3 relative_rotation = origin.rotation().between(
      other.rotation(), {}, H_other ? &H_rotation : nullptr);
  gtsam::Matrix3 H_log;
  error.head<3>() =
      gtsam::Rot3::Logmap(relative_rotation, H_other ? &H_log : nullptr);

  const gtsam::Unit3& p = origin.direction();
  const gtsam::Unit3& q = other.direction();
  const gtsam::Vector3 p_vector = p.unitVector();
  gtsam::Matrix32 H_q_vector;
  const gtsam::Vector3 q_vector = q.unitVector(H_other ? &H_q_vector : nullptr);
  const double x = std::clamp(p_vector.dot(q_vector), -1.0, 1.0);

  gtsam::Matrix2 H_direction = gtsam::Matrix2::Zero();
  if (x <= -1.0 + kDirectionSingularityTolerance) {
    // Unit3::localCoordinates uses this deterministic antipodal convention.
    error.tail<2>() = gtsam::Vector2(M_PI, 0.0);
  } else {
    const double one_minus_x_squared = std::max(0.0, 1.0 - x * x);
    double scale = 1.0;
    double scale_derivative = -1.0 / 3.0;
    if (one_minus_x_squared > kDirectionSingularityTolerance) {
      const double sine = std::sqrt(one_minus_x_squared);
      const double theta = std::acos(x);
      scale = theta / sine;
      scale_derivative = (theta * x - sine) / (sine * sine * sine);
    } else {
      // First-order expansion of acos(x) / sqrt(1 - x^2) near x=1.
      scale = 1.0 - (x - 1.0) / 3.0;
    }

    const gtsam::Vector3 tangent = q_vector - x * p_vector;
    const gtsam::Matrix32 p_basis = p.basis();
    error.tail<2>() = p_basis.transpose() * scale * tangent;

    if (H_other) {
      const gtsam::Matrix3 tangent_projector =
          gtsam::Matrix3::Identity() - p_vector * p_vector.transpose();
      const gtsam::Matrix3 H_error_q =
          scale * tangent_projector +
          scale_derivative * tangent * p_vector.transpose();
      H_direction = p_basis.transpose() * H_error_q * H_q_vector;
    }
  }

  if (H_other) {
    H_other->setZero();
    H_other->block<3, 3>(0, 0) = H_log * H_rotation;
    H_other->block<2, 2>(3, 3) = H_direction;
  }
  return error;
}

gtsam::Vector CameraAwareEssentialMatrixFactor::evaluateError(
    const gtsam::Pose3& world_T_context_body,
    const gtsam::Pose3& world_T_current_body,
    gtsam::OptionalMatrixType H_context_body,
    gtsam::OptionalMatrixType H_current_body) const {
  const bool compute_jacobians = H_context_body || H_current_body;
  gtsam::Matrix66 H_context_cam_context_body;
  gtsam::Matrix66 H_current_cam_current_body;
  const gtsam::Pose3 world_T_context_cam = world_T_context_body.compose(
      context_body_T_cam_,
      compute_jacobians ? &H_context_cam_context_body : nullptr,
      nullptr);
  const gtsam::Pose3 world_T_current_cam = world_T_current_body.compose(
      current_body_T_cam_,
      compute_jacobians ? &H_current_cam_current_body : nullptr,
      nullptr);

  gtsam::Matrix66 H_relative_context_cam;
  gtsam::Matrix66 H_relative_current_cam;
  const gtsam::Pose3 predicted_pose = world_T_context_cam.between(
      world_T_current_cam,
      compute_jacobians ? &H_relative_context_cam : nullptr,
      compute_jacobians ? &H_relative_current_cam : nullptr);
  const double predicted_baseline = predicted_pose.translation().norm();
  if (!isFinite(predicted_pose) || !std::isfinite(predicted_baseline) ||
      predicted_baseline <= kMinimumBaseline) {
    throw std::domain_error(
        "CameraAwareEssentialMatrixFactor prediction must have a nonzero "
        "baseline");
  }

  gtsam::Matrix56 H_essential_relative;
  const gtsam::EssentialMatrix predicted = gtsam::EssentialMatrix::FromPose3(
      predicted_pose, compute_jacobians ? &H_essential_relative : nullptr);
  gtsam::Matrix5 H_error_essential;
  const gtsam::Vector5 error = localCoordinates(
      measured_, predicted, compute_jacobians ? &H_error_essential : nullptr);

  if (H_context_body) {
    *H_context_body = H_error_essential * H_essential_relative *
                      H_relative_context_cam * H_context_cam_context_body;
  }
  if (H_current_body) {
    *H_current_body = H_error_essential * H_essential_relative *
                      H_relative_current_cam * H_current_cam_current_body;
  }
  return error;
}

gtsam::NonlinearFactor::shared_ptr CameraAwareEssentialMatrixFactor::clone()
    const {
  return std::static_pointer_cast<gtsam::NonlinearFactor>(
      std::make_shared<This>(*this));
}

bool CameraAwareEssentialMatrixFactor::equals(
    const gtsam::NonlinearFactor& expected,
    const double tol) const {
  const auto* other = dynamic_cast<const This*>(&expected);
  return other && Base::equals(*other, tol) &&
         context_cam_T_current_cam_.equals(other->context_cam_T_current_cam_,
                                           tol) &&
         context_body_T_cam_.equals(other->context_body_T_cam_, tol) &&
         current_body_T_cam_.equals(other->current_body_T_cam_, tol) &&
         measured_.equals(other->measured_, tol);
}

void CameraAwareEssentialMatrixFactor::print(
    const std::string& s,
    const gtsam::KeyFormatter& key_formatter) const {
  std::cout << s << "CameraAwareEssentialMatrixFactor(" << key_formatter(key1())
            << ", " << key_formatter(key2()) << ")\n";
  measured_.print("  measured essential matrix: ");
  context_body_T_cam_.print("  context body_T_cam: ");
  current_body_T_cam_.print("  current body_T_cam: ");
  if (noiseModel_) {
    noiseModel_->print("  noise model: ");
  }
}

}  // namespace VIO
