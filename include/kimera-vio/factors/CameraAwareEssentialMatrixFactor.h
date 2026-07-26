/* ----------------------------------------------------------------------------
 * Copyright 2026
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace VIO {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using CameraAwareEssentialJacobian = boost::optional<gtsam::Matrix&>;
#else
using CameraAwareEssentialJacobian = gtsam::OptionalMatrixType;
#endif

/**
 * A five degree-of-freedom relative-pose constraint between body poses.
 *
 * The measurement and residual are expressed between cameras.  The optimized
 * variables are body poses, so the (potentially different) body-to-camera
 * transforms at the two endpoints are composed internally.  Translation is
 * constrained only on the directed unit sphere; positive measurement or
 * prediction scale therefore has no effect on the residual.
 */
class CameraAwareEssentialMatrixFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
  using This = CameraAwareEssentialMatrixFactor;

  CameraAwareEssentialMatrixFactor() = default;

  CameraAwareEssentialMatrixFactor(
      gtsam::Key context_body_key,
      gtsam::Key current_body_key,
      const gtsam::Pose3& context_cam_T_current_cam,
      const gtsam::Pose3& context_body_T_cam,
      const gtsam::Pose3& current_body_T_cam,
      const gtsam::SharedNoiseModel& noise_model);

  ~CameraAwareEssentialMatrixFactor() override = default;

  gtsam::Vector evaluateError(
      const gtsam::Pose3& world_T_context_body,
      const gtsam::Pose3& world_T_current_body,
      CameraAwareEssentialJacobian H_context_body = {},
      CameraAwareEssentialJacobian H_current_body = {}) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

  bool equals(const gtsam::NonlinearFactor& expected,
              double tol = 1e-9) const override;

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  const gtsam::EssentialMatrix& measured() const { return measured_; }

  const gtsam::Pose3& measuredPose() const {
    return context_cam_T_current_cam_;
  }

  const gtsam::Pose3& contextBodyTCam() const { return context_body_T_cam_; }

  const gtsam::Pose3& currentBodyTCam() const { return current_body_T_cam_; }

 private:
  static bool isFinite(const gtsam::Pose3& pose);

  static gtsam::Vector5 localCoordinates(
      const gtsam::EssentialMatrix& origin,
      const gtsam::EssentialMatrix& other,
      gtsam::OptionalJacobian<5, 5> H_other = {});

  gtsam::Pose3 context_cam_T_current_cam_;
  gtsam::Pose3 context_body_T_cam_;
  gtsam::Pose3 current_body_T_cam_;
  gtsam::EssentialMatrix measured_;
};

}  // namespace VIO
