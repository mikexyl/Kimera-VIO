/* ----------------------------------------------------------------------------
 * Copyright 2026
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace VIO {

using CameraAwareBaselineRatioFactorBase =
    gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>;
#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using CameraAwareBaselineRatioJacobian = boost::optional<gtsam::Matrix&>;
#else
using CameraAwareBaselineRatioJacobian = gtsam::OptionalMatrixType;
#endif

/**
 * A scale-free constraint on two consecutive camera-center baselines.
 *
 * The optimized variables are body poses.  Endpoint-specific body-to-camera
 * extrinsics are used to recover the three camera centers before evaluating
 * log(||C_k-C_j|| / ||C_j-C_i||) - log(measured_ratio).
 */
class CameraAwareBaselineRatioFactor
    : public CameraAwareBaselineRatioFactorBase {
 public:
  using Base = CameraAwareBaselineRatioFactorBase;
  using This = CameraAwareBaselineRatioFactor;

  CameraAwareBaselineRatioFactor() = default;

  CameraAwareBaselineRatioFactor(gtsam::Key first_body_key,
                                 gtsam::Key middle_body_key,
                                 gtsam::Key last_body_key,
                                 const gtsam::Pose3& first_body_T_cam,
                                 const gtsam::Pose3& middle_body_T_cam,
                                 const gtsam::Pose3& last_body_T_cam,
                                 double measured_baseline_ratio,
                                 const gtsam::SharedNoiseModel& noise_model);

  ~CameraAwareBaselineRatioFactor() override = default;

  gtsam::Vector evaluateError(
      const gtsam::Pose3& world_T_first_body,
      const gtsam::Pose3& world_T_middle_body,
      const gtsam::Pose3& world_T_last_body,
      CameraAwareBaselineRatioJacobian H_first_body = {},
      CameraAwareBaselineRatioJacobian H_middle_body = {},
      CameraAwareBaselineRatioJacobian H_last_body = {}) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

  bool equals(const gtsam::NonlinearFactor& expected,
              double tol = 1e-9) const override;

  void print(const std::string& s = "",
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  double measuredBaselineRatio() const { return measured_baseline_ratio_; }

  const gtsam::Pose3& firstBodyTCam() const { return first_body_T_cam_; }

  const gtsam::Pose3& middleBodyTCam() const { return middle_body_T_cam_; }

  const gtsam::Pose3& lastBodyTCam() const { return last_body_T_cam_; }

 private:
  static bool isFinite(const gtsam::Pose3& pose);

  gtsam::Pose3 first_body_T_cam_;
  gtsam::Pose3 middle_body_T_cam_;
  gtsam::Pose3 last_body_T_cam_;
  double measured_baseline_ratio_ = 1.0;
};

}  // namespace VIO
