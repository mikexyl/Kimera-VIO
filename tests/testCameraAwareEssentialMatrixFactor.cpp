#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "kimera-vio/factors/CameraAwareEssentialMatrixFactor.h"

#define EXPECT_TRUE(condition, message)            \
  do {                                             \
    if (!(condition)) {                            \
      std::cerr << "[FAIL] " << (message) << "\n"; \
      return EXIT_FAILURE;                         \
    }                                              \
  } while (false)

namespace {

constexpr double kTolerance = 1e-8;

gtsam::SharedNoiseModel model(const std::size_t dimension = 5u) {
  return gtsam::noiseModel::Isotropic::Sigma(dimension, 1.0);
}

gtsam::Pose3 bodyPoseForCameraRelative(
    const gtsam::Pose3& world_T_context_body,
    const gtsam::Pose3& context_body_T_cam,
    const gtsam::Pose3& current_body_T_cam,
    const gtsam::Pose3& context_cam_T_current_cam) {
  const gtsam::Pose3 world_T_current_cam =
      world_T_context_body.compose(context_body_T_cam)
          .compose(context_cam_T_current_cam);
  return world_T_current_cam.compose(current_body_T_cam.inverse());
}

struct Geometry {
  gtsam::Pose3 context_body_T_cam{gtsam::Rot3::RzRyRx(0.08, -0.04, 0.11),
                                  gtsam::Point3(0.14, -0.03, 0.06)};
  gtsam::Pose3 current_body_T_cam{gtsam::Rot3::RzRyRx(-0.05, 0.09, -0.03),
                                  gtsam::Point3(-0.07, 0.04, 0.09)};
  gtsam::Pose3 measurement{gtsam::Rot3::RzRyRx(0.19, -0.12, 0.07),
                           gtsam::Point3(0.6, -0.35, 0.42)};
  gtsam::Pose3 world_T_context_body{gtsam::Rot3::RzRyRx(-0.17, 0.13, 0.21),
                                    gtsam::Point3(-2.0, 1.5, 0.7)};
  gtsam::Pose3 world_T_current_body =
      bodyPoseForCameraRelative(world_T_context_body,
                                context_body_T_cam,
                                current_body_T_cam,
                                measurement);
};

int testZeroErrorWithEndpointExtrinsics() {
  const Geometry geometry;
  VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                               2u,
                                               geometry.measurement,
                                               geometry.context_body_T_cam,
                                               geometry.current_body_T_cam,
                                               model());
  EXPECT_TRUE(
      factor.evaluateError(geometry.world_T_context_body,
                           geometry.world_T_current_body)
              .norm() < kTolerance,
      "nontrivial, endpoint-specific camera extrinsics give zero error");
  EXPECT_TRUE(
      factor.measuredPose().equals(geometry.measurement) &&
          factor.contextBodyTCam().equals(geometry.context_body_T_cam) &&
          factor.currentBodyTCam().equals(geometry.current_body_T_cam),
      "measurement accessors preserve constructor inputs");
  const auto cloned = factor.clone();
  EXPECT_TRUE(factor.equals(*cloned), "clone and equality preserve the factor");
  return EXIT_SUCCESS;
}

int testPositiveScaleInvariance() {
  const Geometry geometry;
  const gtsam::Pose3 perturbed_current = geometry.world_T_current_body.retract(
      (gtsam::Vector6() << 0.02, -0.01, 0.015, 0.04, -0.03, 0.01).finished());
  gtsam::Vector5 reference_error;
  for (const double scale : {0.1, 1.0, 17.0}) {
    const gtsam::Pose3 scaled_measurement(
        geometry.measurement.rotation(),
        scale * geometry.measurement.translation());
    VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                                 2u,
                                                 scaled_measurement,
                                                 geometry.context_body_T_cam,
                                                 geometry.current_body_T_cam,
                                                 model());
    const gtsam::Vector5 error =
        factor.evaluateError(geometry.world_T_context_body, perturbed_current);
    if (scale == 0.1) {
      reference_error = error;
    } else {
      EXPECT_TRUE((error - reference_error).norm() < kTolerance,
                  "positive measurement scales produce identical error");
    }
  }

  for (const double scale : {0.2, 4.0}) {
    const gtsam::Pose3 scaled_prediction(
        geometry.measurement.rotation(),
        scale * geometry.measurement.translation());
    const gtsam::Pose3 current_body =
        bodyPoseForCameraRelative(geometry.world_T_context_body,
                                  geometry.context_body_T_cam,
                                  geometry.current_body_T_cam,
                                  scaled_prediction);
    VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                                 2u,
                                                 geometry.measurement,
                                                 geometry.context_body_T_cam,
                                                 geometry.current_body_T_cam,
                                                 model());
    EXPECT_TRUE(
        factor.evaluateError(geometry.world_T_context_body, current_body)
                .norm() < kTolerance,
        "positive predicted scales produce identical zero error");
  }
  return EXIT_SUCCESS;
}

int testResidualSeparationAndDirectionSign() {
  const Geometry geometry;
  VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                               2u,
                                               geometry.measurement,
                                               geometry.context_body_T_cam,
                                               geometry.current_body_T_cam,
                                               model());

  const gtsam::Pose3 rotation_only_prediction(
      geometry.measurement.rotation().retract(
          gtsam::Vector3(0.03, -0.02, 0.01)),
      geometry.measurement.translation());
  const gtsam::Vector5 rotation_error = factor.evaluateError(
      geometry.world_T_context_body,
      bodyPoseForCameraRelative(geometry.world_T_context_body,
                                geometry.context_body_T_cam,
                                geometry.current_body_T_cam,
                                rotation_only_prediction));
  EXPECT_TRUE(rotation_error.head<3>().norm() > 0.02 &&
                  rotation_error.tail<2>().norm() < kTolerance,
              "rotation perturbations affect only rotation residuals");

  const gtsam::Unit3 perturbed_direction =
      gtsam::Unit3(geometry.measurement.translation())
          .retract(gtsam::Vector2(0.04, -0.025));
  const gtsam::Pose3 direction_only_prediction(geometry.measurement.rotation(),
                                               perturbed_direction.point3());
  const gtsam::Vector5 direction_error = factor.evaluateError(
      geometry.world_T_context_body,
      bodyPoseForCameraRelative(geometry.world_T_context_body,
                                geometry.context_body_T_cam,
                                geometry.current_body_T_cam,
                                direction_only_prediction));
  EXPECT_TRUE(direction_error.head<3>().norm() < kTolerance &&
                  direction_error.tail<2>().norm() > 0.03,
              "direction perturbations affect only direction residuals");

  const gtsam::Pose3 opposite_prediction(geometry.measurement.rotation(),
                                         -geometry.measurement.translation());
  const gtsam::Vector5 opposite_error = factor.evaluateError(
      geometry.world_T_context_body,
      bodyPoseForCameraRelative(geometry.world_T_context_body,
                                geometry.context_body_T_cam,
                                geometry.current_body_T_cam,
                                opposite_prediction));
  EXPECT_TRUE(opposite_error.tail<2>().norm() > 3.0,
              "opposite directed translation produces a large error");
  return EXIT_SUCCESS;
}

int testAnalyticJacobiansAndRadialNullSpace() {
  const Geometry geometry;
  VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                               2u,
                                               geometry.measurement,
                                               geometry.context_body_T_cam,
                                               geometry.current_body_T_cam,
                                               model());
  const gtsam::Pose3 context = geometry.world_T_context_body.retract(
      (gtsam::Vector6() << 0.01, -0.02, 0.015, 0.02, 0.01, -0.03).finished());
  const gtsam::Pose3 current = geometry.world_T_current_body.retract(
      (gtsam::Vector6() << -0.015, 0.012, 0.018, -0.025, 0.035, 0.02)
          .finished());
  gtsam::Matrix H_context;
  gtsam::Matrix H_current;
  factor.evaluateError(context, current, &H_context, &H_current);
  const gtsam::Matrix numerical_context =
      gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          [&factor, &current](const gtsam::Pose3& value) {
            return factor.evaluateError(value, current);
          },
          context,
          1e-6);
  const gtsam::Matrix numerical_current =
      gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          [&factor, &context](const gtsam::Pose3& value) {
            return factor.evaluateError(context, value);
          },
          current,
          1e-6);
  EXPECT_TRUE((H_context - numerical_context).norm() < 2e-5 &&
                  (H_current - numerical_current).norm() < 2e-5,
              "analytic Jacobians match numerical derivatives away from zero");

  gtsam::Matrix H_zero_context;
  gtsam::Matrix H_zero_current;
  factor.evaluateError(geometry.world_T_context_body,
                       geometry.world_T_current_body,
                       &H_zero_context,
                       &H_zero_current);
  const gtsam::Pose3 world_T_context_cam =
      geometry.world_T_context_body.compose(geometry.context_body_T_cam);
  const gtsam::Pose3 world_T_current_cam =
      geometry.world_T_current_body.compose(geometry.current_body_T_cam);
  const gtsam::Vector3 radial_world =
      world_T_current_cam.translation() - world_T_context_cam.translation();
  const gtsam::Vector3 radial_current_body =
      geometry.world_T_current_body.rotation().unrotate(radial_world);
  gtsam::Vector6 radial_delta = gtsam::Vector6::Zero();
  radial_delta.tail<3>() = radial_current_body;
  EXPECT_TRUE((H_zero_current * radial_delta).norm() < 1e-8,
              "radial translation lies in the factor Jacobian null space");
  return EXIT_SUCCESS;
}

int testInvalidInputs() {
  const Geometry geometry;
  bool invalid_dimension = false;
  try {
    VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                                 2u,
                                                 geometry.measurement,
                                                 geometry.context_body_T_cam,
                                                 geometry.current_body_T_cam,
                                                 model(6u));
  } catch (const std::invalid_argument&) {
    invalid_dimension = true;
  }
  EXPECT_TRUE(invalid_dimension, "invalid noise dimension is rejected");

  bool zero_translation = false;
  try {
    VIO::CameraAwareEssentialMatrixFactor factor(
        1u,
        2u,
        gtsam::Pose3(geometry.measurement.rotation(), gtsam::Point3::Zero()),
        geometry.context_body_T_cam,
        geometry.current_body_T_cam,
        model());
  } catch (const std::invalid_argument&) {
    zero_translation = true;
  }
  EXPECT_TRUE(zero_translation, "zero measurement translation is rejected");

  bool non_finite = false;
  try {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    VIO::CameraAwareEssentialMatrixFactor factor(
        1u,
        2u,
        gtsam::Pose3(geometry.measurement.rotation(),
                     gtsam::Point3(nan, 0.0, 1.0)),
        geometry.context_body_T_cam,
        geometry.current_body_T_cam,
        model());
  } catch (const std::invalid_argument&) {
    non_finite = true;
  }
  EXPECT_TRUE(non_finite, "non-finite measurement is rejected");

  VIO::CameraAwareEssentialMatrixFactor factor(1u,
                                               2u,
                                               geometry.measurement,
                                               geometry.context_body_T_cam,
                                               geometry.current_body_T_cam,
                                               model());
  bool zero_prediction = false;
  try {
    const gtsam::Pose3 coincident_current =
        bodyPoseForCameraRelative(geometry.world_T_context_body,
                                  geometry.context_body_T_cam,
                                  geometry.current_body_T_cam,
                                  gtsam::Pose3());
    static_cast<void>(factor.evaluateError(geometry.world_T_context_body,
                                           coincident_current));
  } catch (const std::domain_error&) {
    zero_prediction = true;
  }
  EXPECT_TRUE(zero_prediction, "zero predicted baseline is rejected");
  return EXIT_SUCCESS;
}

}  // namespace

int main() {
  int status = EXIT_SUCCESS;
  status |= testZeroErrorWithEndpointExtrinsics();
  status |= testPositiveScaleInvariance();
  status |= testResidualSeparationAndDirectionSign();
  status |= testAnalyticJacobiansAndRadialNullSpace();
  status |= testInvalidInputs();
  if (status == EXIT_SUCCESS) {
    std::cout << "All camera-aware essential-matrix factor tests PASSED.\n";
  }
  return status;
}
