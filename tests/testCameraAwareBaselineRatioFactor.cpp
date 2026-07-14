#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "kimera-vio/factors/CameraAwareBaselineRatioFactor.h"

#define EXPECT_TRUE(condition, message)            \
  do {                                             \
    if (!(condition)) {                            \
      std::cerr << "[FAIL] " << (message) << "\n"; \
      return EXIT_FAILURE;                         \
    }                                              \
  } while (false)

namespace {

constexpr double kTolerance = 1e-9;

gtsam::SharedNoiseModel model(const std::size_t dimension = 1u) {
  return gtsam::noiseModel::Isotropic::Sigma(dimension, 1.0);
}

gtsam::Pose3 bodyPoseForCameraCenter(const gtsam::Point3& center,
                                     const gtsam::Rot3& body_rotation,
                                     const gtsam::Pose3& body_T_cam) {
  return gtsam::Pose3(body_rotation,
                      center - body_rotation.rotate(body_T_cam.translation()));
}

struct Geometry {
  gtsam::Pose3 first_body_T_cam{gtsam::Rot3::RzRyRx(0.08, -0.04, 0.11),
                                gtsam::Point3(0.14, -0.03, 0.06)};
  gtsam::Pose3 middle_body_T_cam{gtsam::Rot3::RzRyRx(-0.05, 0.09, -0.03),
                                 gtsam::Point3(-0.07, 0.04, 0.09)};
  gtsam::Pose3 last_body_T_cam{gtsam::Rot3::RzRyRx(0.02, 0.06, -0.08),
                               gtsam::Point3(0.03, 0.12, -0.05)};
  gtsam::Point3 first_center{-1.0, 0.5, 0.2};
  gtsam::Point3 middle_center{1.0, -0.5, 0.7};
  gtsam::Point3 last_center{0.4, 1.3, 3.1};
  gtsam::Pose3 first_body =
      bodyPoseForCameraCenter(first_center,
                              gtsam::Rot3::RzRyRx(-0.17, 0.13, 0.21),
                              first_body_T_cam);
  gtsam::Pose3 middle_body =
      bodyPoseForCameraCenter(middle_center,
                              gtsam::Rot3::RzRyRx(0.14, -0.07, -0.12),
                              middle_body_T_cam);
  gtsam::Pose3 last_body =
      bodyPoseForCameraCenter(last_center,
                              gtsam::Rot3::RzRyRx(-0.09, 0.18, 0.04),
                              last_body_T_cam);
  double measured_ratio = (last_center - middle_center).norm() /
                          (middle_center - first_center).norm();

  VIO::CameraAwareBaselineRatioFactor factor() const {
    return VIO::CameraAwareBaselineRatioFactor(1u,
                                               2u,
                                               3u,
                                               first_body_T_cam,
                                               middle_body_T_cam,
                                               last_body_T_cam,
                                               measured_ratio,
                                               model());
  }
};

int testZeroErrorAndAccessors() {
  const Geometry geometry;
  const auto factor = geometry.factor();
  EXPECT_TRUE(
      factor.evaluateError(
                geometry.first_body, geometry.middle_body, geometry.last_body)
              .norm() < kTolerance,
      "nontrivial endpoint extrinsics give zero ratio error");
  EXPECT_TRUE(
      std::abs(factor.measuredBaselineRatio() - geometry.measured_ratio) <
              kTolerance &&
          factor.firstBodyTCam().equals(geometry.first_body_T_cam) &&
          factor.middleBodyTCam().equals(geometry.middle_body_T_cam) &&
          factor.lastBodyTCam().equals(geometry.last_body_T_cam),
      "measurement and extrinsic accessors preserve constructor inputs");
  const auto cloned = factor.clone();
  EXPECT_TRUE(factor.equals(*cloned), "clone and equality preserve the factor");
  return EXIT_SUCCESS;
}

int testSignedPerturbationsAndCommonScale() {
  const Geometry geometry;
  const auto factor = geometry.factor();

  const gtsam::Point3 extended_first =
      geometry.first_center -
      0.35 * (geometry.middle_center - geometry.first_center);
  const gtsam::Pose3 first_perturbed =
      bodyPoseForCameraCenter(extended_first,
                              geometry.first_body.rotation(),
                              geometry.first_body_T_cam);
  EXPECT_TRUE(
      factor.evaluateError(
          first_perturbed, geometry.middle_body, geometry.last_body)(0) < 0.0,
      "increasing only the first baseline gives a negative residual");

  const gtsam::Point3 extended_last =
      geometry.last_center +
      0.4 * (geometry.last_center - geometry.middle_center);
  const gtsam::Pose3 last_perturbed = bodyPoseForCameraCenter(
      extended_last, geometry.last_body.rotation(), geometry.last_body_T_cam);
  EXPECT_TRUE(
      factor.evaluateError(
          geometry.first_body, geometry.middle_body, last_perturbed)(0) > 0.0,
      "increasing only the second baseline gives a positive residual");

  for (const double scale : {0.15, 3.5, 20.0}) {
    const gtsam::Pose3 first =
        bodyPoseForCameraCenter(scale * geometry.first_center,
                                geometry.first_body.rotation(),
                                geometry.first_body_T_cam);
    const gtsam::Pose3 middle =
        bodyPoseForCameraCenter(scale * geometry.middle_center,
                                geometry.middle_body.rotation(),
                                geometry.middle_body_T_cam);
    const gtsam::Pose3 last =
        bodyPoseForCameraCenter(scale * geometry.last_center,
                                geometry.last_body.rotation(),
                                geometry.last_body_T_cam);
    EXPECT_TRUE(factor.evaluateError(first, middle, last).norm() < kTolerance,
                "common positive baseline scale leaves the residual unchanged");
  }
  return EXIT_SUCCESS;
}

int testAnalyticJacobians() {
  const Geometry geometry;
  const auto factor = geometry.factor();
  const gtsam::Pose3 first = geometry.first_body.retract(
      (gtsam::Vector6() << 0.01, -0.02, 0.015, 0.02, 0.01, -0.03).finished());
  const gtsam::Pose3 middle = geometry.middle_body.retract(
      (gtsam::Vector6() << -0.015, 0.012, 0.018, -0.025, 0.035, 0.02)
          .finished());
  const gtsam::Pose3 last = geometry.last_body.retract(
      (gtsam::Vector6() << 0.013, 0.009, -0.02, 0.015, -0.02, 0.025)
          .finished());

  gtsam::Matrix H_first;
  gtsam::Matrix H_middle;
  gtsam::Matrix H_last;
  factor.evaluateError(first, middle, last, &H_first, &H_middle, &H_last);
  const gtsam::Matrix numerical_first =
      gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          [&factor, &middle, &last](const gtsam::Pose3& value) {
            return factor.evaluateError(value, middle, last);
          },
          first,
          1e-6);
  const gtsam::Matrix numerical_middle =
      gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          [&factor, &first, &last](const gtsam::Pose3& value) {
            return factor.evaluateError(first, value, last);
          },
          middle,
          1e-6);
  const gtsam::Matrix numerical_last =
      gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          [&factor, &first, &middle](const gtsam::Pose3& value) {
            return factor.evaluateError(first, middle, value);
          },
          last,
          1e-6);
  EXPECT_TRUE((H_first - numerical_first).norm() < 2e-6 &&
                  (H_middle - numerical_middle).norm() < 2e-6 &&
                  (H_last - numerical_last).norm() < 2e-6,
              "analytic Jacobians match numerical derivatives");
  return EXIT_SUCCESS;
}

int testGlobalRigidTransformInvariance() {
  const Geometry geometry;
  const auto factor = geometry.factor();
  const gtsam::Pose3 global_T_world(gtsam::Rot3::RzRyRx(0.7, -0.35, 0.24),
                                    gtsam::Point3(12.0, -8.0, 3.0));
  const double original = factor.evaluateError(
      geometry.first_body, geometry.middle_body, geometry.last_body)(0);
  const double transformed =
      factor.evaluateError(global_T_world.compose(geometry.first_body),
                           global_T_world.compose(geometry.middle_body),
                           global_T_world.compose(geometry.last_body))(0);
  EXPECT_TRUE(std::abs(original - transformed) < kTolerance,
              "global translation and rigid rotation preserve the ratio");
  return EXIT_SUCCESS;
}

int testInvalidInputs() {
  const Geometry geometry;
  const auto make_factor = [&geometry](const double ratio,
                                       const gtsam::SharedNoiseModel& noise,
                                       const gtsam::Pose3& first_extrinsic) {
    return VIO::CameraAwareBaselineRatioFactor(1u,
                                               2u,
                                               3u,
                                               first_extrinsic,
                                               geometry.middle_body_T_cam,
                                               geometry.last_body_T_cam,
                                               ratio,
                                               noise);
  };

  bool invalid_ratio = false;
  try {
    static_cast<void>(make_factor(0.0, model(), geometry.first_body_T_cam));
  } catch (const std::invalid_argument&) {
    invalid_ratio = true;
  }
  EXPECT_TRUE(invalid_ratio, "non-positive measured ratio is rejected");

  bool nonfinite_ratio = false;
  try {
    static_cast<void>(make_factor(std::numeric_limits<double>::quiet_NaN(),
                                  model(),
                                  geometry.first_body_T_cam));
  } catch (const std::invalid_argument&) {
    nonfinite_ratio = true;
  }
  EXPECT_TRUE(nonfinite_ratio, "non-finite measured ratio is rejected");

  bool invalid_dimension = false;
  try {
    static_cast<void>(make_factor(
        geometry.measured_ratio, model(2u), geometry.first_body_T_cam));
  } catch (const std::invalid_argument&) {
    invalid_dimension = true;
  }
  EXPECT_TRUE(invalid_dimension, "invalid noise dimension is rejected");

  bool invalid_extrinsic = false;
  try {
    const gtsam::Pose3 invalid(
        geometry.first_body_T_cam.rotation(),
        gtsam::Point3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    static_cast<void>(make_factor(geometry.measured_ratio, model(), invalid));
  } catch (const std::invalid_argument&) {
    invalid_extrinsic = true;
  }
  EXPECT_TRUE(invalid_extrinsic, "non-finite extrinsic is rejected");

  const auto factor = geometry.factor();
  bool invalid_body_pose = false;
  try {
    const gtsam::Pose3 invalid(
        geometry.first_body.rotation(),
        gtsam::Point3(std::numeric_limits<double>::infinity(), 0.0, 0.0));
    static_cast<void>(factor.evaluateError(
        invalid, geometry.middle_body, geometry.last_body));
  } catch (const std::domain_error&) {
    invalid_body_pose = true;
  }
  EXPECT_TRUE(invalid_body_pose, "non-finite optimized pose is rejected");

  bool zero_first = false;
  try {
    const gtsam::Pose3 coincident_first =
        bodyPoseForCameraCenter(geometry.middle_center,
                                geometry.first_body.rotation(),
                                geometry.first_body_T_cam);
    static_cast<void>(factor.evaluateError(
        coincident_first, geometry.middle_body, geometry.last_body));
  } catch (const std::domain_error&) {
    zero_first = true;
  }
  EXPECT_TRUE(zero_first, "zero first predicted baseline is rejected");

  bool zero_second = false;
  try {
    const gtsam::Pose3 coincident_last =
        bodyPoseForCameraCenter(geometry.middle_center,
                                geometry.last_body.rotation(),
                                geometry.last_body_T_cam);
    static_cast<void>(factor.evaluateError(
        geometry.first_body, geometry.middle_body, coincident_last));
  } catch (const std::domain_error&) {
    zero_second = true;
  }
  EXPECT_TRUE(zero_second, "zero second predicted baseline is rejected");
  return EXIT_SUCCESS;
}

}  // namespace

int main() {
  int status = EXIT_SUCCESS;
  status |= testZeroErrorAndAccessors();
  status |= testSignedPerturbationsAndCommonScale();
  status |= testAnalyticJacobians();
  status |= testGlobalRigidTransformInvariance();
  status |= testInvalidInputs();
  if (status == EXIT_SUCCESS) {
    std::cout << "All camera-aware baseline-ratio factor tests PASSED.\n";
  }
  return status;
}
