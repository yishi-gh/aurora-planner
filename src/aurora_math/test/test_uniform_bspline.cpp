#include "aurora_math/uniform_bspline.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace {

using aurora::math::UniformBspline;
using aurora::math::UniformBsplineKnotMode;

UniformBspline::ControlPointMatrix makeControlPoints() {
  UniformBspline::ControlPointMatrix points(3, 7);
  points << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
            0.0, 0.5, 1.5, 1.0, 2.0, 1.0, 2.5,
            1.0, 1.0, 1.2, 1.8, 1.5, 2.0, 2.0;
  return points;
}

void expectVectorNear(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected,
                      double tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(actual(axis), expected(axis), tolerance);
  }
}

}  // namespace

TEST(UniformBspline, RejectsInvalidInputs) {
  UniformBspline::ControlPointMatrix too_few(3, 3);
  too_few.setZero();
  EXPECT_THROW(UniformBspline(too_few, 0.1), std::invalid_argument);

  UniformBspline::ControlPointMatrix points = makeControlPoints();
  EXPECT_THROW(UniformBspline(points, 0.0), std::invalid_argument);
  EXPECT_THROW(UniformBspline(points, -0.1), std::invalid_argument);
  points(1, 2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(UniformBspline(points, 0.1), std::invalid_argument);
}

TEST(UniformBspline, DurationEndpointsAndTimeClamping) {
  const UniformBspline spline(makeControlPoints(), 0.25);
  EXPECT_EQ(spline.degree(), 3);
  EXPECT_EQ(spline.controlPointCount(), 7);
  EXPECT_DOUBLE_EQ(spline.dt(), 0.25);
  EXPECT_DOUBLE_EQ(spline.duration(), 1.0);

  expectVectorNear(spline.evaluate(0.0), spline.controlPoints().col(0), 1e-12);
  expectVectorNear(spline.evaluate(spline.duration()), spline.controlPoints().col(6), 1e-12);
  expectVectorNear(spline.evaluate(-10.0), spline.evaluate(0.0), 1e-12);
  expectVectorNear(spline.evaluate(10.0), spline.evaluate(spline.duration()), 1e-12);
  expectVectorNear(spline.evaluate(-10.0, 1), spline.evaluate(0.0, 1), 1e-12);
  expectVectorNear(spline.evaluate(10.0, 2), spline.evaluate(spline.duration(), 2), 1e-12);
  EXPECT_THROW(spline.evaluate(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(spline.evaluate(0.1, -1), std::invalid_argument);
}

TEST(UniformBspline, BasisFunctionsFormPartitionOfUnity) {
  const UniformBspline spline(makeControlPoints(), 0.25);
  for (int sample = 0; sample <= 100; ++sample) {
    const double time = spline.duration() * static_cast<double>(sample) / 100.0;
    const Eigen::VectorXd basis = spline.basisFunctions(time);
    EXPECT_EQ(basis.size(), spline.controlPointCount());
    EXPECT_NEAR(basis.sum(), 1.0, 1e-12);
    for (int i = 0; i < basis.size(); ++i) {
      EXPECT_GE(basis(i), -1e-12);
    }
  }
}

TEST(UniformBspline, BasisFunctionsMatchCurveAtBothEndpoints) {
  const UniformBspline::ControlPointMatrix control_points = makeControlPoints();
  const UniformBspline clamped(control_points, 0.25);
  const UniformBspline ego(control_points, 0.25, UniformBsplineKnotMode::EGO_UNCLAMPED);
  for (const UniformBspline &spline : {clamped, ego}) {
    const Eigen::VectorXd start_basis = spline.basisFunctions(0.0);
    const Eigen::VectorXd end_basis = spline.basisFunctions(spline.duration());
    expectVectorNear(spline.evaluate(0.0), control_points * start_basis, 1e-12);
    expectVectorNear(spline.evaluate(spline.duration()), control_points * end_basis, 1e-12);
  }
}

TEST(UniformBspline, DerivativeSplineMatchesEvaluation) {
  const UniformBspline spline(makeControlPoints(), 0.25);
  for (int order = 0; order <= 3; ++order) {
    const UniformBspline derived = spline.derivative(order);
    EXPECT_EQ(derived.degree(), spline.degree() - order);
    EXPECT_EQ(derived.controlPointCount(), spline.controlPointCount() - order);
    EXPECT_DOUBLE_EQ(derived.duration(), spline.duration());
    for (int sample = 1; sample < 20; ++sample) {
      const double time = spline.duration() * static_cast<double>(sample) / 20.0;
      expectVectorNear(derived.evaluate(time), spline.evaluate(time, order), 1e-10);
    }
  }
  EXPECT_THROW(spline.derivative(-1), std::invalid_argument);
  EXPECT_THROW(spline.derivative(4), std::invalid_argument);
  expectVectorNear(spline.evaluate(0.5, 4), Eigen::Vector3d::Zero(), 1e-12);
}

TEST(UniformBspline, SupportsMinimumCubicControlPointCount) {
  UniformBspline::ControlPointMatrix points(3, 4);
  points << 0.0, 1.0, 2.0, 4.0,
            0.0, 1.0, 1.0, 2.0,
            0.0, 0.0, 1.0, 1.0;
  const UniformBspline spline(points, 0.5);
  const UniformBspline jerk = spline.derivative(3);
  EXPECT_EQ(jerk.degree(), 0);
  EXPECT_EQ(jerk.controlPointCount(), 1);
  EXPECT_DOUBLE_EQ(jerk.duration(), spline.duration());
  EXPECT_TRUE(jerk.evaluate(0.0).allFinite());
  EXPECT_TRUE(jerk.evaluate(spline.duration()).allFinite());
  expectVectorNear(jerk.evaluate(0.25), spline.evaluate(0.25, 3), 1e-10);
}

TEST(UniformBspline, DerivativesAreContinuousAtInternalKnots) {
  const UniformBspline spline(makeControlPoints(), 0.25);
  constexpr double epsilon = 1e-8;
  for (int knot = 1; knot < spline.controlPointCount() - spline.degree(); ++knot) {
    const double time = static_cast<double>(knot) * spline.dt();
    for (int order = 0; order <= 2; ++order) {
      expectVectorNear(spline.evaluate(time - epsilon, order),
                       spline.evaluate(time + epsilon, order), 1e-5);
    }
  }
}

TEST(UniformBspline, DerivativeAgreesWithFiniteDifference) {
  const UniformBspline spline(makeControlPoints(), 0.25);
  constexpr double h = 1e-6;
  for (const double time : {0.18, 0.37, 0.63, 0.82}) {
    const Eigen::Vector3d first_difference =
        (spline.evaluate(time + h) - spline.evaluate(time - h)) / (2.0 * h);
    const Eigen::Vector3d second_difference =
        (spline.evaluate(time + h) - 2.0 * spline.evaluate(time) + spline.evaluate(time - h)) /
        (h * h);
    expectVectorNear(spline.evaluate(time, 1), first_difference, 1e-5);
    expectVectorNear(spline.evaluate(time, 2), second_difference, 1e-3);
  }
}

TEST(UniformBspline, EgoParameterizationProducesExpectedShapeAndMode) {
  const std::vector<Eigen::Vector3d> points = {
      {0.0, 0.0, 1.0}, {1.0, 2.0, 1.5}, {2.0, 4.0, 2.0},
      {3.0, 6.0, 2.5}, {4.0, 8.0, 3.0}, {5.0, 10.0, 3.5}};
  const double dt = 0.5;
  const Eigen::Vector3d velocity(2.0, 4.0, 1.0);
  const Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

  const UniformBspline::ControlPointMatrix control_points =
      UniformBspline::parameterizeToControlPoints(points, dt, velocity, velocity,
                                                   acceleration, acceleration);
  ASSERT_EQ(control_points.rows(), 3);
  ASSERT_EQ(control_points.cols(), static_cast<int>(points.size()) + 2);

  const UniformBspline spline(control_points, dt, UniformBsplineKnotMode::EGO_UNCLAMPED);
  EXPECT_EQ(spline.knotMode(), UniformBsplineKnotMode::EGO_UNCLAMPED);
  EXPECT_DOUBLE_EQ(spline.duration(), (points.size() - 1) * dt);
  for (std::size_t i = 0; i < points.size(); ++i) {
    expectVectorNear(spline.evaluate(static_cast<double>(i) * dt), points[i], 1e-10);
  }
  expectVectorNear(spline.evaluate(0.0, 1), velocity, 1e-10);
  expectVectorNear(spline.evaluate(spline.duration(), 1), velocity, 1e-10);
  expectVectorNear(spline.evaluate(0.0, 2), acceleration, 1e-10);
  expectVectorNear(spline.evaluate(spline.duration(), 2), acceleration, 1e-10);
}

TEST(UniformBspline, EgoParameterizationRejectsInvalidInputs) {
  const std::vector<Eigen::Vector3d> points(3, Eigen::Vector3d::Zero());
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  EXPECT_THROW(UniformBspline::parameterizeToControlPoints(points, 0.1, zero, zero, zero, zero),
               std::invalid_argument);

  const std::vector<Eigen::Vector3d> valid_points(4, Eigen::Vector3d::Zero());
  EXPECT_THROW(UniformBspline::parameterizeToControlPoints(valid_points, 0.0, zero, zero, zero, zero),
               std::invalid_argument);

  std::vector<Eigen::Vector3d> non_finite_points = valid_points;
  non_finite_points[1](0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(UniformBspline::parameterizeToControlPoints(non_finite_points, 0.1, zero, zero, zero,
                                                            zero),
               std::invalid_argument);

  Eigen::Vector3d non_finite_velocity = zero;
  non_finite_velocity(2) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(UniformBspline::parameterizeToControlPoints(valid_points, 0.1, non_finite_velocity,
                                                            zero, zero, zero),
               std::invalid_argument);
}

TEST(UniformBspline, EgoParameterizationKeepsEndpointStateForTurningGuide) {
  const std::vector<Eigen::Vector3d> points = {
      {-2.0, -1.0, 1.0}, {-1.0, 1.0, 1.5}, {0.0, -0.5, 2.0},
      {1.0, 1.5, 1.0}, {2.0, 0.0, 1.5}, {3.0, 2.0, 1.0}};
  const double dt = 0.4;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const auto control_points = UniformBspline::parameterizeToControlPoints(
      points, dt, zero, zero, zero, zero);
  const UniformBspline spline(control_points, dt, UniformBsplineKnotMode::EGO_UNCLAMPED);

  expectVectorNear(spline.evaluate(0.0), points.front(), 1e-10);
  expectVectorNear(spline.evaluate(spline.duration()), points.back(), 1e-10);
  expectVectorNear(spline.evaluate(0.0, 1), zero, 1e-10);
  expectVectorNear(spline.evaluate(spline.duration(), 1), zero, 1e-10);
  expectVectorNear(spline.evaluate(0.0, 2), zero, 1e-10);
  expectVectorNear(spline.evaluate(spline.duration(), 2), zero, 1e-10);
}
