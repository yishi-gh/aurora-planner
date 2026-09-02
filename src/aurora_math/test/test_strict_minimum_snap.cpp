#include "aurora_math/strict_minimum_snap.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

using aurora::math::StrictMinimumSnapTrajectory;

void expectVectorNear(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected,
                      double tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(actual(axis), expected(axis), tolerance);
  }
}

StrictMinimumSnapTrajectory::WaypointMatrix makeWaypoints() {
  StrictMinimumSnapTrajectory::WaypointMatrix waypoints(3, 3);
  waypoints << 0.0, 1.0, 3.0,
               0.0, 2.0, 1.0,
               1.0, 1.5, 2.0;
  return waypoints;
}

}  // namespace

TEST(StrictMinimumSnapTrajectory, SatisfiesAllOneSegmentBoundaryStates) {
  const Eigen::Vector3d start_position(0.0, 1.0, 2.0);
  const Eigen::Vector3d start_velocity(0.3, -0.2, 0.1);
  const Eigen::Vector3d start_acceleration(-0.4, 0.5, 0.2);
  const Eigen::Vector3d start_jerk(0.2, -0.1, 0.3);
  const Eigen::Vector3d end_position(5.0, -1.0, 3.0);
  const Eigen::Vector3d end_velocity(-0.1, 0.2, 0.4);
  const Eigen::Vector3d end_acceleration(0.6, -0.3, -0.2);
  const Eigen::Vector3d end_jerk(-0.2, 0.4, 0.1);
  const auto trajectory = StrictMinimumSnapTrajectory::oneSegment(
      start_position, start_velocity, start_acceleration, start_jerk, end_position,
      end_velocity, end_acceleration, end_jerk, 2.0);

  EXPECT_EQ(trajectory.segmentCount(), 1);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 2.0);
  expectVectorNear(trajectory.evaluate(0.0, 0), start_position, 1e-9);
  expectVectorNear(trajectory.evaluate(0.0, 1), start_velocity, 1e-9);
  expectVectorNear(trajectory.evaluate(0.0, 2), start_acceleration, 1e-9);
  expectVectorNear(trajectory.evaluate(0.0, 3), start_jerk, 1e-9);
  expectVectorNear(trajectory.evaluate(2.0, 0), end_position, 1e-9);
  expectVectorNear(trajectory.evaluate(2.0, 1), end_velocity, 1e-9);
  expectVectorNear(trajectory.evaluate(2.0, 2), end_acceleration, 1e-9);
  expectVectorNear(trajectory.evaluate(2.0, 3), end_jerk, 1e-9);
  EXPECT_GE(trajectory.snapCost(), 0.0);
  EXPECT_TRUE(trajectory.evaluate(0.5, 8).isApprox(Eigen::Vector3d::Zero(), 1e-12));
}

TEST(StrictMinimumSnapTrajectory, MaintainsContinuityThroughSixthDerivative) {
  const auto waypoints = makeWaypoints();
  Eigen::VectorXd times(2);
  times << 0.8, 1.4;
  const auto trajectory = StrictMinimumSnapTrajectory::fromWaypoints(
      waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times);

  const double boundary = times(0);
  for (int derivative_order = 0; derivative_order <= 6; ++derivative_order) {
    const Eigen::Vector3d left = trajectory.evaluate(boundary - 1e-8, derivative_order);
    const Eigen::Vector3d right = trajectory.evaluate(boundary + 1e-8, derivative_order);
    EXPECT_LE((left - right).norm(), 1e-5 * (1.0 + std::max(left.norm(), right.norm())))
        << "derivative order " << derivative_order;
  }
  expectVectorNear(trajectory.evaluate(boundary), waypoints.col(1), 1e-9);
  EXPECT_TRUE(std::isfinite(trajectory.snapCost()));
}

TEST(StrictMinimumSnapTrajectory, ClampsTimeAndRejectsInvalidInputs) {
  const auto trajectory = StrictMinimumSnapTrajectory::oneSegment(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);
  expectVectorNear(trajectory.evaluate(-1.0), Eigen::Vector3d::Zero(), 1e-10);
  expectVectorNear(trajectory.evaluate(2.0), Eigen::Vector3d::Ones(), 1e-10);
  EXPECT_THROW(trajectory.evaluate(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(trajectory.evaluate(0.5, -1), std::invalid_argument);
  EXPECT_THROW(StrictMinimumSnapTrajectory::oneSegment(
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0),
               std::invalid_argument);
}

TEST(StrictMinimumSnapTrajectory, PreservesGeometryAndScalesPhysicalDerivatives) {
  const auto fast = StrictMinimumSnapTrajectory::oneSegment(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);
  const auto slow = StrictMinimumSnapTrajectory::oneSegment(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 2.0);

  for (double normalized_time : {0.1, 0.35, 0.7, 0.95}) {
    expectVectorNear(slow.evaluate(2.0 * normalized_time, 0),
                     fast.evaluate(normalized_time, 0), 1e-9);
    expectVectorNear(slow.evaluate(2.0 * normalized_time, 1),
                     0.5 * fast.evaluate(normalized_time, 1), 1e-8);
    expectVectorNear(slow.evaluate(2.0 * normalized_time, 2),
                     0.25 * fast.evaluate(normalized_time, 2), 1e-7);
  }
  EXPECT_NEAR(slow.snapCost(), fast.snapCost() / 128.0, 1e-8);
}

TEST(StrictMinimumSnapTrajectory, HandlesWidelyDifferentSegmentTimes) {
  const auto waypoints = makeWaypoints();
  Eigen::VectorXd times(2);
  times << 1e-3, 1e3;
  const auto trajectory = StrictMinimumSnapTrajectory::fromWaypoints(
      waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times);

  EXPECT_TRUE(std::isfinite(trajectory.duration()));
  EXPECT_DOUBLE_EQ(trajectory.duration(), 1000.001);
  for (const auto &coefficients : trajectory.coefficients()) {
    EXPECT_TRUE(coefficients.allFinite());
  }
  expectVectorNear(trajectory.evaluate(0.0), waypoints.col(0), 1e-8);
  expectVectorNear(trajectory.evaluate(times(0)), waypoints.col(1), 1e-7);
  expectVectorNear(trajectory.evaluate(trajectory.duration()), waypoints.col(2), 1e-7);
}
