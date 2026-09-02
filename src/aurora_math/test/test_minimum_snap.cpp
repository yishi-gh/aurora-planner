#include "aurora_math/minimum_snap.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using aurora::math::MinimumSnapTrajectory;

MinimumSnapTrajectory::WaypointMatrix makeWaypoints() {
  MinimumSnapTrajectory::WaypointMatrix waypoints(3, 4);
  waypoints << 0.0, 1.0, 3.0, 5.0,
               0.0, 2.0, 1.0, 4.0,
               1.0, 1.5, 2.0, 3.0;
  return waypoints;
}

void expectVectorNear(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected,
                      double tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(actual(axis), expected(axis), tolerance);
  }
}

}  // namespace

TEST(MinimumSnapTrajectory, RejectsInvalidInputs) {
  MinimumSnapTrajectory::WaypointMatrix too_few(3, 1);
  too_few.setZero();
  Eigen::VectorXd one_time(1);
  one_time(0) = 1.0;
  EXPECT_THROW(MinimumSnapTrajectory::fromWaypoints(
                   too_few, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), one_time),
               std::invalid_argument);

  auto waypoints = makeWaypoints();
  Eigen::VectorXd times(3);
  times << 1.0, 0.0, 1.0;
  EXPECT_THROW(MinimumSnapTrajectory::fromWaypoints(
                   waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times),
               std::invalid_argument);

  times << 1.0, 1.5, 1.0;
  waypoints(1, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MinimumSnapTrajectory::fromWaypoints(
                   waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times),
               std::invalid_argument);
  EXPECT_THROW(MinimumSnapTrajectory::oneSegment(
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0),
               std::invalid_argument);
}

TEST(MinimumSnapTrajectory, SatisfiesOneSegmentBoundaryStates) {
  const Eigen::Vector3d start_position(0.0, 1.0, 2.0);
  const Eigen::Vector3d start_velocity(0.3, -0.2, 0.1);
  const Eigen::Vector3d start_acceleration(-0.4, 0.5, 0.2);
  const Eigen::Vector3d end_position(5.0, -1.0, 3.0);
  const Eigen::Vector3d end_velocity(-0.1, 0.2, 0.4);
  const Eigen::Vector3d end_acceleration(0.6, -0.3, -0.2);
  const MinimumSnapTrajectory trajectory = MinimumSnapTrajectory::oneSegment(
      start_position, start_velocity, start_acceleration, end_position, end_velocity,
      end_acceleration, 2.0);

  EXPECT_EQ(trajectory.segmentCount(), 1);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 2.0);
  expectVectorNear(trajectory.evaluate(0.0), start_position, 1e-10);
  expectVectorNear(trajectory.evaluate(0.0, 1), start_velocity, 1e-10);
  expectVectorNear(trajectory.evaluate(0.0, 2), start_acceleration, 1e-10);
  expectVectorNear(trajectory.evaluate(2.0), end_position, 1e-10);
  expectVectorNear(trajectory.evaluate(2.0, 1), end_velocity, 1e-10);
  expectVectorNear(trajectory.evaluate(2.0, 2), end_acceleration, 1e-10);
  EXPECT_GE(trajectory.jerkCost(), 0.0);
  EXPECT_TRUE(trajectory.evaluate(-1.0).isApprox(start_position, 1e-10));
  EXPECT_TRUE(trajectory.evaluate(3.0).isApprox(end_position, 1e-10));
}

TEST(MinimumSnapTrajectory, InterpolatesWaypointsAndMaintainsContinuity) {
  const auto waypoints = makeWaypoints();
  Eigen::VectorXd times(3);
  times << 1.0, 1.5, 0.75;
  const MinimumSnapTrajectory trajectory = MinimumSnapTrajectory::fromWaypoints(
      waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), times);

  EXPECT_EQ(trajectory.segmentCount(), 3);
  EXPECT_DOUBLE_EQ(trajectory.duration(), 3.25);
  double boundary_time = 0.0;
  for (int waypoint = 0; waypoint < waypoints.cols(); ++waypoint) {
    expectVectorNear(trajectory.evaluate(boundary_time), waypoints.col(waypoint), 1e-9);
    if (waypoint + 1 < waypoints.cols()) {
      boundary_time += times(waypoint);
    }
  }

  constexpr double epsilon = 1e-7;
  boundary_time = times(0);
  for (int boundary = 0; boundary < times.size() - 1; ++boundary) {
    for (int derivative_order = 0; derivative_order <= 3; ++derivative_order) {
      expectVectorNear(trajectory.evaluate(boundary_time - epsilon, derivative_order),
                       trajectory.evaluate(boundary_time + epsilon, derivative_order), 1e-4);
    }
    boundary_time += times(boundary + 1);
  }
}

TEST(MinimumSnapTrajectory, HandlesDerivativeOrdersAndInvalidTimes) {
  const auto trajectory = MinimumSnapTrajectory::oneSegment(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);

  EXPECT_TRUE(trajectory.evaluate(0.5, 6).isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_TRUE(trajectory.evaluate(0.5, 10).isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_THROW(trajectory.evaluate(0.5, -1), std::invalid_argument);
  EXPECT_THROW(trajectory.evaluate(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
