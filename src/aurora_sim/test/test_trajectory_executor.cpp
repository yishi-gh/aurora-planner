#include "aurora_sim/trajectory_executor.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace {

aurora::simulation::ExecutorSegment makeSegment(double start_stamp, double source_start_time = 0.0,
                                                double duration = 1.0) {
  aurora::math::UniformBspline::ControlPointMatrix points(3, 7);
  points << -4.0, -4.0, -3.5, -2.0, -1.0, 0.0, 1.0,
      0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.0,
      1.0, 1.0, 1.0, 2.5, 3.0, 3.0, 3.0;
  return {start_stamp, source_start_time, duration,
          aurora::math::UniformBspline(points, 0.25)};
}

aurora::simulation::ExecutorTrajectory makeTrajectory(bool validated = true,
                                                       bool safety_accepted = true) {
  aurora::simulation::ExecutorTrajectory trajectory;
  trajectory.trajectory_id = 42U;
  trajectory.validated = validated;
  trajectory.safety_accepted = safety_accepted;
  trajectory.segments.push_back(makeSegment(0.0));
  return trajectory;
}

}  // namespace

TEST(TrajectoryExecutor, RejectsUnvalidatedAndUnsafeCommands) {
  aurora::simulation::TrajectoryExecutor executor;
  auto unvalidated = makeTrajectory(false, true);
  auto result = executor.accept(unvalidated, 0.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::simulation::ExecutionStatus::REJECTED_UNVALIDATED);

  auto unsafe = makeTrajectory(true, false);
  result = executor.accept(unsafe, 0.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::simulation::ExecutionStatus::REJECTED_UNSAFE);
}

TEST(TrajectoryExecutor, ExecutesValidatedThreeDimensionalTrajectory) {
  aurora::simulation::FlightState initial;
  initial.position = {-4.0, 0.0, 1.0};
  aurora::simulation::TrajectoryExecutor executor({}, initial);
  const auto trajectory = makeTrajectory();
  const auto accepted = executor.accept(trajectory, 0.0);
  ASSERT_TRUE(accepted.accepted);

  const auto output = executor.update(0.5);
  EXPECT_TRUE(output.active);
  EXPECT_EQ(output.status, aurora::simulation::ExecutionStatus::ACTIVE);
  EXPECT_GT(output.desired.position.z(), 1.0);
  EXPECT_GT(output.desired.position.x(), -4.0);
  EXPECT_TRUE(output.state.position.allFinite());
  EXPECT_TRUE(output.state.velocity.allFinite());

  const auto completed = executor.update(1.0);
  EXPECT_FALSE(completed.active);
  EXPECT_EQ(completed.status, aurora::simulation::ExecutionStatus::COMPLETED);
}

TEST(TrajectoryExecutor, RejectsGapsAndExpiredCommands) {
  aurora::simulation::TrajectoryExecutor executor;
  auto trajectory = makeTrajectory();
  trajectory.segments.push_back(makeSegment(2.0));
  auto result = executor.accept(trajectory, 0.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::simulation::ExecutionStatus::REJECTED_INVALID);

  result = executor.accept(makeTrajectory(), 2.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::simulation::ExecutionStatus::REJECTED_INVALID);
}

TEST(TrajectoryExecutor, ClockFaultsFailClosedAndRequireReset) {
  aurora::simulation::ExecutorOptions options;
  options.max_update_gap = 0.2;
  aurora::simulation::TrajectoryExecutor executor(options);
  ASSERT_TRUE(executor.accept(makeTrajectory(), 0.0).accepted);

  auto output = executor.update(-0.1);
  EXPECT_EQ(output.status, aurora::simulation::ExecutionStatus::TIME_ROLLBACK);
  EXPECT_TRUE(executor.emergencyStopActive());
  EXPECT_FALSE(executor.accept(makeTrajectory(), 0.0).accepted);

  executor.setEmergencyStop(false);
  ASSERT_TRUE(executor.accept(makeTrajectory(), 0.0).accepted);
  output = executor.update(0.3);
  EXPECT_EQ(output.status, aurora::simulation::ExecutionStatus::TIME_GAP);
  EXPECT_FALSE(output.active);
  EXPECT_TRUE(executor.emergencyStopActive());
  EXPECT_FALSE(executor.accept(makeTrajectory(), 0.3).accepted);
}

TEST(TrajectoryExecutor, EmergencyStopClearsActiveTrajectory) {
  aurora::simulation::TrajectoryExecutor executor;
  ASSERT_TRUE(executor.accept(makeTrajectory(), 0.0).accepted);
  executor.setEmergencyStop(true);
  EXPECT_TRUE(executor.emergencyStopActive());
  EXPECT_EQ(executor.activeTrajectoryId(), 42U);
  const auto output = executor.update(0.1);
  EXPECT_EQ(output.status, aurora::simulation::ExecutionStatus::EMERGENCY_STOP);
  EXPECT_FALSE(output.active);
  EXPECT_TRUE(executor.state().velocity.isZero(1e-12));
}
