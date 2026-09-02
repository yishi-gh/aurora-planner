#include "aurora_planner_core/static_safety_gate.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace {

using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;
using aurora::planner::PlannedTrajectory;
using aurora::planner::SafetyGateStatus;
using aurora::planner::StaticSafetyGate;
using aurora::planner::StaticSafetyGateOptions;
using aurora::planner::TrajectorySegment;
using aurora::math::UniformBspline;

VoxelMap makeFreeMap() {
  VoxelMapConfig config;
  config.origin = Eigen::Vector3d(-8.0, -5.0, 0.0);
  config.dimensions = Eigen::Vector3i(32, 20, 12);
  config.resolution = 0.5;
  VoxelMap map(config);
  for (int x = 0; x < config.dimensions.x(); ++x) {
    for (int y = 0; y < config.dimensions.y(); ++y) {
      for (int z = 0; z < config.dimensions.z(); ++z) {
        map.setOccupancy({x, y, z}, 0.0);
      }
    }
  }
  return map;
}

UniformBspline makeLineSpline(double y = 0.0, double dt = 0.5) {
  UniformBspline::ControlPointMatrix control_points(3, 7);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) = Eigen::Vector3d(-1.2 + 0.4 * static_cast<double>(index), y, 1.0);
  }
  return UniformBspline(control_points, dt);
}

PlannedTrajectory makeCandidate(double y = 0.0) {
  const UniformBspline spline = makeLineSpline(y);
  PlannedTrajectory trajectory;
  trajectory.trajectory_id = 21U;
  trajectory.segments.push_back(TrajectorySegment{10.0, 0.0, spline.duration(), spline});
  return trajectory;
}

PlannedTrajectory makeSplitCandidate() {
  const UniformBspline spline = makeLineSpline();
  PlannedTrajectory trajectory;
  trajectory.trajectory_id = 22U;
  trajectory.segments.push_back(TrajectorySegment{10.0, 0.0, 0.75, spline});
  trajectory.segments.push_back(TrajectorySegment{10.75, 0.75, spline.duration() - 0.75, spline});
  return trajectory;
}

StaticSafetyGate makeGate() {
  StaticSafetyGateOptions options;
  options.validation.samples_per_span = 16;
  options.validation.max_velocity = 3.0;
  options.validation.max_acceleration = 6.0;
  return StaticSafetyGate(options);
}

}  // namespace

TEST(StaticSafetyGate, AcceptsContiguousCandidateAndReportsSafeCurrentFallback) {
  VoxelMap map = makeFreeMap();
  PlannedTrajectory candidate = makeSplitCandidate();
  PlannedTrajectory current = makeCandidate();
  current.validated = true;

  const auto result = makeGate().evaluate(map, candidate, 10.0, current);

  EXPECT_TRUE(result.accepted) << result.detail;
  EXPECT_EQ(result.status, SafetyGateStatus::ACCEPTED);
  EXPECT_EQ(result.checked_segments, 2U);
  EXPECT_TRUE(result.current_trajectory_available);
  EXPECT_TRUE(result.current_state_checked);
  EXPECT_TRUE(result.current_state_continuous);
  EXPECT_TRUE(result.current_trajectory_fallback_available);
  EXPECT_EQ(result.failed_segment, std::numeric_limits<std::size_t>::max());
}

TEST(StaticSafetyGate, RejectsTimeGapBeforeStaticValidation) {
  VoxelMap map = makeFreeMap();
  const UniformBspline spline = makeLineSpline();
  PlannedTrajectory candidate;
  candidate.segments.push_back(TrajectorySegment{10.0, 0.0, 0.75, spline});
  candidate.segments.push_back(TrajectorySegment{10.9, 0.75, spline.duration() - 0.75, spline});

  const auto result = makeGate().evaluate(map, candidate, 10.0);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, SafetyGateStatus::TIME_DISCONTINUITY);
  EXPECT_EQ(result.checked_samples, 0U);
}

TEST(StaticSafetyGate, RejectsStateDiscontinuityAtSegmentBoundary) {
  VoxelMap map = makeFreeMap();
  const UniformBspline first_spline = makeLineSpline();
  const UniformBspline second_spline = makeLineSpline(0.4);
  PlannedTrajectory candidate;
  candidate.segments.push_back(TrajectorySegment{10.0, 0.0, 0.75, first_spline});
  candidate.segments.push_back(
      TrajectorySegment{10.75, 0.0, second_spline.duration(), second_spline});

  const auto result = makeGate().evaluate(map, candidate, 10.0);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, SafetyGateStatus::STATE_DISCONTINUITY);
  EXPECT_EQ(result.failed_segment, 1U);
}

TEST(StaticSafetyGate, RejectsStaticCollisionAndUnknownSpace) {
  VoxelMap occupied_map = makeFreeMap();
  occupied_map.addBox(Eigen::Vector3d(-0.5, -1.0, 0.5), Eigen::Vector3d(0.5, 1.0, 1.5));
  const auto collision = makeGate().evaluate(occupied_map, makeCandidate(), 10.0);
  EXPECT_FALSE(collision.accepted);
  EXPECT_EQ(collision.status, SafetyGateStatus::STATIC_COLLISION);
  EXPECT_GT(collision.occupied_samples, 0U);

  VoxelMapConfig unknown_config;
  unknown_config.origin = Eigen::Vector3d(-8.0, -5.0, 0.0);
  unknown_config.dimensions = Eigen::Vector3i(32, 20, 12);
  unknown_config.resolution = 0.5;
  VoxelMap unknown_map(unknown_config);
  const auto unknown = makeGate().evaluate(unknown_map, makeCandidate(), 10.0);
  EXPECT_FALSE(unknown.accepted);
  EXPECT_EQ(unknown.status, SafetyGateStatus::UNKNOWN_SPACE);
  EXPECT_GT(unknown.unknown_samples, 0U);
}

TEST(StaticSafetyGate, RejectsCurrentStateDiscontinuityWithoutTakingFsmAction) {
  VoxelMap map = makeFreeMap();
  PlannedTrajectory current = makeCandidate(0.25);
  current.validated = true;

  const auto result = makeGate().evaluate(map, makeCandidate(), 10.0, current);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, SafetyGateStatus::CURRENT_STATE_DISCONTINUITY);
  EXPECT_TRUE(result.current_trajectory_available);
  EXPECT_TRUE(result.current_state_checked);
  EXPECT_FALSE(result.current_state_continuous);
  EXPECT_TRUE(result.current_trajectory_fallback_available);
}

TEST(StaticSafetyGate, RejectsDynamicsAfterStaticChecks) {
  VoxelMap map = makeFreeMap();
  StaticSafetyGateOptions options;
  options.validation.samples_per_span = 16;
  options.validation.max_velocity = 1.0;
  options.validation.max_acceleration = 6.0;
  const auto result = StaticSafetyGate(options).evaluate(map, makeCandidate(0.0), 10.0);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, SafetyGateStatus::VELOCITY_LIMIT);
  EXPECT_GT(result.maximum_velocity, options.validation.max_velocity);
}

TEST(StaticSafetyGate, RequiresCandidateToStartAtEvaluationTime) {
  VoxelMap map = makeFreeMap();
  const auto result = makeGate().evaluate(map, makeCandidate(), 10.1);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, SafetyGateStatus::TIME_DISCONTINUITY);
}

TEST(StaticSafetyGate, ReportsInvalidEvaluationTimeAsInputError) {
  VoxelMap map = makeFreeMap();

  const auto result = makeGate().evaluate(
      map, makeCandidate(), std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, SafetyGateStatus::INVALID_INPUT);
}
