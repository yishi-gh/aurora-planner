#include "aurora_planner_core/static_local_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;
using aurora::planner::GlobalReference;
using aurora::planner::LocalHorizonMode;
using aurora::planner::LocalPlannerOptions;
using aurora::planner::PlannerAction;
using aurora::planner::PlannerState;
using aurora::planner::PlanningRequest;
using aurora::planner::PlanningResult;
using aurora::planner::PlanningStatus;
using aurora::planner::ReplanFsmOptions;
using aurora::planner::ReplanObservation;
using aurora::planner::ReplanTrigger;
using aurora::planner::StrictMinimumSnapReferenceOptions;
using aurora::planner::StaticReplanFsm;
using aurora::planner::TrajectorySegment;
using aurora::planner::PlannedTrajectory;
using aurora::planner::StaticLocalPlanner;
using aurora::planner::VehicleState;
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

PlanningRequest makeRequest(double stamp = 10.0) {
  PlanningRequest request;
  request.request_id = static_cast<std::uint64_t>(stamp * 10.0);
  request.planning_stamp = stamp;
  request.vehicle_state.stamp = stamp;
  request.vehicle_state.position = Eigen::Vector3d(-4.0, 0.0, 1.0);
  request.vehicle_state.velocity.setZero();
  request.vehicle_state.acceleration.setZero();
  request.global_reference = GlobalReference::fromWaypoints({
      Eigen::Vector3d(-4.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(4.0, 0.0, 1.0),
  });
  return request;
}

LocalPlannerOptions makeOptions() {
  LocalPlannerOptions options;
  options.local_horizon = 3.0;
  options.resampling.spacing = 0.5;
  options.resampling.minimum_points = 9;
  options.optimizer.interval = 0.5;
  options.optimizer.clearance = 0.6;
  options.optimizer.max_velocity = 3.0;
  options.optimizer.max_acceleration = 6.0;
  options.optimizer.lambda_obstacle = 20.0;
  options.optimizer.max_iterations = 80;
  options.validation.samples_per_span = 16;
  options.validation.max_velocity = 3.0;
  options.validation.max_acceleration = 6.0;
  options.stitch_prefix_duration = 0.4;
  return options;
}

PlanningResult makeSuccessfulPlanningResult(double start_stamp = 10.0) {
  aurora::math::UniformBspline::ControlPointMatrix control_points(3, 7);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) = Eigen::Vector3d(-3.0 + static_cast<double>(index), 0.0, 1.0);
  }
  const UniformBspline spline(control_points, 0.5);
  PlannedTrajectory trajectory;
  trajectory.trajectory_id = 1U;
  trajectory.validated = true;
  trajectory.segments.push_back(
      TrajectorySegment{start_stamp, 0.0, spline.duration(), spline});

  PlanningResult result;
  result.status = PlanningStatus::SUCCESS;
  result.detail = "test trajectory accepted";
  result.trajectory = std::move(trajectory);
  return result;
}

ReplanObservation makeActiveObservation(double now = 10.0) {
  ReplanObservation observation;
  observation.now = now;
  observation.has_global_reference = true;
  observation.active_trajectory_available = true;
  observation.active_trajectory_safe = true;
  observation.active_trajectory_end_stamp = 20.0;
  return observation;
}

}  // namespace

TEST(StaticLocalPlanner, ExtractsDistanceAndTimeHorizonGoals) {
  PlanningRequest request = makeRequest();
  LocalPlannerOptions options = makeOptions();

  const auto distance_goal = StaticLocalPlanner::extractLocalGoal(request, options);
  EXPECT_NEAR(distance_goal.position.x(), -1.0, 1e-9);
  EXPECT_FALSE(distance_goal.terminal);
  EXPECT_NEAR(distance_goal.reference_progress, 3.0, 1e-9);

  request.global_reference.points.clear();
  for (int i = 0; i <= 8; ++i) {
    aurora::planner::ReferencePoint point;
    point.has_time = true;
    point.time_from_start = static_cast<double>(i);
    point.position = Eigen::Vector3d(-4.0 + static_cast<double>(i), 0.0, 1.0);
    point.velocity = Eigen::Vector3d::UnitX();
    request.global_reference.points.push_back(point);
  }
  options.horizon_mode = LocalHorizonMode::TIME;
  options.local_horizon = 2.5;
  const auto time_goal = StaticLocalPlanner::extractLocalGoal(request, options);
  EXPECT_NEAR(time_goal.position.x(), -1.5, 1e-9);
  EXPECT_NEAR(time_goal.reference_time, 2.5, 1e-9);
  EXPECT_FALSE(time_goal.terminal);
}

TEST(GlobalReference, ConvertsStrictMinimumSnapTrajectoryToTimedReference) {
  const auto trajectory = aurora::math::StrictMinimumSnapTrajectory::oneSegment(
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, 1.0, 2.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 2.0);

  const GlobalReference reference = GlobalReference::fromTrajectory(trajectory, 0.5);

  ASSERT_EQ(reference.points.size(), 5U);
  for (std::size_t index = 0; index < reference.points.size(); ++index) {
    EXPECT_TRUE(reference.points[index].has_time);
    EXPECT_NEAR(reference.points[index].time_from_start, 0.5 * static_cast<double>(index),
                1e-12);
  }
  EXPECT_TRUE(reference.points.front().position.isApprox(trajectory.evaluate(0.0, 0), 1e-10));
  EXPECT_TRUE(reference.points.front().velocity.isApprox(trajectory.evaluate(0.0, 1), 1e-10));
  EXPECT_TRUE(reference.points.back().position.isApprox(trajectory.evaluate(2.0, 0), 1e-10));
  EXPECT_TRUE(reference.points.back().acceleration.isApprox(trajectory.evaluate(2.0, 2), 1e-10));
}

TEST(GlobalReference, AllocatesTimesAndBuildsStrictThreeDimensionalReference) {
  const std::vector<Eigen::Vector3d> waypoints{
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 2.0, 1.5),
      Eigen::Vector3d(3.0, 2.0, 2.0)};
  StrictMinimumSnapReferenceOptions options;
  options.sample_interval = 0.4;
  options.time_allocation.max_velocity = 3.0;
  options.time_allocation.max_acceleration = 4.0;
  options.time_allocation.max_jerk = 8.0;
  options.time_allocation.minimum_segment_time = 0.2;
  options.time_allocation.time_scale = 1.25;
  options.start_velocity = Eigen::Vector3d(0.1, 0.0, 0.0);
  options.end_acceleration = Eigen::Vector3d(0.0, 0.0, 0.2);

  const GlobalReference reference =
      GlobalReference::fromWaypointsWithTimeAllocation(waypoints, options);

  ASSERT_GE(reference.points.size(), 2U);
  EXPECT_TRUE(reference.points.front().has_time);
  EXPECT_TRUE(reference.points.back().has_time);
  EXPECT_DOUBLE_EQ(reference.points.front().time_from_start, 0.0);
  EXPECT_GT(reference.points.back().time_from_start, 0.0);
  for (std::size_t index = 1; index < reference.points.size(); ++index) {
    EXPECT_TRUE(reference.points[index].has_time);
    EXPECT_GT(reference.points[index].time_from_start,
              reference.points[index - 1U].time_from_start);
  }
  EXPECT_TRUE(reference.points.front().position.isApprox(waypoints.front(), 1e-9));
  EXPECT_TRUE(reference.points.back().position.isApprox(waypoints.back(), 1e-9));
  EXPECT_TRUE(reference.points.front().velocity.isApprox(options.start_velocity, 1e-8));
  EXPECT_TRUE(reference.points.back().acceleration.isApprox(options.end_acceleration, 1e-8));
}

TEST(StaticLocalPlanner, PlansAndValidatesAStaticLocalTrajectory) {
  VoxelMap map = makeFreeMap();
  PlanningRequest request = makeRequest();
  LocalPlannerOptions options = makeOptions();
  options.stitch_prefix_duration = 0.0;

  const StaticLocalPlanner planner(options);
  const auto result = planner.plan(map, request);

  ASSERT_EQ(result.status, PlanningStatus::SUCCESS) << result.detail;
  ASSERT_TRUE(result.trajectory.has_value());
  ASSERT_EQ(result.trajectory->segments.size(), 1U);
  EXPECT_TRUE(result.trajectory->validated);
  EXPECT_EQ(result.trajectory->map_version, map.version());
  EXPECT_TRUE(result.trajectory->contains(request.planning_stamp));
  EXPECT_GT(result.trajectory->endStamp(), request.planning_stamp);
  const auto start = result.trajectory->evaluate(request.planning_stamp);
  EXPECT_NEAR(start.position.x(), request.vehicle_state.position.x(), 1e-8);
  EXPECT_NEAR(start.position.y(), request.vehicle_state.position.y(), 1e-8);
  EXPECT_NEAR(result.final_cost, result.optimization.cost.total, 1e-12);
  EXPECT_LT(result.final_cost, std::numeric_limits<double>::infinity());
}

TEST(StaticLocalPlanner, PassesAbsoluteRiskTimeToTheOptimizer) {
  VoxelMap map = makeFreeMap();
  PlanningRequest request = makeRequest();
  LocalPlannerOptions options = makeOptions();
  options.stitch_prefix_duration = 0.0;
  options.optimizer.lambda_smooth = 0.0;
  options.optimizer.lambda_obstacle = 0.0;
  options.optimizer.lambda_feasibility = 0.0;
  options.optimizer.lambda_fitness = 0.0;
  options.optimizer.lambda_risk = 1.0;
  options.optimizer.max_iterations = 30;

  std::vector<double> observed_stamps;
  const aurora::trajectory::RiskCostFunction risk_cost =
      [&observed_stamps](double stamp, const Eigen::Vector3d &position) {
        (void)position;
        observed_stamps.push_back(stamp);
        aurora::trajectory::RiskCostEvaluation result;
        result.value = 0.1;
        result.gradient.setZero();
        return result;
      };

  const StaticLocalPlanner planner(options);
  const auto result = planner.plan(map, request, std::nullopt, risk_cost);

  ASSERT_EQ(result.status, PlanningStatus::SUCCESS) << result.detail;
  ASSERT_TRUE(result.trajectory.has_value());
  ASSERT_FALSE(observed_stamps.empty());
  EXPECT_DOUBLE_EQ(observed_stamps.front(), request.planning_stamp);
  EXPECT_TRUE(result.optimization.risk_enabled);
  EXPECT_FALSE(result.optimization.risk_fallback);
  EXPECT_DOUBLE_EQ(result.final_cost, result.initial_cost);
}

TEST(StaticLocalPlanner, RetainsValidatedPrefixAndKeepsBoundaryStateContinuous) {
  VoxelMap map = makeFreeMap();
  PlanningRequest first_request = makeRequest();
  LocalPlannerOptions options = makeOptions();
  const StaticLocalPlanner planner(options);

  const auto first = planner.plan(map, first_request);
  ASSERT_EQ(first.status, PlanningStatus::SUCCESS) << first.detail;
  ASSERT_TRUE(first.trajectory.has_value());

  PlanningRequest second_request = makeRequest(10.2);
  const auto previous_state = first.trajectory->evaluate(second_request.planning_stamp);
  second_request.vehicle_state.stamp = second_request.planning_stamp;
  second_request.vehicle_state.position = previous_state.position;
  second_request.vehicle_state.velocity = previous_state.velocity;
  second_request.vehicle_state.acceleration = previous_state.acceleration;

  const auto second = planner.plan(map, second_request, first.trajectory);
  ASSERT_EQ(second.status, PlanningStatus::SUCCESS) << second.detail;
  ASSERT_TRUE(second.trajectory.has_value());
  ASSERT_EQ(second.trajectory->segments.size(), 2U);
  EXPECT_TRUE(second.used_stitch_prefix);
  EXPECT_NEAR(second.stitch_prefix_duration, options.stitch_prefix_duration, 1e-9);

  const double join = second.trajectory->segments.back().start_stamp;
  const auto before_join = second.trajectory->segments.front().evaluate(join);
  const auto after_join = second.trajectory->segments.back().evaluate(join);
  EXPECT_NEAR((before_join.position - after_join.position).norm(), 0.0, 1e-8);
  EXPECT_NEAR((before_join.velocity - after_join.velocity).norm(), 0.0, 1e-8);
  EXPECT_NEAR((before_join.acceleration - after_join.acceleration).norm(), 0.0, 1e-7);
}

TEST(StaticLocalPlanner, DoesNotPublishWhenStaticSearchHasNoPath) {
  VoxelMap map = makeFreeMap();
  map.addBox(Eigen::Vector3d(-0.5, -5.0, 0.0), Eigen::Vector3d(0.5, 5.0, 6.0));

  PlanningRequest request = makeRequest();
  LocalPlannerOptions options = makeOptions();
  options.local_horizon = 20.0;
  options.stitch_prefix_duration = 0.0;
  const StaticLocalPlanner planner(options);
  const auto result = planner.plan(map, request);

  EXPECT_EQ(result.status, PlanningStatus::SEARCH_FAILED);
  EXPECT_FALSE(result.trajectory.has_value());
  EXPECT_EQ(result.search.status, aurora::search::SearchStatus::NO_PATH);
}

TEST(StaticReplanFsm, TransitionsAndUsesTheConfirmedTriggerPriority) {
  StaticReplanFsm fsm;
  ReplanObservation observation;
  observation.now = 10.0;

  auto decision = fsm.step(observation);
  EXPECT_EQ(decision.state, PlannerState::WAIT_TARGET);
  EXPECT_EQ(decision.action, PlannerAction::WAIT);

  observation.has_global_reference = true;
  decision = fsm.step(observation);
  EXPECT_EQ(decision.state, PlannerState::GENERATE);
  EXPECT_EQ(decision.action, PlannerAction::START_PLANNING);
  decision = fsm.step(observation);
  EXPECT_EQ(decision.action, PlannerAction::WAIT_FOR_RESULT);

  decision = fsm.onPlanningResult(makeSuccessfulPlanningResult(), observation.now);
  EXPECT_EQ(decision.state, PlannerState::EXECUTE);
  EXPECT_EQ(decision.action, PlannerAction::ACCEPT_NEW_TRAJECTORY);

  auto triggerFor = [](bool collision, bool stale, bool failed, bool dynamic_updated,
                       bool map_updated, bool near_end, bool goal_expired) {
    StaticReplanFsm local_fsm;
    ReplanObservation local_observation = makeActiveObservation();
    local_observation.current_trajectory_collision = collision;
    local_observation.safety_information_stale = stale;
    local_observation.planning_failed = failed;
    local_observation.dynamic_obstacle_updated = dynamic_updated;
    local_observation.map_updated = map_updated;
    local_observation.local_goal_expired = goal_expired;
    if (near_end) {
      local_observation.active_trajectory_end_stamp = 10.5;
    }
    const auto initial_decision = local_fsm.step(local_observation);
    EXPECT_EQ(initial_decision.action, PlannerAction::START_PLANNING);
    EXPECT_EQ(local_fsm.onPlanningResult(makeSuccessfulPlanningResult(), 10.0).state,
              PlannerState::EXECUTE);
    const auto replan_decision = local_fsm.step(local_observation);
    EXPECT_EQ(replan_decision.action, PlannerAction::REQUEST_REPLAN);
    const auto start_decision = local_fsm.step(local_observation);
    EXPECT_EQ(start_decision.action, PlannerAction::START_PLANNING);
    EXPECT_EQ(start_decision.trigger, replan_decision.trigger);
    return replan_decision;
  };

  decision = triggerFor(true, true, true, true, true, true, true);
  EXPECT_EQ(decision.state, PlannerState::REPLAN);
  EXPECT_EQ(decision.action, PlannerAction::REQUEST_REPLAN);
  EXPECT_EQ(decision.trigger, ReplanTrigger::CURRENT_TRAJECTORY_COLLISION);

  decision = triggerFor(false, true, true, true, true, true, true);
  EXPECT_EQ(decision.trigger, ReplanTrigger::SAFETY_INFORMATION_STALE);

  decision = triggerFor(false, false, true, true, true, true, true);
  EXPECT_EQ(decision.trigger, ReplanTrigger::PLANNING_FAILURE);

  decision = triggerFor(false, false, false, true, true, true, true);
  EXPECT_EQ(decision.trigger, ReplanTrigger::DYNAMIC_OBSTACLE_UPDATED);

  decision = triggerFor(false, false, false, false, true, true, true);
  EXPECT_EQ(decision.trigger, ReplanTrigger::MAP_UPDATED);

  decision = triggerFor(false, false, false, false, false, true, true);
  EXPECT_EQ(decision.trigger, ReplanTrigger::TRAJECTORY_NEAR_END);

  decision = triggerFor(false, false, false, false, false, false, true);
  EXPECT_EQ(decision.trigger, ReplanTrigger::LOCAL_GOAL_EXPIRED);
}

TEST(StaticReplanFsm, KeepsFallbackThenEntersDegradedAndStopsWhenSafeTimeRunsOut) {
  ReplanFsmOptions options;
  options.max_consecutive_failures = 3U;
  options.emergency_time_remaining = 0.5;
  StaticReplanFsm fsm(options);
  ReplanObservation observation = makeActiveObservation();

  fsm.step(observation);
  ASSERT_EQ(fsm.state(), PlannerState::GENERATE);
  ASSERT_EQ(fsm.onPlanningResult(makeSuccessfulPlanningResult(), observation.now).state,
            PlannerState::EXECUTE);

  PlanningResult failed;
  failed.status = PlanningStatus::SEARCH_FAILED;
  failed.detail = "no static path";
  for (std::size_t attempt = 1U; attempt < options.max_consecutive_failures; ++attempt) {
    observation.map_updated = true;
    ASSERT_EQ(fsm.step(observation).trigger, ReplanTrigger::MAP_UPDATED);
    ASSERT_EQ(fsm.step(observation).action, PlannerAction::START_PLANNING);
    const auto decision = fsm.onPlanningResult(failed, observation.now);
    EXPECT_EQ(decision.state, PlannerState::EXECUTE);
    EXPECT_EQ(decision.action, PlannerAction::KEEP_CURRENT_TRAJECTORY);
    EXPECT_EQ(decision.consecutive_failures, attempt);
  }

  observation.map_updated = true;
  ASSERT_EQ(fsm.step(observation).trigger, ReplanTrigger::MAP_UPDATED);
  ASSERT_EQ(fsm.step(observation).action, PlannerAction::START_PLANNING);
  auto decision = fsm.onPlanningResult(failed, observation.now);
  EXPECT_EQ(decision.state, PlannerState::DEGRADED);
  EXPECT_EQ(decision.action, PlannerAction::HOLD_POSITION);
  EXPECT_EQ(decision.consecutive_failures, options.max_consecutive_failures);

  observation.map_updated = false;
  observation.now = 15.0;
  decision = fsm.step(observation);
  EXPECT_EQ(decision.state, PlannerState::DEGRADED);
  EXPECT_EQ(decision.action, PlannerAction::HOLD_POSITION);

  observation.now = 19.6;
  decision = fsm.step(observation);
  EXPECT_EQ(decision.state, PlannerState::EMERGENCY_STOP);
  EXPECT_EQ(decision.action, PlannerAction::EMERGENCY_STOP);
}

TEST(StaticReplanFsm, StopsImmediatelyWhenPlanningFailsWithoutSafeFallback) {
  StaticReplanFsm fsm;
  ReplanObservation observation;
  observation.now = 10.0;
  observation.has_global_reference = true;
  ASSERT_EQ(fsm.step(observation).action, PlannerAction::START_PLANNING);

  PlanningResult failed;
  failed.status = PlanningStatus::VALIDATION_FAILED;
  failed.detail = "collision";
  const auto decision = fsm.onPlanningResult(failed, observation.now);
  EXPECT_EQ(decision.state, PlannerState::EMERGENCY_STOP);
  EXPECT_EQ(decision.action, PlannerAction::EMERGENCY_STOP);
  EXPECT_EQ(decision.trigger, ReplanTrigger::PLANNING_FAILURE);

  const auto latched = fsm.step(observation);
  EXPECT_EQ(latched.state, PlannerState::EMERGENCY_STOP);
  EXPECT_EQ(latched.action, PlannerAction::EMERGENCY_STOP);
}
