#include "aurora_planner_core/static_local_planner.hpp"
#include "aurora_planner_core/static_safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::planner {
namespace {

constexpr double kEpsilon = 1e-9;

bool finiteVector(const Eigen::Vector3d &value) { return value.allFinite(); }

bool finiteOrPositiveInfinity(double value) {
  return std::isfinite(value) || value == std::numeric_limits<double>::infinity();
}

void validateReferencePoint(const ReferencePoint &point) {
  if (point.has_time && (!std::isfinite(point.time_from_start) || point.time_from_start < 0.0)) {
    throw std::invalid_argument("reference point time must be finite and non-negative");
  }
  if (!finiteVector(point.position) || !finiteVector(point.velocity) ||
      !finiteVector(point.acceleration)) {
    throw std::invalid_argument("reference point state must be finite");
  }
}

}  // namespace

GlobalReference GlobalReference::fromWaypoints(
    const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration) {
  if (waypoints.size() < 2U || !start_velocity.allFinite() || !end_velocity.allFinite() ||
      !start_acceleration.allFinite() || !end_acceleration.allFinite()) {
    throw std::invalid_argument("global waypoint reference is invalid");
  }

  GlobalReference reference;
  reference.points.reserve(waypoints.size());
  for (const Eigen::Vector3d &waypoint : waypoints) {
    if (!waypoint.allFinite()) {
      throw std::invalid_argument("global waypoint reference contains non-finite values");
    }
    ReferencePoint point;
    point.position = waypoint;
    reference.points.push_back(point);
  }
  reference.points.front().velocity = start_velocity;
  reference.points.front().acceleration = start_acceleration;
  reference.points.back().velocity = end_velocity;
  reference.points.back().acceleration = end_acceleration;
  return reference;
}

GlobalReference GlobalReference::fromTrajectory(
    const aurora::math::MinimumSnapTrajectory &trajectory, double sample_interval) {
  if (!std::isfinite(sample_interval) || sample_interval <= 0.0) {
    throw std::invalid_argument("reference trajectory sample interval must be positive");
  }
  const std::size_t sample_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(trajectory.duration() / sample_interval)));

  GlobalReference reference;
  reference.points.reserve(sample_count + 1U);
  for (std::size_t sample = 0; sample <= sample_count; ++sample) {
    const double time = trajectory.duration() * static_cast<double>(sample) /
                        static_cast<double>(sample_count);
    ReferencePoint point;
    point.time_from_start = time;
    point.has_time = true;
    point.position = trajectory.evaluate(time, 0);
    point.velocity = trajectory.evaluate(time, 1);
    point.acceleration = trajectory.evaluate(time, 2);
    reference.points.push_back(point);
  }
  return reference;
}

GlobalReference GlobalReference::fromTrajectory(
    const aurora::math::StrictMinimumSnapTrajectory &trajectory, double sample_interval) {
  if (!std::isfinite(sample_interval) || sample_interval <= 0.0) {
    throw std::invalid_argument("reference trajectory sample interval must be positive");
  }
  const std::size_t sample_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(trajectory.duration() / sample_interval)));

  GlobalReference reference;
  reference.points.reserve(sample_count + 1U);
  for (std::size_t sample = 0; sample <= sample_count; ++sample) {
    const double time = trajectory.duration() * static_cast<double>(sample) /
                        static_cast<double>(sample_count);
    ReferencePoint point;
    point.time_from_start = time;
    point.has_time = true;
    point.position = trajectory.evaluate(time, 0);
    point.velocity = trajectory.evaluate(time, 1);
    point.acceleration = trajectory.evaluate(time, 2);
    reference.points.push_back(point);
  }
  return reference;
}

GlobalReference GlobalReference::fromWaypointsWithTimeAllocation(
    const std::vector<Eigen::Vector3d> &waypoints,
    const StrictMinimumSnapReferenceOptions &options) {
  if (waypoints.size() < 2U) {
    throw std::invalid_argument("strict minimum snap reference requires at least two waypoints");
  }

  aurora::math::StrictMinimumSnapTrajectory::WaypointMatrix waypoint_matrix(
      3, static_cast<Eigen::Index>(waypoints.size()));
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    if (!waypoints[index].allFinite()) {
      throw std::invalid_argument("strict minimum snap reference waypoints must be finite");
    }
    waypoint_matrix.col(static_cast<Eigen::Index>(index)) = waypoints[index];
  }

  const Eigen::VectorXd segment_times =
      aurora::math::allocateSegmentTimes(waypoint_matrix, options.time_allocation);
  const auto trajectory = aurora::math::StrictMinimumSnapTrajectory::fromWaypoints(
      waypoint_matrix, options.start_velocity, options.end_velocity,
      options.start_acceleration, options.end_acceleration, options.start_jerk,
      options.end_jerk, segment_times);
  return fromTrajectory(trajectory, options.sample_interval);
}

TrajectoryState TrajectorySegment::evaluate(double stamp) const {
  if (!std::isfinite(stamp) || !std::isfinite(start_stamp) || !std::isfinite(source_start_time) ||
      !std::isfinite(duration) || duration <= 0.0) {
    throw std::invalid_argument("invalid trajectory segment evaluation arguments");
  }
  const double relative_time = std::clamp(stamp - start_stamp, 0.0, duration);
  const double spline_time = std::clamp(source_start_time + relative_time, 0.0, spline.duration());
  TrajectoryState state;
  state.stamp = start_stamp + relative_time;
  state.position = spline.evaluate(spline_time, 0);
  state.velocity = spline.evaluate(spline_time, 1);
  state.acceleration = spline.evaluate(spline_time, 2);
  return state;
}

double PlannedTrajectory::startStamp() const noexcept {
  return empty() ? std::numeric_limits<double>::quiet_NaN() : segments.front().start_stamp;
}

double PlannedTrajectory::endStamp() const noexcept {
  return empty() ? std::numeric_limits<double>::quiet_NaN() : segments.back().endStamp();
}

bool PlannedTrajectory::contains(double stamp) const noexcept {
  return !empty() && std::isfinite(stamp) && stamp >= startStamp() - kEpsilon &&
         stamp <= endStamp() + kEpsilon;
}

TrajectoryState PlannedTrajectory::evaluate(double stamp) const {
  if (empty() || !std::isfinite(stamp)) {
    throw std::invalid_argument("cannot evaluate an empty or invalid planned trajectory");
  }
  const TrajectorySegment *selected = &segments.front();
  for (const TrajectorySegment &segment : segments) {
    if (stamp + kEpsilon >= segment.start_stamp) {
      selected = &segment;
    } else {
      break;
    }
  }
  return selected->evaluate(stamp);
}

std::vector<TrajectorySegment> PlannedTrajectory::slice(double start_stamp,
                                                         double end_stamp) const {
  if (empty() || !std::isfinite(start_stamp) || !std::isfinite(end_stamp) ||
      end_stamp <= start_stamp || start_stamp < this->startStamp() - kEpsilon ||
      end_stamp > this->endStamp() + kEpsilon) {
    throw std::invalid_argument("trajectory slice is outside the trajectory lifetime");
  }

  std::vector<TrajectorySegment> result;
  for (const TrajectorySegment &segment : segments) {
    const double overlap_start = std::max(start_stamp, segment.start_stamp);
    const double overlap_end = std::min(end_stamp, segment.endStamp());
    if (overlap_end <= overlap_start + kEpsilon) {
      continue;
    }
    TrajectorySegment piece = segment;
    piece.start_stamp = overlap_start;
    piece.source_start_time += overlap_start - segment.start_stamp;
    piece.duration = overlap_end - overlap_start;
    result.push_back(std::move(piece));
  }
  if (result.empty()) {
    throw std::logic_error("trajectory slice produced no segments");
  }
  return result;
}

const char *toString(PlanningStatus status) noexcept {
  switch (status) {
    case PlanningStatus::SUCCESS:
      return "SUCCESS";
    case PlanningStatus::GOAL_REACHED:
      return "GOAL_REACHED";
    case PlanningStatus::INVALID_REQUEST:
      return "INVALID_REQUEST";
    case PlanningStatus::NO_GLOBAL_REFERENCE:
      return "NO_GLOBAL_REFERENCE";
    case PlanningStatus::INVALID_HORIZON:
      return "INVALID_HORIZON";
    case PlanningStatus::LOCAL_GOAL_UNAVAILABLE:
      return "LOCAL_GOAL_UNAVAILABLE";
    case PlanningStatus::SEARCH_FAILED:
      return "SEARCH_FAILED";
    case PlanningStatus::OPTIMIZATION_FAILED:
      return "OPTIMIZATION_FAILED";
    case PlanningStatus::PREFIX_UNSAFE:
      return "PREFIX_UNSAFE";
    case PlanningStatus::VALIDATION_FAILED:
      return "VALIDATION_FAILED";
  }
  return "UNKNOWN_STATUS";
}

const char *toString(PlannerState state) noexcept {
  switch (state) {
    case PlannerState::INIT:
      return "INIT";
    case PlannerState::WAIT_TARGET:
      return "WAIT_TARGET";
    case PlannerState::GENERATE:
      return "GENERATE";
    case PlannerState::EXECUTE:
      return "EXECUTE";
    case PlannerState::REPLAN:
      return "REPLAN";
    case PlannerState::DEGRADED:
      return "DEGRADED";
    case PlannerState::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
  }
  return "UNKNOWN_STATE";
}

const char *toString(ReplanTrigger trigger) noexcept {
  switch (trigger) {
    case ReplanTrigger::NONE:
      return "NONE";
    case ReplanTrigger::CURRENT_TRAJECTORY_COLLISION:
      return "CURRENT_TRAJECTORY_COLLISION";
    case ReplanTrigger::SAFETY_INFORMATION_STALE:
      return "SAFETY_INFORMATION_STALE";
    case ReplanTrigger::PLANNING_FAILURE:
      return "PLANNING_FAILURE";
    case ReplanTrigger::MAP_UPDATED:
      return "MAP_UPDATED";
    case ReplanTrigger::TRAJECTORY_NEAR_END:
      return "TRAJECTORY_NEAR_END";
    case ReplanTrigger::LOCAL_GOAL_EXPIRED:
      return "LOCAL_GOAL_EXPIRED";
    case ReplanTrigger::DYNAMIC_OBSTACLE_UPDATED:
      return "DYNAMIC_OBSTACLE_UPDATED";
  }
  return "UNKNOWN_TRIGGER";
}

const char *toString(PlannerAction action) noexcept {
  switch (action) {
    case PlannerAction::WAIT:
      return "WAIT";
    case PlannerAction::REQUEST_REPLAN:
      return "REQUEST_REPLAN";
    case PlannerAction::START_PLANNING:
      return "START_PLANNING";
    case PlannerAction::WAIT_FOR_RESULT:
      return "WAIT_FOR_RESULT";
    case PlannerAction::ACCEPT_NEW_TRAJECTORY:
      return "ACCEPT_NEW_TRAJECTORY";
    case PlannerAction::KEEP_CURRENT_TRAJECTORY:
      return "KEEP_CURRENT_TRAJECTORY";
    case PlannerAction::HOLD_POSITION:
      return "HOLD_POSITION";
    case PlannerAction::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case PlannerAction::INVALID_INPUT:
      return "INVALID_INPUT";
  }
  return "UNKNOWN_ACTION";
}

StaticReplanFsm::StaticReplanFsm(ReplanFsmOptions options) : options_(std::move(options)) {
  validateOptions(options_);
}

void StaticReplanFsm::validateOptions(const ReplanFsmOptions &options) {
  if (options.max_consecutive_failures == 0U ||
      !std::isfinite(options.trajectory_near_end_margin) ||
      options.trajectory_near_end_margin <= 0.0 ||
      !std::isfinite(options.emergency_time_remaining) ||
      options.emergency_time_remaining < 0.0) {
    throw std::invalid_argument("invalid replanning FSM options");
  }
}

bool StaticReplanFsm::validateObservation(const ReplanObservation &observation,
                                          std::string *detail) {
  if (!std::isfinite(observation.now)) {
    if (detail != nullptr) {
      *detail = "FSM observation time is not finite";
    }
    return false;
  }
  if (observation.active_trajectory_safe && !observation.active_trajectory_available) {
    if (detail != nullptr) {
      *detail = "an unavailable trajectory cannot be marked safe";
    }
    return false;
  }
  if (observation.active_trajectory_available &&
      !std::isfinite(observation.active_trajectory_end_stamp)) {
    if (detail != nullptr) {
      *detail = "active trajectory end time is required when a trajectory is available";
    }
    return false;
  }
  return true;
}

ReplanTrigger StaticReplanFsm::selectTrigger(const ReplanObservation &observation,
                                             const ReplanFsmOptions &options) {
  if (observation.current_trajectory_collision) {
    return ReplanTrigger::CURRENT_TRAJECTORY_COLLISION;
  }
  if (observation.safety_information_stale) {
    return ReplanTrigger::SAFETY_INFORMATION_STALE;
  }
  if (observation.planning_failed) {
    return ReplanTrigger::PLANNING_FAILURE;
  }
  if (observation.dynamic_obstacle_updated) {
    return ReplanTrigger::DYNAMIC_OBSTACLE_UPDATED;
  }
  if (observation.map_updated) {
    return ReplanTrigger::MAP_UPDATED;
  }
  if (!observation.active_trajectory_available ||
      observation.active_trajectory_end_stamp - observation.now <=
          options.trajectory_near_end_margin) {
    return ReplanTrigger::TRAJECTORY_NEAR_END;
  }
  if (observation.local_goal_expired) {
    return ReplanTrigger::LOCAL_GOAL_EXPIRED;
  }
  return ReplanTrigger::NONE;
}

double StaticReplanFsm::remainingTrajectoryTime(const ReplanObservation &observation) {
  if (!observation.active_trajectory_available ||
      !std::isfinite(observation.active_trajectory_end_stamp)) {
    return 0.0;
  }
  return std::max(0.0, observation.active_trajectory_end_stamp - observation.now);
}

FsmDecision StaticReplanFsm::makeDecision(PlannerState previous_state, PlannerAction action,
                                          ReplanTrigger trigger, std::string detail) const {
  FsmDecision decision;
  decision.previous_state = previous_state;
  decision.state = state_;
  decision.trigger = trigger;
  decision.action = action;
  decision.consecutive_failures = consecutive_failures_;
  decision.detail = std::move(detail);
  return decision;
}

void StaticReplanFsm::reset() noexcept {
  state_ = PlannerState::INIT;
  consecutive_failures_ = 0U;
  last_observation_.reset();
}

FsmDecision StaticReplanFsm::step(const ReplanObservation &observation) {
  std::string invalid_detail;
  const PlannerState previous_state = state_;
  if (!validateObservation(observation, &invalid_detail)) {
    FsmDecision decision = makeDecision(previous_state, PlannerAction::INVALID_INPUT,
                                        ReplanTrigger::NONE, std::move(invalid_detail));
    decision.valid = false;
    return decision;
  }
  last_observation_ = observation;

  if (state_ == PlannerState::EMERGENCY_STOP) {
    return makeDecision(previous_state, PlannerAction::EMERGENCY_STOP, ReplanTrigger::NONE,
                        "emergency stop is latched until reset");
  }

  switch (state_) {
    case PlannerState::INIT:
      if (observation.has_global_reference) {
        state_ = PlannerState::GENERATE;
        return makeDecision(previous_state, PlannerAction::START_PLANNING, ReplanTrigger::NONE,
                            "global reference available; start initial planning");
      }
      state_ = PlannerState::WAIT_TARGET;
      return makeDecision(previous_state, PlannerAction::WAIT, ReplanTrigger::NONE,
                          "waiting for a global reference");

    case PlannerState::WAIT_TARGET:
      if (observation.has_global_reference) {
        state_ = PlannerState::GENERATE;
        return makeDecision(previous_state, PlannerAction::START_PLANNING, ReplanTrigger::NONE,
                            "global reference received; start planning");
      }
      if (observation.active_trajectory_available && observation.active_trajectory_safe) {
        return makeDecision(previous_state, PlannerAction::KEEP_CURRENT_TRAJECTORY,
                            ReplanTrigger::NONE,
                            "no new target; keep the currently validated trajectory");
      }
      return makeDecision(previous_state, PlannerAction::WAIT, ReplanTrigger::NONE,
                          "waiting for a global reference");

    case PlannerState::GENERATE:
      if (!observation.has_global_reference) {
        state_ = PlannerState::WAIT_TARGET;
        return makeDecision(previous_state, PlannerAction::WAIT, ReplanTrigger::NONE,
                            "global reference disappeared while planning");
      }
      return makeDecision(previous_state, PlannerAction::WAIT_FOR_RESULT, ReplanTrigger::NONE,
                          "planning request is in flight");

    case PlannerState::EXECUTE: {
      if (!observation.has_global_reference) {
        if (observation.active_trajectory_available && observation.active_trajectory_safe) {
          state_ = PlannerState::WAIT_TARGET;
          return makeDecision(previous_state, PlannerAction::KEEP_CURRENT_TRAJECTORY,
                              ReplanTrigger::NONE,
                              "target is unavailable; keep the validated trajectory");
        }
        state_ = PlannerState::EMERGENCY_STOP;
        return makeDecision(previous_state, PlannerAction::EMERGENCY_STOP, ReplanTrigger::NONE,
                            "target and safe active trajectory are both unavailable");
      }
      const ReplanTrigger trigger = selectTrigger(observation, options_);
      if (trigger == ReplanTrigger::NONE) {
        return makeDecision(previous_state, PlannerAction::KEEP_CURRENT_TRAJECTORY,
                            ReplanTrigger::NONE, "active trajectory remains usable");
      }
      state_ = PlannerState::REPLAN;
      return makeDecision(previous_state, PlannerAction::REQUEST_REPLAN, trigger,
                          std::string("replanning requested by ") + toString(trigger));
    }

    case PlannerState::REPLAN: {
      if (!observation.has_global_reference) {
        state_ = PlannerState::WAIT_TARGET;
        return makeDecision(previous_state, PlannerAction::WAIT, ReplanTrigger::NONE,
                            "replanning cancelled because the target disappeared");
      }
      // Preserve the reason that caused the two-step REPLAN transition in
      // the START_PLANNING decision so transport diagnostics can explain the
      // actual planning attempt.
      const ReplanTrigger trigger = selectTrigger(observation, options_);
      state_ = PlannerState::GENERATE;
      return makeDecision(previous_state, PlannerAction::START_PLANNING, trigger,
                          "start the requested replanning attempt");
    }

    case PlannerState::DEGRADED:
      if (!observation.active_trajectory_available || !observation.active_trajectory_safe ||
          observation.current_trajectory_collision ||
          remainingTrajectoryTime(observation) <= options_.emergency_time_remaining) {
        state_ = PlannerState::EMERGENCY_STOP;
        return makeDecision(previous_state, PlannerAction::EMERGENCY_STOP, ReplanTrigger::NONE,
                            "degraded fallback no longer has sufficient safe time");
      }
      return makeDecision(previous_state, PlannerAction::HOLD_POSITION, ReplanTrigger::NONE,
                          "degraded mode: hold position while the safe fallback remains valid");

    case PlannerState::EMERGENCY_STOP:
      break;
  }

  state_ = PlannerState::EMERGENCY_STOP;
  return makeDecision(previous_state, PlannerAction::EMERGENCY_STOP, ReplanTrigger::NONE,
                      "unhandled FSM state; fail safe");
}

FsmDecision StaticReplanFsm::failedPlanningDecision(const PlanningResult &result, double now) {
  const PlannerState previous_state = state_;
  ++consecutive_failures_;

  ReplanObservation observation;
  if (last_observation_.has_value()) {
    observation = *last_observation_;
  }
  observation.now = now;
  const bool safe_fallback = observation.active_trajectory_available &&
                             observation.active_trajectory_safe &&
                             !observation.current_trajectory_collision &&
                             remainingTrajectoryTime(observation) > options_.emergency_time_remaining;
  const std::string failure = std::string("planning failed with ") +
                              toString(result.status) + ": " + result.detail;

  if (!safe_fallback) {
    state_ = PlannerState::EMERGENCY_STOP;
    return makeDecision(previous_state, PlannerAction::EMERGENCY_STOP,
                        ReplanTrigger::PLANNING_FAILURE,
                        failure + "; no safe fallback remains");
  }
  if (consecutive_failures_ >= options_.max_consecutive_failures) {
    state_ = PlannerState::DEGRADED;
    return makeDecision(previous_state, PlannerAction::HOLD_POSITION,
                        ReplanTrigger::PLANNING_FAILURE,
                        failure + "; failure limit reached, entering degraded mode");
  }
  state_ = PlannerState::EXECUTE;
  return makeDecision(previous_state, PlannerAction::KEEP_CURRENT_TRAJECTORY,
                      ReplanTrigger::PLANNING_FAILURE,
                      failure + "; keeping the validated fallback trajectory");
}

FsmDecision StaticReplanFsm::onPlanningResult(const PlanningResult &result, double now) {
  const PlannerState previous_state = state_;
  if (!std::isfinite(now)) {
    FsmDecision decision = makeDecision(previous_state, PlannerAction::INVALID_INPUT,
                                        ReplanTrigger::NONE, "planning result time is not finite");
    decision.valid = false;
    return decision;
  }
  if (state_ == PlannerState::EMERGENCY_STOP) {
    return makeDecision(previous_state, PlannerAction::EMERGENCY_STOP, ReplanTrigger::NONE,
                        "planning result ignored because emergency stop is latched");
  }

  if (result.status == PlanningStatus::GOAL_REACHED) {
    consecutive_failures_ = 0U;
    state_ = PlannerState::WAIT_TARGET;
    return makeDecision(previous_state, PlannerAction::HOLD_POSITION, ReplanTrigger::NONE,
                        "terminal goal reached; hold until a new target arrives");
  }
  if (result.status == PlanningStatus::SUCCESS && result.trajectory.has_value() &&
      result.trajectory->validated && !result.trajectory->empty()) {
    consecutive_failures_ = 0U;
    state_ = PlannerState::EXECUTE;
    return makeDecision(previous_state, PlannerAction::ACCEPT_NEW_TRAJECTORY, ReplanTrigger::NONE,
                        "validated trajectory accepted for execution");
  }
  return failedPlanningDecision(result, now);
}

StaticLocalPlanner::StaticLocalPlanner(LocalPlannerOptions options) : options_(std::move(options)) {
  validateOptions(options_);
}

void StaticLocalPlanner::validateOptions(const LocalPlannerOptions &options) {
  if (!std::isfinite(options.local_horizon) || options.local_horizon <= 0.0 ||
      !std::isfinite(options.goal_tolerance) || options.goal_tolerance < 0.0 ||
      !finiteOrPositiveInfinity(options.max_projection_distance) ||
      options.max_projection_distance < 0.0 ||
      !std::isfinite(options.stitch_prefix_duration) || options.stitch_prefix_duration < 0.0 ||
      !std::isfinite(options.stitch_position_tolerance) ||
      options.stitch_position_tolerance < 0.0 ||
      !std::isfinite(options.stitch_velocity_tolerance) ||
      options.stitch_velocity_tolerance < 0.0 ||
      !std::isfinite(options.stitch_acceleration_tolerance) ||
      options.stitch_acceleration_tolerance < 0.0) {
    throw std::invalid_argument("invalid local planner options");
  }
  if (options.resampling.spacing <= 0.0 || !std::isfinite(options.resampling.spacing) ||
      options.resampling.minimum_points < 4U ||
      options.resampling.duplicate_epsilon < 0.0 ||
      !std::isfinite(options.resampling.duplicate_epsilon)) {
    throw std::invalid_argument("invalid local path resampling options");
  }
  if (options.horizon_mode == LocalHorizonMode::TIME && options.resampling.minimum_points < 4U) {
    throw std::invalid_argument("time horizon requires at least four guide points");
  }
}

void StaticLocalPlanner::validateRequest(const PlanningRequest &request,
                                         const LocalPlannerOptions &options) {
  if (!std::isfinite(request.planning_stamp) || !std::isfinite(request.vehicle_state.stamp) ||
      !finiteVector(request.vehicle_state.position) ||
      !finiteVector(request.vehicle_state.velocity) ||
      !finiteVector(request.vehicle_state.acceleration)) {
    throw std::invalid_argument("planning request vehicle state or timestamp is invalid");
  }
  if (request.global_reference.points.size() < 2U) {
    throw std::invalid_argument("planning request has no usable global reference");
  }
  bool timed = request.global_reference.points.front().has_time;
  double previous_time = -std::numeric_limits<double>::infinity();
  for (const ReferencePoint &point : request.global_reference.points) {
    validateReferencePoint(point);
    if (point.has_time != timed) {
      throw std::invalid_argument("global reference must be consistently timed or untimed");
    }
    if (timed && point.time_from_start <= previous_time) {
      throw std::invalid_argument("global reference times must be strictly increasing");
    }
    if (timed) {
      previous_time = point.time_from_start;
    }
  }
  if (options.horizon_mode == LocalHorizonMode::TIME && !timed) {
    throw std::invalid_argument("time horizon requires a time-parameterized reference");
  }
}

StaticLocalPlanner::Projection StaticLocalPlanner::projectOntoReference(
    const PlanningRequest &request) {
  const auto &points = request.global_reference.points;
  const std::vector<double> lengths = cumulativeLengths(request.global_reference);
  Projection best;
  best.distance_to_path = std::numeric_limits<double>::infinity();

  for (std::size_t segment = 0; segment + 1U < points.size(); ++segment) {
    const Eigen::Vector3d delta = points[segment + 1U].position - points[segment].position;
    const double length = delta.norm();
    if (length <= kEpsilon) {
      continue;
    }
    const double fraction = std::clamp(
        delta.dot(request.vehicle_state.position - points[segment].position) / (length * length),
        0.0, 1.0);
    const Eigen::Vector3d closest = points[segment].position + fraction * delta;
    const double distance = (request.vehicle_state.position - closest).norm();
    if (distance < best.distance_to_path) {
      best.segment = segment;
      best.fraction = fraction;
      best.distance_along_path = lengths[segment] + fraction * length;
      best.distance_to_path = distance;
    }
  }
  if (!std::isfinite(best.distance_to_path)) {
    throw std::invalid_argument("global reference has no non-zero segment");
  }
  return best;
}

ReferencePoint StaticLocalPlanner::interpolate(const ReferencePoint &from,
                                               const ReferencePoint &to, double fraction) {
  const double alpha = std::clamp(fraction, 0.0, 1.0);
  ReferencePoint result;
  result.has_time = from.has_time && to.has_time;
  result.time_from_start = from.time_from_start +
                           alpha * (to.time_from_start - from.time_from_start);
  result.position = (1.0 - alpha) * from.position + alpha * to.position;
  result.velocity = (1.0 - alpha) * from.velocity + alpha * to.velocity;
  result.acceleration = (1.0 - alpha) * from.acceleration + alpha * to.acceleration;
  return result;
}

std::vector<double> StaticLocalPlanner::cumulativeLengths(const GlobalReference &reference) {
  std::vector<double> lengths(reference.points.size(), 0.0);
  for (std::size_t index = 1; index < reference.points.size(); ++index) {
    lengths[index] = lengths[index - 1U] +
                     (reference.points[index].position - reference.points[index - 1U].position)
                         .norm();
  }
  return lengths;
}

ReferencePoint StaticLocalPlanner::pointAtDistance(const GlobalReference &reference,
                                                   const std::vector<double> &cumulative_lengths,
                                                   double distance, std::size_t *segment) {
  const double clamped = std::clamp(distance, 0.0, cumulative_lengths.back());
  if (clamped <= kEpsilon) {
    if (segment != nullptr) {
      *segment = 0U;
    }
    return reference.points.front();
  }
  if (clamped >= cumulative_lengths.back() - kEpsilon) {
    if (segment != nullptr) {
      *segment = reference.points.size() - 2U;
    }
    return reference.points.back();
  }
  for (std::size_t index = 0; index + 1U < reference.points.size(); ++index) {
    const double segment_length = cumulative_lengths[index + 1U] - cumulative_lengths[index];
    if (segment_length > kEpsilon && clamped <= cumulative_lengths[index + 1U]) {
      if (segment != nullptr) {
        *segment = index;
      }
      return interpolate(reference.points[index], reference.points[index + 1U],
                         (clamped - cumulative_lengths[index]) / segment_length);
    }
  }
  throw std::logic_error("failed to locate a distance on the global reference");
}

ReferencePoint StaticLocalPlanner::pointAtTime(const GlobalReference &reference, double time,
                                               std::size_t *segment) {
  const double clamped = std::clamp(time, reference.points.front().time_from_start,
                                    reference.points.back().time_from_start);
  if (clamped <= reference.points.front().time_from_start + kEpsilon) {
    if (segment != nullptr) {
      *segment = 0U;
    }
    return reference.points.front();
  }
  if (clamped >= reference.points.back().time_from_start - kEpsilon) {
    if (segment != nullptr) {
      *segment = reference.points.size() - 2U;
    }
    return reference.points.back();
  }
  for (std::size_t index = 0; index + 1U < reference.points.size(); ++index) {
    const double from = reference.points[index].time_from_start;
    const double to = reference.points[index + 1U].time_from_start;
    if (clamped <= to) {
      if (segment != nullptr) {
        *segment = index;
      }
      return interpolate(reference.points[index], reference.points[index + 1U],
                         (clamped - from) / (to - from));
    }
  }
  throw std::logic_error("failed to locate a time on the global reference");
}

LocalGoal StaticLocalPlanner::extractLocalGoal(const PlanningRequest &request,
                                               const LocalPlannerOptions &options) {
  validateOptions(options);
  validateRequest(request, options);
  const Projection projection = projectOntoReference(request);
  if (std::isfinite(options.max_projection_distance) &&
      projection.distance_to_path > options.max_projection_distance) {
    throw std::invalid_argument("vehicle is too far from the global reference");
  }

  LocalGoal goal;
  const auto lengths = cumulativeLengths(request.global_reference);
  if (options.horizon_mode == LocalHorizonMode::DISTANCE) {
    const double target_distance =
        std::min(projection.distance_along_path + options.local_horizon, lengths.back());
    std::size_t segment = 0U;
    const ReferencePoint target =
        pointAtDistance(request.global_reference, lengths, target_distance, &segment);
    goal.position = target.position;
    goal.velocity = target.velocity;
    goal.acceleration = target.acceleration;
    goal.reference_progress = target_distance;
    goal.reference_time = target.has_time ? target.time_from_start : 0.0;
    goal.source_segment = segment;
    goal.terminal = target_distance >= lengths.back() - kEpsilon;
  } else {
    const ReferencePoint projection_point = pointAtDistance(
        request.global_reference, lengths, projection.distance_along_path, nullptr);
    const double target_time = std::min(
        projection_point.time_from_start + options.local_horizon,
        request.global_reference.points.back().time_from_start);
    std::size_t segment = 0U;
    const ReferencePoint target = pointAtTime(request.global_reference, target_time, &segment);
    goal.position = target.position;
    goal.velocity = target.velocity;
    goal.acceleration = target.acceleration;
    goal.reference_progress = projection.distance_along_path;
    goal.reference_time = target_time;
    goal.source_segment = segment;
    goal.terminal = target_time >= request.global_reference.points.back().time_from_start - kEpsilon;
  }
  return goal;
}

bool StaticLocalPlanner::statesClose(const VehicleState &lhs, const TrajectoryState &rhs,
                                     const LocalPlannerOptions &options) {
  return (lhs.position - rhs.position).norm() <= options.stitch_position_tolerance &&
         (lhs.velocity - rhs.velocity).norm() <= options.stitch_velocity_tolerance &&
         (lhs.acceleration - rhs.acceleration).norm() <= options.stitch_acceleration_tolerance;
}

bool StaticLocalPlanner::validatePrefix(const aurora::map::VoxelMap &map,
                                        const PlannedTrajectory &previous, double start_stamp,
                                        double end_stamp, const LocalPlannerOptions &options) {
  const auto pieces = previous.slice(start_stamp, end_stamp);
  for (const TrajectorySegment &piece : pieces) {
    const auto validation = aurora::trajectory::validateStaticTrajectoryWindow(
        map, piece.spline, piece.source_start_time, piece.duration, options.validation);
    if (!validation.valid) {
      return false;
    }
  }
  return true;
}

PlanningResult StaticLocalPlanner::plan(
    const aurora::map::VoxelMap &map, const PlanningRequest &request,
    const std::optional<PlannedTrajectory> &previous,
    const std::optional<aurora::trajectory::RiskCostFunction> &risk_cost) const {
  PlanningResult result;
  try {
    validateOptions(options_);
    validateRequest(request, options_);
  } catch (const std::exception &error) {
    result.status = request.global_reference.points.empty()
                        ? PlanningStatus::NO_GLOBAL_REFERENCE
                        : PlanningStatus::INVALID_REQUEST;
    result.detail = error.what();
    return result;
  }

  try {
    result.local_goal = extractLocalGoal(request, options_);
  } catch (const std::exception &error) {
    result.status = error.what() != nullptr &&
                            std::string(error.what()).find("horizon") != std::string::npos
                        ? PlanningStatus::INVALID_HORIZON
                        : PlanningStatus::LOCAL_GOAL_UNAVAILABLE;
    result.detail = error.what();
    return result;
  }

  if (result.local_goal.terminal &&
      (request.vehicle_state.position - result.local_goal.position).norm() <=
          options_.goal_tolerance) {
    result.status = PlanningStatus::GOAL_REACHED;
    result.detail = "local goal is within the terminal goal tolerance";
    return result;
  }

  VehicleState boundary_state = request.vehicle_state;
  std::vector<TrajectorySegment> prefix_segments;
  std::string prefix_note;
  if (previous.has_value() && previous->validated && options_.stitch_prefix_duration > 0.0 &&
      previous->contains(request.planning_stamp)) {
    const TrajectoryState previous_now = previous->evaluate(request.planning_stamp);
    const double prefix_end = std::min(request.planning_stamp + options_.stitch_prefix_duration,
                                       previous->endStamp());
    if (statesClose(request.vehicle_state, previous_now, options_) &&
        prefix_end > request.planning_stamp + kEpsilon) {
      try {
        if (validatePrefix(map, *previous, request.planning_stamp, prefix_end, options_)) {
          prefix_segments = previous->slice(request.planning_stamp, prefix_end);
          const TrajectoryState prefix_state = previous->evaluate(prefix_end);
          boundary_state.stamp = prefix_state.stamp;
          boundary_state.position = prefix_state.position;
          boundary_state.velocity = prefix_state.velocity;
          boundary_state.acceleration = prefix_state.acceleration;
          result.stitch_prefix_duration = prefix_end - request.planning_stamp;
          result.used_stitch_prefix = true;
        } else {
          prefix_note = "previous trajectory prefix failed current-map validation; replanned from current state";
        }
      } catch (const std::exception &error) {
        prefix_note = std::string("previous trajectory prefix was unavailable: ") + error.what();
      }
    } else {
      prefix_note = "current state is not close enough to the previous trajectory; replanned from current state";
    }
  }

  const aurora::search::AStar3D search(map);
  auto search_options = options_.search;
  search_options.allow_unknown = !options_.validation.reject_unknown;
  result.search = search.search(boundary_state.position, result.local_goal.position,
                                search_options);
  if (result.search.status != aurora::search::SearchStatus::SUCCESS) {
    result.status = PlanningStatus::SEARCH_FAILED;
    result.detail = std::string("local search failed: ") +
                    aurora::search::toString(result.search.status) + " (" + result.search.detail + ")";
    return result;
  }

  try {
    const auto guide_points = aurora::math::resamplePath(result.search.path, options_.resampling);
    const auto initial_control_points = aurora::math::UniformBspline::parameterizeToControlPoints(
        guide_points, options_.optimizer.interval, boundary_state.velocity,
        result.local_goal.velocity, boundary_state.acceleration, result.local_goal.acceleration);

    LocalPlannerOptions planning_options = options_;
    planning_options.optimizer.risk_time_origin =
        request.planning_stamp + result.stitch_prefix_duration;
    if (risk_cost.has_value()) {
      planning_options.optimizer.risk_cost = *risk_cost;
    }
    aurora::trajectory::StaticBsplineOptimizer optimizer(
        map, initial_control_points, initial_control_points, planning_options.optimizer);
    const auto initial_cost = optimizer.evaluate(initial_control_points, false);
    result.initial_cost = initial_cost.total;
    result.optimization = optimizer.optimize();
    result.final_cost = result.optimization.cost.total;

    const aurora::math::UniformBspline optimized_spline(
        result.optimization.control_points, options_.optimizer.interval,
        options_.optimizer.knot_mode);
    result.validation = aurora::trajectory::validateStaticTrajectory(
        map, optimized_spline, options_.validation);
    if (!result.validation.valid) {
      result.status = PlanningStatus::VALIDATION_FAILED;
      result.detail = std::string("optimized trajectory failed validation: ") +
                      result.validation.detail;
      return result;
    }

    TrajectorySegment new_segment{request.planning_stamp + result.stitch_prefix_duration,
                                 0.0, optimized_spline.duration(), optimized_spline};

    PlannedTrajectory planned;
    planned.trajectory_id = request.request_id;
    planned.map_version = map.version();
    planned.segments = std::move(prefix_segments);
    planned.segments.push_back(std::move(new_segment));

    StaticSafetyGateOptions gate_options;
    gate_options.validation = options_.validation;
    gate_options.position_tolerance = options_.stitch_position_tolerance;
    gate_options.velocity_tolerance = options_.stitch_velocity_tolerance;
    gate_options.acceleration_tolerance = options_.stitch_acceleration_tolerance;
    const StaticSafetyGate gate(gate_options);
    const auto gate_result = gate.evaluate(map, planned, request.planning_stamp, previous);
    result.validation = gate_result.validation;
    if (!gate_result.accepted) {
      result.status = PlanningStatus::VALIDATION_FAILED;
      result.detail = std::string("static safety gate rejected trajectory with ") +
                      toString(gate_result.status) + ": " + gate_result.detail;
      return result;
    }

    planned.validated = true;
    result.trajectory = std::move(planned);
    result.status = PlanningStatus::SUCCESS;
    result.detail = prefix_note.empty() ? "static local trajectory validated" :
                                         "static local trajectory validated; " + prefix_note;
    return result;
  } catch (const std::exception &error) {
    result.status = PlanningStatus::OPTIMIZATION_FAILED;
    result.detail = std::string("local trajectory generation failed: ") + error.what();
    return result;
  }
}

}  // namespace aurora::planner
