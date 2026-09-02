#include "aurora_sim/trajectory_executor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aurora::simulation {
namespace {

constexpr double kEpsilon = 1e-12;

bool finiteVector(const Eigen::Vector3d &value) { return value.allFinite(); }

Eigen::Vector3d clampNorm(const Eigen::Vector3d &value, double limit) {
  const double norm = value.norm();
  if (norm <= limit || norm <= kEpsilon) {
    return value;
  }
  return value * (limit / norm);
}

}  // namespace

const char *toString(ExecutionStatus status) noexcept {
  switch (status) {
    case ExecutionStatus::IDLE:
      return "IDLE";
    case ExecutionStatus::ACTIVE:
      return "ACTIVE";
    case ExecutionStatus::COMPLETED:
      return "COMPLETED";
    case ExecutionStatus::REJECTED_UNVALIDATED:
      return "REJECTED_UNVALIDATED";
    case ExecutionStatus::REJECTED_UNSAFE:
      return "REJECTED_UNSAFE";
    case ExecutionStatus::REJECTED_INVALID:
      return "REJECTED_INVALID";
    case ExecutionStatus::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case ExecutionStatus::TIME_ROLLBACK:
      return "TIME_ROLLBACK";
    case ExecutionStatus::TIME_GAP:
      return "TIME_GAP";
  }
  return "UNKNOWN";
}

TrajectoryExecutor::TrajectoryExecutor(ExecutorOptions options, FlightState initial_state)
    : options_(std::move(options)), state_(std::move(initial_state)),
      last_desired_(state_) {
  validateOptions(options_);
  if (!std::isfinite(state_.stamp) || !finiteVector(state_.position) ||
      !finiteVector(state_.velocity) || !finiteVector(state_.acceleration)) {
    throw std::invalid_argument("initial flight state must be finite");
  }
}

void TrajectoryExecutor::validateOptions(const ExecutorOptions &options) {
  if (!std::isfinite(options.max_velocity) || options.max_velocity <= 0.0 ||
      !std::isfinite(options.max_acceleration) || options.max_acceleration <= 0.0 ||
      !std::isfinite(options.position_gain) || options.position_gain < 0.0 ||
      !std::isfinite(options.velocity_gain) || options.velocity_gain < 0.0 ||
      !std::isfinite(options.max_integration_step) || options.max_integration_step <= 0.0 ||
      !std::isfinite(options.max_update_gap) || options.max_update_gap <= 0.0 ||
      !std::isfinite(options.time_tolerance) || options.time_tolerance < 0.0) {
    throw std::invalid_argument("invalid trajectory executor options");
  }
}

AcceptanceResult TrajectoryExecutor::validateTrajectory(const ExecutorTrajectory &trajectory,
                                                         double now, double tolerance) {
  if (!std::isfinite(now)) {
    return {false, ExecutionStatus::REJECTED_INVALID, "execution time is not finite"};
  }
  if (trajectory.segments.empty()) {
    return {false, ExecutionStatus::REJECTED_INVALID, "trajectory has no segments"};
  }
  double previous_end = std::numeric_limits<double>::quiet_NaN();
  for (const ExecutorSegment &segment : trajectory.segments) {
    if (!std::isfinite(segment.start_stamp) || !std::isfinite(segment.source_start_time) ||
        !std::isfinite(segment.duration) || segment.duration <= 0.0 ||
        segment.source_start_time < -tolerance ||
        segment.source_start_time > segment.spline.duration() + tolerance) {
      return {false, ExecutionStatus::REJECTED_INVALID, "trajectory contains an invalid segment"};
    }
    if (std::isfinite(previous_end) &&
        std::abs(segment.start_stamp - previous_end) > tolerance) {
      return {false, ExecutionStatus::REJECTED_INVALID,
              "trajectory segments are not contiguous in absolute time"};
    }
    previous_end = segment.endStamp();
  }
  if (!std::isfinite(previous_end) || now > previous_end + tolerance) {
    return {false, ExecutionStatus::REJECTED_INVALID, "trajectory has already expired"};
  }
  return {true, ExecutionStatus::ACTIVE, "trajectory accepted"};
}

AcceptanceResult TrajectoryExecutor::accept(const ExecutorTrajectory &trajectory, double now) {
  if (emergency_stop_active_) {
    return {false, ExecutionStatus::EMERGENCY_STOP,
            "trajectory rejected while emergency stop is active"};
  }
  if (!trajectory.validated) {
    return {false, ExecutionStatus::REJECTED_UNVALIDATED,
            "trajectory validation_state is not VALIDATED"};
  }
  if (!trajectory.safety_accepted) {
    return {false, ExecutionStatus::REJECTED_UNSAFE,
            "trajectory safety report is not accepted"};
  }
  const AcceptanceResult validation =
      validateTrajectory(trajectory, now, options_.time_tolerance);
  if (!validation.accepted) {
    return validation;
  }
  if (std::isfinite(last_update_stamp_) &&
      now + options_.time_tolerance < last_update_stamp_) {
    return {false, ExecutionStatus::TIME_ROLLBACK,
            "trajectory timestamp is older than the execution clock"};
  }

  active_segments_ = trajectory.segments;
  active_trajectory_id_ = trajectory.trajectory_id;
  has_active_trajectory_ = true;
  last_update_stamp_ = now;
  state_.stamp = now;
  last_desired_ = desiredAt(now);
  last_status_ = ExecutionStatus::ACTIVE;
  last_detail_ = "trajectory accepted by the execution gate";
  return validation;
}

const ExecutorSegment *TrajectoryExecutor::segmentAt(double stamp) const noexcept {
  if (active_segments_.empty() || !std::isfinite(stamp)) {
    return nullptr;
  }
  const ExecutorSegment *selected = &active_segments_.front();
  for (const ExecutorSegment &segment : active_segments_) {
    if (stamp + options_.time_tolerance >= segment.start_stamp) {
      selected = &segment;
    } else {
      break;
    }
  }
  return selected;
}

FlightState TrajectoryExecutor::desiredAt(double stamp) const {
  const ExecutorSegment *segment = segmentAt(stamp);
  if (segment == nullptr) {
    return state_;
  }
  const double relative_time =
      std::clamp(stamp - segment->start_stamp, 0.0, segment->duration);
  const double spline_time = std::clamp(
      segment->source_start_time + relative_time, 0.0, segment->spline.duration());
  FlightState desired;
  desired.stamp = stamp;
  desired.position = segment->spline.evaluate(spline_time, 0);
  desired.velocity = segment->spline.evaluate(spline_time, 1);
  desired.acceleration = segment->spline.evaluate(spline_time, 2);
  return desired;
}

void TrajectoryExecutor::integrate(double step, double stamp) {
  const FlightState desired = desiredAt(stamp);
  const Eigen::Vector3d acceleration = clampNorm(
      desired.acceleration + options_.position_gain * (desired.position - state_.position) +
          options_.velocity_gain * (desired.velocity - state_.velocity),
      options_.max_acceleration);
  state_.position += state_.velocity * step + 0.5 * acceleration * step * step;
  state_.velocity = clampNorm(state_.velocity + acceleration * step, options_.max_velocity);
  state_.acceleration = acceleration;
  state_.stamp = stamp;
  last_desired_ = desired;
}

ExecutorOutput TrajectoryExecutor::makeOutput(ExecutionStatus status, bool active, double now,
                                               const std::string &detail) const {
  ExecutorOutput output;
  output.status = status;
  output.active = active;
  output.trajectory_id = active_trajectory_id_;
  output.state = state_;
  output.state.stamp = now;
  output.desired = active ? last_desired_ : state_;
  output.desired.stamp = now;
  output.tracking_error = (output.desired.position - output.state.position).norm();
  output.detail = detail;
  return output;
}

ExecutorOutput TrajectoryExecutor::update(double now) {
  if (!std::isfinite(now)) {
    last_status_ = ExecutionStatus::REJECTED_INVALID;
    last_detail_ = "execution clock is not finite";
    return makeOutput(last_status_, false, state_.stamp, last_detail_);
  }
  if (emergency_stop_active_) {
    state_.stamp = now;
    state_.acceleration.setZero();
    state_.velocity.setZero();
    last_status_ = ExecutionStatus::EMERGENCY_STOP;
    last_detail_ = "emergency stop is active";
    return makeOutput(last_status_, false, now, last_detail_);
  }
  if (std::isfinite(last_update_stamp_) && now + options_.time_tolerance < last_update_stamp_) {
    has_active_trajectory_ = false;
    active_segments_.clear();
    state_.stamp = now;
    state_.velocity.setZero();
    state_.acceleration.setZero();
    emergency_stop_active_ = true;
    last_status_ = ExecutionStatus::TIME_ROLLBACK;
    last_detail_ = "execution clock moved backwards; fail closed";
    return makeOutput(last_status_, false, now, last_detail_);
  }
  if (!std::isfinite(last_update_stamp_)) {
    last_update_stamp_ = now;
  }
  const double elapsed = now - last_update_stamp_;
  if (elapsed > options_.max_update_gap + options_.time_tolerance) {
    has_active_trajectory_ = false;
    active_segments_.clear();
    state_.stamp = now;
    state_.velocity.setZero();
    state_.acceleration.setZero();
    emergency_stop_active_ = true;
    last_update_stamp_ = now;
    last_status_ = ExecutionStatus::TIME_GAP;
    last_detail_ = "execution update gap exceeded the configured watchdog";
    return makeOutput(last_status_, false, now, last_detail_);
  }

  double cursor = last_update_stamp_;
  while (cursor + options_.time_tolerance < now) {
    const double step = std::min(options_.max_integration_step, now - cursor);
    cursor += step;
    if (has_active_trajectory_) {
      integrate(step, cursor);
    } else {
      state_.stamp = cursor;
      state_.acceleration.setZero();
      state_.velocity *= std::exp(-options_.velocity_gain * step);
      last_desired_ = state_;
    }
  }
  last_update_stamp_ = now;
  if (has_active_trajectory_ && now >= active_segments_.back().endStamp() - options_.time_tolerance) {
    has_active_trajectory_ = false;
    last_status_ = ExecutionStatus::COMPLETED;
    last_detail_ = "trajectory time window completed";
  } else {
    last_status_ = has_active_trajectory_ ? ExecutionStatus::ACTIVE : ExecutionStatus::IDLE;
    last_detail_ = has_active_trajectory_ ? "trajectory is being executed" : "holding position";
  }
  return makeOutput(last_status_, has_active_trajectory_, now, last_detail_);
}

void TrajectoryExecutor::setEmergencyStop(bool active) noexcept {
  emergency_stop_active_ = active;
  if (active) {
    has_active_trajectory_ = false;
    active_segments_.clear();
    state_.velocity.setZero();
    state_.acceleration.setZero();
    last_status_ = ExecutionStatus::EMERGENCY_STOP;
    last_detail_ = "emergency stop engaged";
  } else {
    last_status_ = ExecutionStatus::IDLE;
    last_detail_ = "emergency stop reset";
    last_update_stamp_ = std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace aurora::simulation
