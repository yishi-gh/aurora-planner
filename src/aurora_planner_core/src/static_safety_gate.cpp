#include "aurora_planner_core/static_safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace aurora::planner {
namespace {

constexpr double kEpsilon = 1e-9;

bool finiteState(const TrajectoryState &state) {
  return std::isfinite(state.stamp) && state.position.allFinite() && state.velocity.allFinite() &&
         state.acceleration.allFinite();
}

bool closeState(const TrajectoryState &lhs, const TrajectoryState &rhs,
                const StaticSafetyGateOptions &options) {
  return (lhs.position - rhs.position).norm() <= options.position_tolerance &&
         (lhs.velocity - rhs.velocity).norm() <= options.velocity_tolerance &&
         (lhs.acceleration - rhs.acceleration).norm() <= options.acceleration_tolerance;
}

bool finiteNonNegative(double value) { return std::isfinite(value) && value >= 0.0; }

bool finitePositive(double value) { return std::isfinite(value) && value > 0.0; }

SafetyGateStatus mapValidationStatus(aurora::trajectory::ValidationStatus status) {
  switch (status) {
    case aurora::trajectory::ValidationStatus::OUT_OF_MAP:
      return SafetyGateStatus::OUT_OF_MAP;
    case aurora::trajectory::ValidationStatus::OCCUPIED:
      return SafetyGateStatus::STATIC_COLLISION;
    case aurora::trajectory::ValidationStatus::UNKNOWN:
      return SafetyGateStatus::UNKNOWN_SPACE;
    case aurora::trajectory::ValidationStatus::VELOCITY_LIMIT:
      return SafetyGateStatus::VELOCITY_LIMIT;
    case aurora::trajectory::ValidationStatus::ACCELERATION_LIMIT:
      return SafetyGateStatus::ACCELERATION_LIMIT;
    case aurora::trajectory::ValidationStatus::NONFINITE:
      return SafetyGateStatus::INVALID_SEGMENT;
    case aurora::trajectory::ValidationStatus::INVALID_OPTIONS:
      return SafetyGateStatus::INVALID_OPTIONS;
    case aurora::trajectory::ValidationStatus::VALID:
      return SafetyGateStatus::ACCEPTED;
  }
  return SafetyGateStatus::INVALID_SEGMENT;
}

bool validateSegmentShape(const TrajectorySegment &segment, std::string *detail) {
  if (!std::isfinite(segment.start_stamp) || !std::isfinite(segment.source_start_time) ||
      !finitePositive(segment.duration) || segment.source_start_time < 0.0 ||
      !std::isfinite(segment.spline.duration()) || segment.spline.duration() <= 0.0 ||
      !std::isfinite(segment.start_stamp + segment.duration) ||
      segment.source_start_time > segment.spline.duration() + kEpsilon ||
      segment.source_start_time + segment.duration > segment.spline.duration() + kEpsilon) {
    if (detail != nullptr) {
      *detail = "trajectory segment has an invalid time window";
    }
    return false;
  }
  return true;
}

enum class TrajectoryStructureStatus {
  VALID,
  EMPTY,
  INVALID_SEGMENT,
  TIME_DISCONTINUITY,
};

TrajectoryStructureStatus validateTrajectoryStructure(const PlannedTrajectory &trajectory,
                                                      double time_tolerance,
                                                      std::string *detail) {
  if (trajectory.empty()) {
    if (detail != nullptr) {
      *detail = "candidate trajectory is empty";
    }
    return TrajectoryStructureStatus::EMPTY;
  }

  double expected_start = trajectory.segments.front().start_stamp;
  for (std::size_t index = 0; index < trajectory.segments.size(); ++index) {
    const TrajectorySegment &segment = trajectory.segments[index];
    if (!validateSegmentShape(segment, detail)) {
      return TrajectoryStructureStatus::INVALID_SEGMENT;
    }
    if (index > 0U && std::abs(segment.start_stamp - expected_start) > time_tolerance) {
      if (detail != nullptr) {
        *detail = "trajectory segments are not strictly time-contiguous";
      }
      return TrajectoryStructureStatus::TIME_DISCONTINUITY;
    }
    expected_start = segment.endStamp();
  }
  return TrajectoryStructureStatus::VALID;
}

void accumulateValidation(StaticSafetyGateResult *result,
                          const aurora::trajectory::StaticTrajectoryValidationResult &validation) {
  result->checked_samples += validation.checked_samples;
  result->occupied_samples += validation.occupied_samples;
  result->unknown_samples += validation.unknown_samples;
  result->maximum_velocity = std::max(result->maximum_velocity, validation.maximum_velocity);
  result->maximum_acceleration =
      std::max(result->maximum_acceleration, validation.maximum_acceleration);
}

}  // namespace

const char *toString(SafetyGateStatus status) noexcept {
  switch (status) {
    case SafetyGateStatus::ACCEPTED:
      return "ACCEPTED";
    case SafetyGateStatus::INVALID_OPTIONS:
      return "INVALID_OPTIONS";
    case SafetyGateStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case SafetyGateStatus::EMPTY_CANDIDATE:
      return "EMPTY_CANDIDATE";
    case SafetyGateStatus::INVALID_SEGMENT:
      return "INVALID_SEGMENT";
    case SafetyGateStatus::TIME_DISCONTINUITY:
      return "TIME_DISCONTINUITY";
    case SafetyGateStatus::STATE_DISCONTINUITY:
      return "STATE_DISCONTINUITY";
    case SafetyGateStatus::CURRENT_STATE_DISCONTINUITY:
      return "CURRENT_STATE_DISCONTINUITY";
    case SafetyGateStatus::OUT_OF_MAP:
      return "OUT_OF_MAP";
    case SafetyGateStatus::STATIC_COLLISION:
      return "STATIC_COLLISION";
    case SafetyGateStatus::UNKNOWN_SPACE:
      return "UNKNOWN_SPACE";
    case SafetyGateStatus::VELOCITY_LIMIT:
      return "VELOCITY_LIMIT";
    case SafetyGateStatus::ACCELERATION_LIMIT:
      return "ACCELERATION_LIMIT";
  }
  return "UNKNOWN_STATUS";
}

StaticSafetyGate::StaticSafetyGate(StaticSafetyGateOptions options) : options_(std::move(options)) {
  validateOptions(options_);
}

void StaticSafetyGate::validateOptions(const StaticSafetyGateOptions &options) {
  if (!finiteNonNegative(options.time_tolerance) ||
      !finiteNonNegative(options.position_tolerance) ||
      !finiteNonNegative(options.velocity_tolerance) ||
      !finiteNonNegative(options.acceleration_tolerance) ||
      options.validation.samples_per_span == 0U ||
      !std::isfinite(options.validation.max_velocity) || options.validation.max_velocity <= 0.0 ||
      !std::isfinite(options.validation.max_acceleration) ||
      options.validation.max_acceleration <= 0.0 ||
      !std::isfinite(options.validation.tolerance) || options.validation.tolerance < 0.0) {
    throw std::invalid_argument("invalid static safety gate options");
  }
}

StaticSafetyGateResult StaticSafetyGate::evaluate(
    const aurora::map::VoxelMap &map, const PlannedTrajectory &candidate, double now,
    const std::optional<PlannedTrajectory> &current) const {
  StaticSafetyGateResult result;
  result.current_trajectory_available = current.has_value() && !current->empty();

  if (!std::isfinite(now)) {
    result.status = SafetyGateStatus::INVALID_INPUT;
    result.detail = "safety gate evaluation time is not finite";
    return result;
  }
  if (candidate.empty()) {
    result.status = SafetyGateStatus::EMPTY_CANDIDATE;
    result.detail = "candidate trajectory is empty";
    return result;
  }

  result.candidate_start_stamp = candidate.startStamp();
  result.candidate_end_stamp = candidate.endStamp();

  std::string structure_detail;
  const auto structure_status =
      validateTrajectoryStructure(candidate, options_.time_tolerance, &structure_detail);
  if (structure_status != TrajectoryStructureStatus::VALID) {
    result.status = structure_status == TrajectoryStructureStatus::EMPTY
                        ? SafetyGateStatus::EMPTY_CANDIDATE
                        : structure_status == TrajectoryStructureStatus::TIME_DISCONTINUITY
                            ? SafetyGateStatus::TIME_DISCONTINUITY
                            : SafetyGateStatus::INVALID_SEGMENT;
    result.detail = std::move(structure_detail);
    return result;
  }
  if (options_.require_start_at_now &&
      std::abs(result.candidate_start_stamp - now) > options_.time_tolerance) {
    result.status = SafetyGateStatus::TIME_DISCONTINUITY;
    result.detail = "candidate trajectory does not start at the gate evaluation time";
    return result;
  }

  for (std::size_t index = 1U; index < candidate.segments.size(); ++index) {
    const TrajectoryState before = candidate.segments[index - 1U].evaluate(
        candidate.segments[index - 1U].endStamp());
    const TrajectoryState after = candidate.segments[index].evaluate(
        candidate.segments[index].start_stamp);
    if (!finiteState(before) || !finiteState(after)) {
      result.status = SafetyGateStatus::INVALID_SEGMENT;
      result.failed_segment = index;
      result.detail = "trajectory segment boundary state is non-finite";
      return result;
    }
    if (!closeState(before, after, options_)) {
      result.status = SafetyGateStatus::STATE_DISCONTINUITY;
      result.failed_segment = index;
      result.detail = "trajectory segment boundary state is discontinuous";
      return result;
    }
  }

  if (current.has_value() && !current->empty()) {
    // Fallback certification is independent from candidate continuity. A
    // rejected candidate must not hide a still-usable active trajectory.
    if (current->validated && current->contains(now) && current->endStamp() > now + kEpsilon) {
      try {
        const auto current_pieces = current->slice(now, current->endStamp());
        bool safe = true;
        for (const TrajectorySegment &piece : current_pieces) {
          const auto validation = aurora::trajectory::validateStaticTrajectoryWindow(
              map, piece.spline, piece.source_start_time, piece.duration, options_.validation);
          if (!validation.valid) {
            safe = false;
            break;
          }
        }
        result.current_trajectory_fallback_available = safe;
      } catch (const std::exception &) {
        result.current_trajectory_fallback_available = false;
      }
    }

    if (current->contains(now)) {
      try {
        const TrajectoryState current_state = current->evaluate(now);
        const TrajectoryState candidate_state = candidate.evaluate(now);
        result.current_state_checked = true;
        result.current_state_continuous = closeState(current_state, candidate_state, options_);
        if (!result.current_state_continuous) {
          result.status = SafetyGateStatus::CURRENT_STATE_DISCONTINUITY;
          result.detail = "candidate does not continue the current trajectory state";
          return result;
        }
      } catch (const std::exception &error) {
        result.status = SafetyGateStatus::CURRENT_STATE_DISCONTINUITY;
        result.detail = std::string("current trajectory state could not be evaluated: ") +
                        error.what();
        return result;
      }
    }

  }

  result.validation.status = aurora::trajectory::ValidationStatus::VALID;
  result.validation.valid = true;
  result.checked_segments = 0U;
  for (std::size_t index = 0; index < candidate.segments.size(); ++index) {
    const TrajectorySegment &segment = candidate.segments[index];
    result.validation = aurora::trajectory::validateStaticTrajectoryWindow(
        map, segment.spline, segment.source_start_time, segment.duration, options_.validation);
    accumulateValidation(&result, result.validation);
    ++result.checked_segments;
    if (!result.validation.valid) {
      result.failed_segment = index;
      result.status = mapValidationStatus(result.validation.status);
      result.detail = std::string("candidate segment failed static validation: ") +
                      result.validation.detail;
      return result;
    }
  }

  result.status = SafetyGateStatus::ACCEPTED;
  result.accepted = true;
  result.detail = "candidate trajectory passed static safety gating";
  return result;
}

}  // namespace aurora::planner
