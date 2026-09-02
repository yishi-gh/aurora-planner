#pragma once

#include "aurora_planner_core/static_local_planner.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <string>

namespace aurora::planner {

enum class SafetyGateStatus {
  ACCEPTED,
  INVALID_OPTIONS,
  INVALID_INPUT,
  EMPTY_CANDIDATE,
  INVALID_SEGMENT,
  TIME_DISCONTINUITY,
  STATE_DISCONTINUITY,
  CURRENT_STATE_DISCONTINUITY,
  OUT_OF_MAP,
  STATIC_COLLISION,
  UNKNOWN_SPACE,
  VELOCITY_LIMIT,
  ACCELERATION_LIMIT,
};

const char *toString(SafetyGateStatus status) noexcept;

struct StaticSafetyGateOptions {
  aurora::trajectory::StaticTrajectoryValidationOptions validation;
  double time_tolerance{1e-6};
  double position_tolerance{1e-5};
  double velocity_tolerance{1e-5};
  double acceleration_tolerance{1e-5};
  bool require_start_at_now{true};
};

struct StaticSafetyGateResult {
  SafetyGateStatus status{SafetyGateStatus::INVALID_OPTIONS};
  bool accepted{false};

  std::size_t checked_segments{0};
  std::size_t failed_segment{std::numeric_limits<std::size_t>::max()};
  std::size_t checked_samples{0};
  std::size_t occupied_samples{0};
  std::size_t unknown_samples{0};
  double maximum_velocity{0.0};
  double maximum_acceleration{0.0};

  double candidate_start_stamp{std::numeric_limits<double>::quiet_NaN()};
  double candidate_end_stamp{std::numeric_limits<double>::quiet_NaN()};

  bool current_trajectory_available{false};
  bool current_state_checked{false};
  bool current_state_continuous{false};
  bool current_trajectory_fallback_available{false};

  // The last static validation result is retained for diagnostics. It is not
  // a permission to publish unless status is ACCEPTED.
  aurora::trajectory::StaticTrajectoryValidationResult validation;
  std::string detail;
};

// Pure C++ publish gate for the static planning phase. It certifies the
// candidate and reports whether the previous trajectory is still a usable
// fallback. FSM actions remain outside this class.
class StaticSafetyGate {
public:
  explicit StaticSafetyGate(StaticSafetyGateOptions options = {});

  const StaticSafetyGateOptions &options() const noexcept { return options_; }

  StaticSafetyGateResult evaluate(
      const aurora::map::VoxelMap &map, const PlannedTrajectory &candidate, double now,
      const std::optional<PlannedTrajectory> &current = std::nullopt) const;

private:
  static void validateOptions(const StaticSafetyGateOptions &options);

  StaticSafetyGateOptions options_;
};

}  // namespace aurora::planner
