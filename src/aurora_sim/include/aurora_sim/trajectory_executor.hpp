#pragma once

#include "aurora_math/uniform_bspline.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace aurora::simulation {

enum class ExecutionStatus {
  IDLE,
  ACTIVE,
  COMPLETED,
  REJECTED_UNVALIDATED,
  REJECTED_UNSAFE,
  REJECTED_INVALID,
  EMERGENCY_STOP,
  TIME_ROLLBACK,
  TIME_GAP,
};

const char *toString(ExecutionStatus status) noexcept;

struct ExecutorSegment {
  double start_stamp{std::numeric_limits<double>::quiet_NaN()};
  double source_start_time{std::numeric_limits<double>::quiet_NaN()};
  double duration{0.0};
  aurora::math::UniformBspline spline;

  double endStamp() const noexcept { return start_stamp + duration; }
};

struct ExecutorTrajectory {
  std::uint64_t trajectory_id{0U};
  bool validated{false};
  bool safety_accepted{false};
  std::vector<ExecutorSegment> segments;
};

struct FlightState {
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct ExecutorOptions {
  double max_velocity{3.0};
  double max_acceleration{6.0};
  double position_gain{6.0};
  double velocity_gain{4.0};
  double max_integration_step{0.01};
  double max_update_gap{0.5};
  double time_tolerance{1e-6};
};

struct ExecutorOutput {
  ExecutionStatus status{ExecutionStatus::IDLE};
  bool active{false};
  std::uint64_t trajectory_id{0U};
  FlightState state;
  FlightState desired;
  double tracking_error{0.0};
  std::string detail;
};

struct AcceptanceResult {
  bool accepted{false};
  ExecutionStatus status{ExecutionStatus::REJECTED_INVALID};
  std::string detail;
};

// A deterministic, acceleration-limited 3D execution model. It is deliberately
// independent of ROS and can be used as a software-in-the-loop substitute for
// a real flight controller in repeatable tests.
class TrajectoryExecutor {
public:
  explicit TrajectoryExecutor(ExecutorOptions options = {}, FlightState initial_state = {});

  const ExecutorOptions &options() const noexcept { return options_; }
  const FlightState &state() const noexcept { return state_; }
  bool emergencyStopActive() const noexcept { return emergency_stop_active_; }
  std::uint64_t activeTrajectoryId() const noexcept { return active_trajectory_id_; }

  AcceptanceResult accept(const ExecutorTrajectory &trajectory, double now);
  ExecutorOutput update(double now);
  void setEmergencyStop(bool active) noexcept;

private:
  static void validateOptions(const ExecutorOptions &options);
  static AcceptanceResult validateTrajectory(const ExecutorTrajectory &trajectory,
                                             double now, double tolerance);
  const ExecutorSegment *segmentAt(double stamp) const noexcept;
  FlightState desiredAt(double stamp) const;
  void integrate(double step, double stamp);
  ExecutorOutput makeOutput(ExecutionStatus status, bool active, double now,
                            const std::string &detail) const;

  ExecutorOptions options_;
  FlightState state_;
  FlightState last_desired_;
  std::vector<ExecutorSegment> active_segments_;
  std::uint64_t active_trajectory_id_{0U};
  double last_update_stamp_{std::numeric_limits<double>::quiet_NaN()};
  bool has_active_trajectory_{false};
  bool emergency_stop_active_{false};
  ExecutionStatus last_status_{ExecutionStatus::IDLE};
  std::string last_detail_;
};

}  // namespace aurora::simulation
