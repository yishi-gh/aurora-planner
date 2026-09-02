#pragma once

#include "aurora_math/uniform_bspline.hpp"
#include "aurora_msgs/msg/trajectory.hpp"
#include "aurora_msgs/msg/trajectory_execution_status.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace aurora::flight {

enum class AdmissionStatus {
  ACCEPTED,
  EMERGENCY_STOP,
  REJECTED_UNVALIDATED,
  REJECTED_UNSAFE,
  REJECTED_INVALID,
  EXPIRED,
  FRAME_MISMATCH,
  SETPOINT_LIMIT,
};

const char *toString(AdmissionStatus status) noexcept;

enum class FeedbackAction {
  KEEP_EXECUTING,
  COMPLETED,
  REQUEST_REPLAN,
  EMERGENCY_STOP,
};

const char *toString(FeedbackAction action) noexcept;

struct AdapterOptions {
  std::string expected_frame{"map"};
  double time_tolerance{1e-6};
  double setpoint_interval{0.05};
  std::size_t max_setpoints{2000U};
};

struct FlightSetpoint {
  double stamp{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct AdmittedSegment {
  double start_stamp{std::numeric_limits<double>::quiet_NaN()};
  double source_start_time{std::numeric_limits<double>::quiet_NaN()};
  double duration{0.0};
  aurora::math::UniformBspline spline;

  double endStamp() const noexcept { return start_stamp + duration; }
};

struct AdmittedTrajectory {
  std::uint64_t trajectory_id{0U};
  std::string frame_id;
  std::vector<AdmittedSegment> segments;
  std::vector<FlightSetpoint> setpoints;

  double startStamp() const noexcept {
    return segments.empty() ? std::numeric_limits<double>::quiet_NaN()
                            : segments.front().start_stamp;
  }

  double endStamp() const noexcept {
    return segments.empty() ? std::numeric_limits<double>::quiet_NaN()
                            : segments.back().start_stamp + segments.back().duration;
  }
};

struct AdmissionResult {
  bool accepted{false};
  AdmissionStatus status{AdmissionStatus::REJECTED_INVALID};
  std::string detail;
  std::optional<AdmittedTrajectory> trajectory;
};

struct FeedbackResult {
  bool valid{false};
  FeedbackAction action{FeedbackAction::EMERGENCY_STOP};
  std::string detail;
};

// The admission boundary shared by PX4/GZ, MAVROS and the deterministic
// software-in-the-loop executor. It does not send commands or depend on a
// particular flight controller; concrete adapters consume the admitted data.
class TrajectoryAdmission {
public:
  explicit TrajectoryAdmission(AdapterOptions options = {});

  const AdapterOptions &options() const noexcept { return options_; }

  AdmissionResult admit(const aurora_msgs::msg::Trajectory &message,
                        double now,
                        bool emergency_stop = false) const;

  FeedbackResult observeFeedback(
      const aurora_msgs::msg::TrajectoryExecutionStatus &feedback,
      std::uint64_t expected_trajectory_id,
      double now) const;

private:
  static void validateOptions(const AdapterOptions &options);
  static bool validRosTime(const builtin_interfaces::msg::Time &time);
  static double timeToSeconds(const builtin_interfaces::msg::Time &time);
  static bool finitePoint(const geometry_msgs::msg::Point &point);
  static bool validFeedbackStatus(std::uint8_t status);

  AdapterOptions options_;
};

}  // namespace aurora::flight
