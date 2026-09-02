#include "aurora_flight_adapter/trajectory_admission.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::flight {
namespace {

constexpr double kEpsilon = 1e-12;

bool finiteVector(const Eigen::Vector3d &value) { return value.allFinite(); }

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &point) {
  return {point.x, point.y, point.z};
}

}  // namespace

const char *toString(AdmissionStatus status) noexcept {
  switch (status) {
    case AdmissionStatus::ACCEPTED:
      return "ACCEPTED";
    case AdmissionStatus::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case AdmissionStatus::REJECTED_UNVALIDATED:
      return "REJECTED_UNVALIDATED";
    case AdmissionStatus::REJECTED_UNSAFE:
      return "REJECTED_UNSAFE";
    case AdmissionStatus::REJECTED_INVALID:
      return "REJECTED_INVALID";
    case AdmissionStatus::EXPIRED:
      return "EXPIRED";
    case AdmissionStatus::FRAME_MISMATCH:
      return "FRAME_MISMATCH";
    case AdmissionStatus::SETPOINT_LIMIT:
      return "SETPOINT_LIMIT";
  }
  return "UNKNOWN";
}

const char *toString(FeedbackAction action) noexcept {
  switch (action) {
    case FeedbackAction::KEEP_EXECUTING:
      return "KEEP_EXECUTING";
    case FeedbackAction::COMPLETED:
      return "COMPLETED";
    case FeedbackAction::REQUEST_REPLAN:
      return "REQUEST_REPLAN";
    case FeedbackAction::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

TrajectoryAdmission::TrajectoryAdmission(AdapterOptions options)
    : options_(std::move(options)) {
  validateOptions(options_);
}

void TrajectoryAdmission::validateOptions(const AdapterOptions &options) {
  if (options.expected_frame.empty() || !std::isfinite(options.time_tolerance) ||
      options.time_tolerance < 0.0 || !std::isfinite(options.setpoint_interval) ||
      options.setpoint_interval <= 0.0 || options.max_setpoints == 0U) {
    throw std::invalid_argument("invalid flight trajectory adapter options");
  }
}

bool TrajectoryAdmission::validRosTime(const builtin_interfaces::msg::Time &time) {
  return time.sec >= 0 && time.nanosec < 1000000000U;
}

double TrajectoryAdmission::timeToSeconds(const builtin_interfaces::msg::Time &time) {
  return static_cast<double>(time.sec) + 1e-9 * static_cast<double>(time.nanosec);
}

bool TrajectoryAdmission::finitePoint(const geometry_msgs::msg::Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool TrajectoryAdmission::validFeedbackStatus(std::uint8_t status) {
  return status <= aurora_msgs::msg::TrajectoryExecutionStatus::TIME_GAP;
}

AdmissionResult TrajectoryAdmission::admit(const aurora_msgs::msg::Trajectory &message,
                                            double now,
                                            bool emergency_stop) const {
  AdmissionResult result;
  if (emergency_stop) {
    result.status = AdmissionStatus::EMERGENCY_STOP;
    result.detail = "trajectory rejected while emergency stop is active";
    return result;
  }
  if (!std::isfinite(now)) {
    result.detail = "execution time is not finite";
    return result;
  }
  if (message.validation_state != aurora_msgs::msg::Trajectory::VALIDATED) {
    result.status = AdmissionStatus::REJECTED_UNVALIDATED;
    result.detail = "trajectory validation_state is not VALIDATED";
    return result;
  }
  if (!message.safety_report.accepted ||
      message.safety_report.status != aurora_msgs::msg::SafetyReport::ACCEPTED) {
    result.status = AdmissionStatus::REJECTED_UNSAFE;
    result.detail = "trajectory safety report is not accepted or has a non-accepted status";
    return result;
  }
  if (!validRosTime(message.header.stamp) || message.header.frame_id.empty()) {
    result.detail = "trajectory header has an invalid timestamp or empty frame";
    return result;
  }
  if (message.header.frame_id != options_.expected_frame) {
    result.status = AdmissionStatus::FRAME_MISMATCH;
    result.detail = "trajectory frame does not match the flight adapter frame";
    return result;
  }
  if (message.segments.empty()) {
    result.detail = "trajectory has no segments";
    return result;
  }

  AdmittedTrajectory admitted;
  admitted.trajectory_id = message.trajectory_id;
  admitted.frame_id = message.header.frame_id;
  admitted.segments.reserve(message.segments.size());
  double previous_end = std::numeric_limits<double>::quiet_NaN();
  for (const auto &message_segment : message.segments) {
    if (!validRosTime(message_segment.start_stamp) ||
        !std::isfinite(message_segment.source_start_time) ||
        !std::isfinite(message_segment.duration) || message_segment.duration <= 0.0 ||
        !std::isfinite(message_segment.dt) || message_segment.dt <= 0.0 ||
        message_segment.degree != 3 || message_segment.control_points.size() < 4U ||
        message_segment.source_start_time < -options_.time_tolerance) {
      result.detail = "trajectory contains an invalid segment header";
      return result;
    }
    if (message_segment.knot_mode != aurora_msgs::msg::TrajectorySegment::CLAMPED &&
        message_segment.knot_mode != aurora_msgs::msg::TrajectorySegment::EGO_UNCLAMPED) {
      result.detail = "trajectory contains an unknown knot mode";
      return result;
    }
    const double start_stamp = timeToSeconds(message_segment.start_stamp);
    const double end_stamp = start_stamp + message_segment.duration;
    if (!std::isfinite(start_stamp) || !std::isfinite(end_stamp) ||
        (std::isfinite(previous_end) &&
         std::abs(start_stamp - previous_end) > options_.time_tolerance)) {
      result.detail = "trajectory segments are not contiguous in absolute time";
      return result;
    }

    aurora::math::UniformBspline::ControlPointMatrix control_points(
        3, static_cast<Eigen::Index>(message_segment.control_points.size()));
    for (std::size_t index = 0U; index < message_segment.control_points.size(); ++index) {
      if (!finitePoint(message_segment.control_points[index])) {
        result.detail = "trajectory contains a non-finite control point";
        return result;
      }
      control_points.col(static_cast<Eigen::Index>(index)) =
          toEigen(message_segment.control_points[index]);
    }
    const auto knot_mode = message_segment.knot_mode ==
                                   aurora_msgs::msg::TrajectorySegment::CLAMPED
                               ? aurora::math::UniformBsplineKnotMode::CLAMPED
                               : aurora::math::UniformBsplineKnotMode::EGO_UNCLAMPED;
    try {
      aurora::math::UniformBspline spline(control_points, message_segment.dt, knot_mode);
      const double spline_end = spline.duration();
      if (!std::isfinite(spline_end) ||
          message_segment.source_start_time > spline_end + options_.time_tolerance ||
          message_segment.source_start_time + message_segment.duration <
              -options_.time_tolerance ||
          message_segment.source_start_time + message_segment.duration >
              spline_end + options_.time_tolerance) {
        result.detail = "trajectory segment exceeds its spline time window";
        return result;
      }
      admitted.segments.push_back(
          {start_stamp, message_segment.source_start_time, message_segment.duration,
           std::move(spline)});
    } catch (const std::exception &error) {
      result.detail = std::string("trajectory spline is invalid: ") + error.what();
      return result;
    }
    previous_end = end_stamp;
  }

  if (!std::isfinite(admitted.endStamp()) || now > admitted.endStamp() + options_.time_tolerance) {
    result.status = AdmissionStatus::EXPIRED;
    result.detail = "trajectory has already expired";
    return result;
  }

  const auto append_setpoint = [&](double stamp, const AdmittedSegment &segment) -> bool {
    if (admitted.setpoints.size() >= options_.max_setpoints) {
      return false;
    }
    const double relative_time =
        std::clamp(stamp - segment.start_stamp, 0.0, segment.duration);
    const double spline_time = std::clamp(
        segment.source_start_time + relative_time, 0.0, segment.spline.duration());
    FlightSetpoint point;
    point.stamp = stamp;
    point.position = segment.spline.evaluate(spline_time, 0);
    point.velocity = segment.spline.evaluate(spline_time, 1);
    point.acceleration = segment.spline.evaluate(spline_time, 2);
    if (!finiteVector(point.position) || !finiteVector(point.velocity) ||
        !finiteVector(point.acceleration)) {
      return false;
    }
    if (!admitted.setpoints.empty() &&
        std::abs(admitted.setpoints.back().stamp - stamp) <= options_.time_tolerance) {
      admitted.setpoints.back() = point;
    } else {
      admitted.setpoints.push_back(point);
    }
    return true;
  };

  const double first_stamp = admitted.startStamp();
  for (const auto &segment : admitted.segments) {
    const double segment_end = segment.endStamp();
    const double sample_start = std::max(now, segment.start_stamp);
    if (sample_start > segment_end + options_.time_tolerance) {
      continue;
    }
    if (!append_setpoint(sample_start, segment)) {
      result.status = admitted.setpoints.size() >= options_.max_setpoints
                          ? AdmissionStatus::SETPOINT_LIMIT
                          : AdmissionStatus::REJECTED_INVALID;
      result.detail = result.status == AdmissionStatus::SETPOINT_LIMIT
                          ? "trajectory exceeds the setpoint limit"
                          : "trajectory evaluation produced a non-finite setpoint";
      return result;
    }
    double sample_stamp = sample_start + options_.setpoint_interval;
    while (sample_stamp < segment_end - options_.time_tolerance) {
      if (!append_setpoint(sample_stamp, segment)) {
        result.status = admitted.setpoints.size() >= options_.max_setpoints
                            ? AdmissionStatus::SETPOINT_LIMIT
                            : AdmissionStatus::REJECTED_INVALID;
        result.detail = result.status == AdmissionStatus::SETPOINT_LIMIT
                            ? "trajectory exceeds the setpoint limit"
                            : "trajectory evaluation produced a non-finite setpoint";
        return result;
      }
      sample_stamp += options_.setpoint_interval;
    }
    if (!append_setpoint(segment_end, segment)) {
      result.status = admitted.setpoints.size() >= options_.max_setpoints
                          ? AdmissionStatus::SETPOINT_LIMIT
                          : AdmissionStatus::REJECTED_INVALID;
      result.detail = result.status == AdmissionStatus::SETPOINT_LIMIT
                          ? "trajectory exceeds the setpoint limit"
                          : "trajectory evaluation produced a non-finite setpoint";
      return result;
    }
  }
  if (admitted.setpoints.empty() || admitted.setpoints.front().stamp < first_stamp -
                                             options_.time_tolerance) {
    result.status = AdmissionStatus::EXPIRED;
    result.detail = "trajectory has no executable setpoint at the current time";
    return result;
  }

  result.accepted = true;
  result.status = AdmissionStatus::ACCEPTED;
  result.detail = "trajectory admitted for flight-controller execution";
  result.trajectory = std::move(admitted);
  return result;
}

FeedbackResult TrajectoryAdmission::observeFeedback(
    const aurora_msgs::msg::TrajectoryExecutionStatus &feedback,
    std::uint64_t expected_trajectory_id,
    double now) const {
  FeedbackResult result;
  if (!std::isfinite(now) || !validRosTime(feedback.header.stamp)) {
    result.detail = "flight-controller feedback has an invalid time";
    return result;
  }
  const double stamp = timeToSeconds(feedback.header.stamp);
  if (stamp > now + options_.time_tolerance) {
    result.detail = "flight-controller feedback is from the future";
    return result;
  }
  if (feedback.trajectory_id != expected_trajectory_id) {
    result.action = FeedbackAction::REQUEST_REPLAN;
    result.detail = "flight-controller feedback refers to a different trajectory";
    return result;
  }
  if (!validFeedbackStatus(feedback.status)) {
    result.detail = "flight-controller feedback has an unknown status";
    return result;
  }

  result.valid = true;
  switch (feedback.status) {
    case aurora_msgs::msg::TrajectoryExecutionStatus::ACTIVE:
      result.action = feedback.accepted ? FeedbackAction::KEEP_EXECUTING
                                        : FeedbackAction::REQUEST_REPLAN;
      result.detail = feedback.accepted ? "flight controller is executing the trajectory"
                                        : "flight controller did not accept the active trajectory";
      break;
    case aurora_msgs::msg::TrajectoryExecutionStatus::COMPLETED:
      result.action = feedback.accepted ? FeedbackAction::COMPLETED
                                        : FeedbackAction::REQUEST_REPLAN;
      result.detail = feedback.accepted ? "flight controller completed the trajectory"
                                        : "completion feedback was not accepted";
      break;
    case aurora_msgs::msg::TrajectoryExecutionStatus::IDLE:
    case aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_UNVALIDATED:
    case aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_UNSAFE:
    case aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_INVALID:
      result.action = FeedbackAction::REQUEST_REPLAN;
      result.detail = feedback.detail.empty() ? "flight controller rejected the trajectory"
                                               : feedback.detail;
      break;
    case aurora_msgs::msg::TrajectoryExecutionStatus::EMERGENCY_STOP:
    case aurora_msgs::msg::TrajectoryExecutionStatus::TIME_ROLLBACK:
    case aurora_msgs::msg::TrajectoryExecutionStatus::TIME_GAP:
      result.action = FeedbackAction::EMERGENCY_STOP;
      result.detail = feedback.detail.empty() ? "flight controller reported a safety fault"
                                               : feedback.detail;
      break;
  }
  return result;
}

}  // namespace aurora::flight
