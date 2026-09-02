#include "aurora_sim/trajectory_executor.hpp"

#include "aurora_flight_adapter/trajectory_admission.hpp"
#include "aurora_msgs/msg/emergency_stop_state.hpp"
#include "aurora_msgs/msg/trajectory.hpp"
#include "aurora_msgs/msg/trajectory_execution_status.hpp"
#include "aurora_msgs/msg/vehicle_state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &point) {
  geometry_msgs::msg::Point result;
  result.x = point.x();
  result.y = point.y();
  result.z = point.z();
  return result;
}

geometry_msgs::msg::Vector3 toVector(const Eigen::Vector3d &value) {
  geometry_msgs::msg::Vector3 result;
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  return result;
}

std::uint8_t toMessageStatus(aurora::simulation::ExecutionStatus status) {
  using Status = aurora::simulation::ExecutionStatus;
  switch (status) {
    case Status::IDLE:
      return aurora_msgs::msg::TrajectoryExecutionStatus::IDLE;
    case Status::ACTIVE:
      return aurora_msgs::msg::TrajectoryExecutionStatus::ACTIVE;
    case Status::COMPLETED:
      return aurora_msgs::msg::TrajectoryExecutionStatus::COMPLETED;
    case Status::REJECTED_UNVALIDATED:
      return aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_UNVALIDATED;
    case Status::REJECTED_UNSAFE:
      return aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_UNSAFE;
    case Status::EMERGENCY_STOP:
      return aurora_msgs::msg::TrajectoryExecutionStatus::EMERGENCY_STOP;
    case Status::TIME_ROLLBACK:
      return aurora_msgs::msg::TrajectoryExecutionStatus::TIME_ROLLBACK;
    case Status::TIME_GAP:
      return aurora_msgs::msg::TrajectoryExecutionStatus::TIME_GAP;
    case Status::REJECTED_INVALID:
      return aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_INVALID;
  }
  return aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_INVALID;
}

aurora::simulation::ExecutionStatus toExecutionStatus(
    aurora::flight::AdmissionStatus status) {
  switch (status) {
    case aurora::flight::AdmissionStatus::ACCEPTED:
      return aurora::simulation::ExecutionStatus::ACTIVE;
    case aurora::flight::AdmissionStatus::EMERGENCY_STOP:
      return aurora::simulation::ExecutionStatus::EMERGENCY_STOP;
    case aurora::flight::AdmissionStatus::REJECTED_UNVALIDATED:
      return aurora::simulation::ExecutionStatus::REJECTED_UNVALIDATED;
    case aurora::flight::AdmissionStatus::REJECTED_UNSAFE:
      return aurora::simulation::ExecutionStatus::REJECTED_UNSAFE;
    case aurora::flight::AdmissionStatus::REJECTED_INVALID:
    case aurora::flight::AdmissionStatus::EXPIRED:
    case aurora::flight::AdmissionStatus::FRAME_MISMATCH:
    case aurora::flight::AdmissionStatus::SETPOINT_LIMIT:
      return aurora::simulation::ExecutionStatus::REJECTED_INVALID;
  }
  return aurora::simulation::ExecutionStatus::REJECTED_INVALID;
}

class AuroraSimulationNode final : public rclcpp::Node {
public:
  AuroraSimulationNode() : Node("aurora_sim_node") {
    const auto initial_x = declare_parameter<double>("simulation.initial_x", -4.0);
    const auto initial_y = declare_parameter<double>("simulation.initial_y", 0.0);
    const auto initial_z = declare_parameter<double>("simulation.initial_z", 1.0);
    aurora::simulation::ExecutorOptions options;
    options.max_velocity = declare_parameter<double>("simulation.max_velocity", 3.0);
    options.max_acceleration = declare_parameter<double>("simulation.max_acceleration", 6.0);
    options.position_gain = declare_parameter<double>("simulation.position_gain", 6.0);
    options.velocity_gain = declare_parameter<double>("simulation.velocity_gain", 4.0);
    options.max_integration_step = declare_parameter<double>(
        "simulation.max_integration_step", 0.01);
    options.max_update_gap = declare_parameter<double>("simulation.max_update_gap", 0.5);
    options.time_tolerance = declare_parameter<double>("simulation.time_tolerance", 1e-6);
    reject_trajectories_ = declare_parameter<bool>("simulation.reject_trajectories", false);
    publish_rate_hz_ = declare_parameter<double>("simulation.publish_rate_hz", 50.0);
    // Parameters must be declared after the Node base is constructed; the
    // resulting assignments are intentional runtime initialization.
    // cppcheck-suppress useInitializationList
    frame_id_ = declare_parameter<std::string>("simulation.frame_id", "map");
    // cppcheck-suppress useInitializationList
    trajectory_topic_ = declare_parameter<std::string>(
        "topics.trajectory", "/aurora/trajectory");
    // cppcheck-suppress useInitializationList
    vehicle_state_topic_ = declare_parameter<std::string>(
        "topics.vehicle_state", "/aurora/sim/vehicle_state");
    // cppcheck-suppress useInitializationList
    desired_pose_topic_ = declare_parameter<std::string>(
        "topics.desired_pose", "/aurora/sim/desired_pose");
    // cppcheck-suppress useInitializationList
    emergency_state_topic_ = declare_parameter<std::string>(
        "topics.emergency_state", "/aurora/emergency_stop_state");
    // cppcheck-suppress useInitializationList
    execution_status_topic_ = declare_parameter<std::string>(
        "topics.execution_status", "/aurora/sim/execution_status");

    aurora::flight::AdapterOptions adapter_options;
    adapter_options.expected_frame = frame_id_;
    adapter_options.time_tolerance = options.time_tolerance;
    adapter_options.setpoint_interval = declare_parameter<double>(
        "simulation.setpoint_interval", 0.05);
    const auto max_setpoints = declare_parameter<std::int64_t>(
        "simulation.max_setpoints", 2000);
    if (max_setpoints <= 0) {
      throw std::invalid_argument("simulation.max_setpoints must be positive");
    }
    adapter_options.max_setpoints = static_cast<std::size_t>(max_setpoints);
    flight_admission_ = std::make_unique<aurora::flight::TrajectoryAdmission>(
        adapter_options);

    aurora::simulation::FlightState initial_state;
    initial_state.position = {initial_x, initial_y, initial_z};
    // cppcheck-suppress useInitializationList
    executor_ = std::make_unique<aurora::simulation::TrajectoryExecutor>(
        options, initial_state);

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto emergency_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    trajectory_subscription_ = create_subscription<aurora_msgs::msg::Trajectory>(
        trajectory_topic_, reliable_qos,
        std::bind(&AuroraSimulationNode::onTrajectory, this, std::placeholders::_1));
    emergency_subscription_ = create_subscription<aurora_msgs::msg::EmergencyStopState>(
        emergency_state_topic_, emergency_qos,
        std::bind(&AuroraSimulationNode::onEmergencyState, this, std::placeholders::_1));
    vehicle_state_publisher_ = create_publisher<aurora_msgs::msg::VehicleState>(
        vehicle_state_topic_, reliable_qos);
    desired_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        desired_pose_topic_, reliable_qos);
    execution_status_publisher_ = create_publisher<
        aurora_msgs::msg::TrajectoryExecutionStatus>(execution_status_topic_, reliable_qos);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("simulation.publish_rate_hz must be positive");
    }
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&AuroraSimulationNode::onTimer, this));
  }

private:
  aurora::simulation::ExecutorTrajectory toCoreTrajectory(
      const aurora::flight::AdmittedTrajectory &admitted) const {
    aurora::simulation::ExecutorTrajectory result;
    result.trajectory_id = admitted.trajectory_id;
    result.validated = true;
    result.safety_accepted = true;
    result.segments.reserve(admitted.segments.size());
    for (const auto &segment : admitted.segments) {
      result.segments.push_back({segment.start_stamp, segment.source_start_time,
                                 segment.duration, segment.spline});
    }
    return result;
  }

  void publishStatus(const aurora::simulation::ExecutorOutput &output, bool accepted) {
    aurora_msgs::msg::TrajectoryExecutionStatus message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = output.trajectory_id;
    message.status = toMessageStatus(output.status);
    message.accepted = accepted;
    message.active = output.active;
    message.tracking_error = output.tracking_error;
    message.detail = output.detail;
    execution_status_publisher_->publish(std::move(message));
  }

  aurora::simulation::ExecutorOutput outputForStatus(
      aurora::simulation::ExecutionStatus status, std::uint64_t trajectory_id,
      const std::string &detail) const {
    aurora::simulation::ExecutorOutput output;
    output.status = status;
    output.trajectory_id = trajectory_id;
    output.detail = detail;
    output.state = executor_->state();
    output.desired = output.state;
    return output;
  }

  void onTrajectory(const aurora_msgs::msg::Trajectory::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reject_trajectories_) {
      publishStatus(outputForStatus(
                        aurora::simulation::ExecutionStatus::REJECTED_UNSAFE,
                        message->trajectory_id,
                        "simulation flight-controller rejection was injected"),
                    false);
      return;
    }
    const double now_stamp = now().seconds();
    try {
      const auto admission = flight_admission_->admit(
          *message, now_stamp, executor_->emergencyStopActive());
      if (!admission.accepted || !admission.trajectory.has_value()) {
        const auto output = outputForStatus(
            toExecutionStatus(admission.status), message->trajectory_id,
            admission.detail);
        publishStatus(output, false);
        RCLCPP_WARN(get_logger(), "rejected trajectory %lu at adapter boundary: %s",
                    message->trajectory_id, admission.detail.c_str());
        return;
      }
      const auto acceptance = executor_->accept(
          toCoreTrajectory(*admission.trajectory), now_stamp);
      const auto output = outputForStatus(acceptance.status, message->trajectory_id,
                                          acceptance.detail);
      publishStatus(output, acceptance.accepted);
      if (!acceptance.accepted) {
        RCLCPP_WARN(
            get_logger(), "rejected trajectory %lu: %s", message->trajectory_id,
            acceptance.detail.c_str());
      }
    } catch (const std::exception &error) {
      const auto output = outputForStatus(
          aurora::simulation::ExecutionStatus::REJECTED_INVALID,
          message->trajectory_id, error.what());
      publishStatus(output, false);
      RCLCPP_WARN(get_logger(), "invalid trajectory %lu: %s", message->trajectory_id,
                  error.what());
    }
  }

  void onEmergencyState(
      const aurora_msgs::msg::EmergencyStopState::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    executor_->setEmergencyStop(message->active);
    const auto output = outputForStatus(
        message->active ? aurora::simulation::ExecutionStatus::EMERGENCY_STOP
                        : aurora::simulation::ExecutionStatus::IDLE,
        0U, message->active ? "emergency stop received" : "emergency stop reset");
    publishStatus(output, false);
  }

  void onTimer() {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time current_time = now();
    const auto output = executor_->update(current_time.seconds());

    aurora_msgs::msg::VehicleState state_message;
    state_message.header.stamp = current_time;
    state_message.header.frame_id = frame_id_;
    state_message.position = toPoint(output.state.position);
    state_message.velocity = toVector(output.state.velocity);
    state_message.acceleration = toVector(output.state.acceleration);
    state_message.has_state_covariance = false;
    vehicle_state_publisher_->publish(std::move(state_message));

    geometry_msgs::msg::PoseStamped desired_message;
    desired_message.header.stamp = current_time;
    desired_message.header.frame_id = frame_id_;
    desired_message.pose.position = toPoint(output.desired.position);
    desired_message.pose.orientation.w = 1.0;
    desired_pose_publisher_->publish(std::move(desired_message));

    if (output.status != last_published_status_ || output.active != last_published_active_ ||
        output.trajectory_id != last_published_trajectory_id_) {
      publishStatus(output, output.status == aurora::simulation::ExecutionStatus::ACTIVE ||
                                output.status == aurora::simulation::ExecutionStatus::COMPLETED);
      last_published_status_ = output.status;
      last_published_active_ = output.active;
      last_published_trajectory_id_ = output.trajectory_id;
    }
  }

  std::unique_ptr<aurora::simulation::TrajectoryExecutor> executor_;
  std::unique_ptr<aurora::flight::TrajectoryAdmission> flight_admission_;
  std::mutex mutex_;
  bool reject_trajectories_{false};
  double publish_rate_hz_{50.0};
  std::string frame_id_;
  std::string trajectory_topic_;
  std::string vehicle_state_topic_;
  std::string desired_pose_topic_;
  std::string emergency_state_topic_;
  std::string execution_status_topic_;
  aurora::simulation::ExecutionStatus last_published_status_{
      aurora::simulation::ExecutionStatus::IDLE};
  bool last_published_active_{false};
  std::uint64_t last_published_trajectory_id_{0U};
  rclcpp::Subscription<aurora_msgs::msg::Trajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<aurora_msgs::msg::EmergencyStopState>::SharedPtr emergency_subscription_;
  rclcpp::Publisher<aurora_msgs::msg::VehicleState>::SharedPtr vehicle_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_publisher_;
  rclcpp::Publisher<aurora_msgs::msg::TrajectoryExecutionStatus>::SharedPtr
      execution_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AuroraSimulationNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("aurora_sim_node"), "failed to start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
