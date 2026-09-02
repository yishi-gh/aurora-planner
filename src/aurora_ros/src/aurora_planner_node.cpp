#include "aurora_planner_core/static_local_planner.hpp"
#include "aurora_planner_core/static_safety_gate.hpp"
#include "aurora_map/voxel_map.hpp"
#include "aurora_prediction/kinematic_predictor.hpp"
#include "aurora_flight_adapter/trajectory_admission.hpp"
#include "aurora_ros/dynamic_obstacle_adapter.hpp"
#include "aurora_ros/depth_image_adapter.hpp"
#include "aurora_ros/unassociated_obstacle_adapter.hpp"
#include "aurora_risk/dynamic_risk_evaluator.hpp"
#include "aurora_risk/risk_cost_field.hpp"
#include "aurora_msgs/msg/dynamic_obstacle_track_array.hpp"
#include "aurora_msgs/msg/emergency_stop_state.hpp"
#include "aurora_msgs/msg/planner_status.hpp"
#include "aurora_msgs/msg/planning_request.hpp"
#include "aurora_msgs/msg/planning_result.hpp"
#include "aurora_msgs/msg/risk_report.hpp"
#include "aurora_msgs/msg/safety_report.hpp"
#include "aurora_msgs/msg/trajectory.hpp"
#include "aurora_msgs/msg/unassociated_obstacle_detection_array.hpp"
#include "aurora_msgs/srv/set_emergency_stop.hpp"
#include "aurora_tracking/obstacle_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using aurora::map::VoxelMap;
using aurora::planner::FsmDecision;
using aurora::planner::GlobalReference;
using aurora::planner::LocalHorizonMode;
using aurora::planner::LocalPlannerOptions;
using aurora::planner::PlannedTrajectory;
using aurora::planner::PlanningRequest;
using aurora::planner::PlanningResult;
using aurora::planner::PlanningStatus;
using aurora::planner::ReplanObservation;
using aurora::planner::StaticLocalPlanner;
using aurora::planner::StaticReplanFsm;
using aurora::planner::StaticSafetyGate;
using aurora::planner::StaticSafetyGateOptions;
using aurora::planner::StaticSafetyGateResult;
using aurora::prediction::KinematicPredictor;
using aurora::prediction::KinematicPredictorOptions;
using aurora::prediction::PredictionResult;
using aurora::prediction::TrackState;
using aurora::risk::DynamicRiskEvaluation;
using aurora::risk::DynamicRiskEvaluator;
using aurora::risk::DynamicRiskEvaluatorOptions;
using aurora::risk::DynamicRiskCostField;
using aurora::risk::DynamicRiskCostFieldOptions;
using aurora::risk::DynamicRiskInput;
using aurora::risk::MapQualitySample;
using aurora::risk::MapRiskState;
using aurora::risk::RiskContext;
using aurora::risk::RiskStatus;
using aurora::risk::TrajectorySample;
using aurora::trajectory::RiskCostEvaluation;
using aurora::trajectory::RiskCostFunction;
using aurora::ros::DynamicObstacleAdapter;
using aurora::ros::DynamicTrackSnapshot;
using aurora::ros::DepthImageAdapter;
using aurora::ros::DepthImageAdapterOptions;
using aurora::ros::DepthImagePointCloud;
using aurora::ros::UnassociatedDetectionBatch;
using aurora::ros::UnassociatedObstacleAdapter;
using aurora::tracking::LifecycleState;
using aurora::tracking::ObstacleTracker;
using aurora::tracking::ObstacleTrackerOptions;
using aurora::tracking::TrackingResult;

enum class DynamicInputMode {
  EXTERNAL_TRACKS,
  INTERNAL_DETECTIONS,
};

enum class DynamicInputSource {
  EXTERNAL_TRACKS,
  INTERNAL_DETECTIONS,
};

double timeToSeconds(const builtin_interfaces::msg::Time &time) {
  return static_cast<double>(time.sec) + 1e-9 * static_cast<double>(time.nanosec);
}

builtin_interfaces::msg::Time secondsToTime(double seconds) {
  builtin_interfaces::msg::Time result;
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return result;
  }
  const double whole_seconds = std::floor(seconds);
  std::int64_t sec = static_cast<std::int64_t>(whole_seconds);
  std::int64_t nanosec = static_cast<std::int64_t>(
      std::llround((seconds - whole_seconds) * 1e9));
  if (nanosec >= 1000000000LL) {
    ++sec;
    nanosec -= 1000000000LL;
  }
  if (sec > std::numeric_limits<std::int32_t>::max()) {
    return builtin_interfaces::msg::Time{};
  }
  result.sec = static_cast<std::int32_t>(sec);
  result.nanosec = static_cast<std::uint32_t>(nanosec);
  return result;
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &point) {
  return {point.x, point.y, point.z};
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 &vector) {
  return {vector.x, vector.y, vector.z};
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &value) {
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

bool hasPointField(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                     [&name](const sensor_msgs::msg::PointField &field) {
                       return field.name == name;
                     });
}

bool transformPoint(const geometry_msgs::msg::TransformStamped &transform,
                    const Eigen::Vector3d &point, Eigen::Vector3d *transformed) {
  const Eigen::Vector3d translation(
      transform.transform.translation.x, transform.transform.translation.y,
      transform.transform.translation.z);
  Eigen::Quaterniond rotation(
      transform.transform.rotation.w, transform.transform.rotation.x,
      transform.transform.rotation.y, transform.transform.rotation.z);
  if (!translation.allFinite() || !point.allFinite() || !rotation.coeffs().allFinite() ||
      !std::isfinite(rotation.norm()) || rotation.norm() <= 1e-12) {
    return false;
  }
  rotation.normalize();
  *transformed = rotation * point + translation;
  return transformed->allFinite();
}

class AuroraPlannerNode final : public rclcpp::Node {
public:
  explicit AuroraPlannerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("aurora_planner_node", options),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    configure();
    createInterfaces();
    publishEmergencyState(false, false, "planner initialized",
                          aurora_msgs::msg::EmergencyStopState::UNSPECIFIED);
    planner_thread_ = std::thread(&AuroraPlannerNode::planningLoop, this);
  }

  ~AuroraPlannerNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (planner_thread_.joinable()) {
      planner_thread_.join();
    }
  }

private:
  void configure() {
    map_frame_ = declare_parameter<std::string>("map.frame", "map");
    pointcloud_topic_ = declare_parameter<std::string>("topics.pointcloud", "/points");
    depth_image_topic_ =
        declare_parameter<std::string>("topics.depth_image", "/camera/depth/image_rect_raw");
    camera_info_topic_ =
        declare_parameter<std::string>("topics.camera_info", "/camera/depth/camera_info");
    planning_request_topic_ =
        declare_parameter<std::string>("topics.planning_request", "/aurora/planning_request");
    trajectory_topic_ =
        declare_parameter<std::string>("topics.trajectory", "/aurora/trajectory");
    planning_result_topic_ =
        declare_parameter<std::string>("topics.planning_result", "/aurora/planning_result");
    planner_status_topic_ =
        declare_parameter<std::string>("topics.planner_status", "/aurora/planner_status");
    emergency_state_topic_ = declare_parameter<std::string>(
        "topics.emergency_state", "/aurora/emergency_stop_state");
    dynamic_obstacle_topic_ = declare_parameter<std::string>(
        "topics.dynamic_obstacle_tracks", "/aurora/dynamic_obstacle_tracks");
    dynamic_detection_topic_ = declare_parameter<std::string>(
        "topics.dynamic_obstacle_detections", "/aurora/dynamic_obstacle_detections");
    vehicle_state_topic_ = declare_parameter<std::string>(
        "topics.vehicle_state", "/aurora/sim/vehicle_state");
    execution_status_topic_ = declare_parameter<std::string>(
        "topics.execution_status", "/aurora/sim/execution_status");
    const auto dynamic_input_mode =
        declare_parameter<std::string>("dynamic_input_mode", "external_tracks");
    if (dynamic_input_mode == "external_tracks") {
      dynamic_input_mode_ = DynamicInputMode::EXTERNAL_TRACKS;
    } else if (dynamic_input_mode == "internal_detections") {
      dynamic_input_mode_ = DynamicInputMode::INTERNAL_DETECTIONS;
    } else {
      throw std::invalid_argument(
          "dynamic_input_mode must be external_tracks or internal_detections");
    }
    emergency_service_ = declare_parameter<std::string>(
        "services.emergency_stop", "/aurora/set_emergency_stop");

    vehicle_radius_ = declare_parameter<double>("vehicle.radius", 0.65);

    aurora::map::VoxelMapConfig map_config;
    map_config.origin = Eigen::Vector3d(
        declare_parameter<double>("map.origin_x", -20.0),
        declare_parameter<double>("map.origin_y", -20.0),
        declare_parameter<double>("map.origin_z", -2.0));
    const auto dimension_x = declare_parameter<std::int64_t>("map.dimensions_x", 80);
    const auto dimension_y = declare_parameter<std::int64_t>("map.dimensions_y", 80);
    const auto dimension_z = declare_parameter<std::int64_t>("map.dimensions_z", 40);
    if (dimension_x <= 0 || dimension_x > std::numeric_limits<int>::max() ||
        dimension_y <= 0 || dimension_y > std::numeric_limits<int>::max() ||
        dimension_z <= 0 || dimension_z > std::numeric_limits<int>::max()) {
      throw std::invalid_argument("map dimensions exceed the supported integer range");
    }
    map_config.dimensions = Eigen::Vector3i(static_cast<int>(dimension_x),
                                            static_cast<int>(dimension_y),
                                            static_cast<int>(dimension_z));
    map_config.resolution = declare_parameter<double>("map.resolution", 0.5);
    map_config.occupancy_threshold =
        declare_parameter<double>("map.occupancy_threshold", 0.8);
    map_config.p_hit = declare_parameter<double>("map.p_hit", 0.65);
    map_config.p_miss = declare_parameter<double>("map.p_miss", 0.35);
    map_config.p_min = declare_parameter<double>("map.p_min", 0.12);
    map_config.p_max = declare_parameter<double>("map.p_max", 0.90);
    map_inflation_radius_ =
        declare_parameter<double>("map.inflation_radius", vehicle_radius_);
    map_ = std::make_unique<VoxelMap>(map_config);
    map_->inflate(map_inflation_radius_);

    LocalPlannerOptions planner_options;
    const auto horizon_mode =
        declare_parameter<std::string>("planning.horizon_mode", "distance");
    if (horizon_mode == "distance") {
      planner_options.horizon_mode = LocalHorizonMode::DISTANCE;
    } else if (horizon_mode == "time") {
      planner_options.horizon_mode = LocalHorizonMode::TIME;
    } else {
      throw std::invalid_argument("planning.horizon_mode must be distance or time");
    }
    planner_options.local_horizon = declare_parameter<double>("planning.local_horizon", 6.0);
    planner_options.goal_tolerance = declare_parameter<double>("planning.goal_tolerance", 0.25);
    planner_options.max_projection_distance = declare_parameter<double>(
        "planning.max_projection_distance", std::numeric_limits<double>::infinity());
    planner_options.resampling.spacing =
        declare_parameter<double>("planning.resampling_spacing", 0.5);
    const auto minimum_points =
        declare_parameter<std::int64_t>("planning.resampling_minimum_points", 9);
    if (minimum_points < 0) {
      throw std::invalid_argument("planning.resampling_minimum_points is out of range");
    }
    planner_options.resampling.minimum_points = static_cast<std::size_t>(minimum_points);
    planner_options.optimizer.interval =
        declare_parameter<double>("planning.optimizer_interval", 0.25);
    planner_options.optimizer.clearance =
        declare_parameter<double>("planning.optimizer_clearance", map_inflation_radius_);
    planner_options.optimizer.max_iterations =
        declare_parameter<int>("planning.optimizer_max_iterations", 180);
    planner_options.optimizer.max_compute_time_sec = declare_parameter<double>(
        "planning.optimizer_max_compute_time_sec", 0.0);
    planner_options.optimizer.samples_per_span = static_cast<std::size_t>(declare_parameter<int>(
        "planning.optimizer_samples_per_span", 8));
    planner_options.optimizer.lambda_risk = declare_parameter<double>(
        "planning.optimizer_lambda_risk", 0.0);
    const auto max_optimizer_risk_evaluations = declare_parameter<std::int64_t>(
        "planning.optimizer_max_risk_evaluations", 10000);
    if (max_optimizer_risk_evaluations <= 0) {
      throw std::invalid_argument(
          "planning.optimizer_max_risk_evaluations must be positive");
    }
    planner_options.optimizer.max_risk_evaluations =
        static_cast<std::size_t>(max_optimizer_risk_evaluations);
    planner_options.validation.samples_per_span = static_cast<std::size_t>(declare_parameter<int>(
        "planning.validation_samples_per_span", 16));
    planner_options.validation.max_velocity =
        declare_parameter<double>("vehicle.max_velocity", 3.0);
    planner_options.validation.max_acceleration =
        declare_parameter<double>("vehicle.max_acceleration", 6.0);
    planner_options.optimizer.max_velocity = planner_options.validation.max_velocity;
    planner_options.optimizer.max_acceleration = planner_options.validation.max_acceleration;
    planner_options.validation.reject_unknown =
        declare_parameter<bool>("map.reject_unknown", true);
    planner_options.stitch_prefix_duration =
        declare_parameter<double>("planning.stitch_prefix_duration", 0.5);
    planner_options.stitch_position_tolerance =
        declare_parameter<double>("planning.stitch_position_tolerance", 0.5);
    planner_options.stitch_velocity_tolerance =
        declare_parameter<double>("planning.stitch_velocity_tolerance", 1.0);
    planner_options.stitch_acceleration_tolerance =
        declare_parameter<double>("planning.stitch_acceleration_tolerance", 2.0);
    planner_ = std::make_unique<StaticLocalPlanner>(planner_options);

    StaticSafetyGateOptions gate_options;
    gate_options.validation = planner_options.validation;
    gate_options.position_tolerance = planner_options.stitch_position_tolerance;
    gate_options.velocity_tolerance = planner_options.stitch_velocity_tolerance;
    gate_options.acceleration_tolerance = planner_options.stitch_acceleration_tolerance;
    safety_gate_ = std::make_unique<StaticSafetyGate>(gate_options);

    pointcloud_max_points_ = declare_parameter<std::int64_t>("pointcloud.max_points", 100000);
    pointcloud_max_range_ = declare_parameter<double>("pointcloud.max_range", 30.0);
    pointcloud_confidence_ = declare_parameter<double>("pointcloud.confidence", 1.0);
    depth_image_enabled_ = declare_parameter<bool>("depth.enabled", false);
    const auto depth_max_points =
        declare_parameter<std::int64_t>("depth.max_points", 100000);
    const auto depth_pixel_stride =
        declare_parameter<std::int64_t>("depth.pixel_stride", 1);
    depth_min_range_ = declare_parameter<double>("depth.min_range", 0.1);
    depth_max_range_ = declare_parameter<double>("depth.max_range", 30.0);
    depth_confidence_ = declare_parameter<double>("depth.confidence", 1.0);
    const double depth_camera_info_time_tolerance = declare_parameter<double>(
        "depth.camera_info_time_tolerance", 0.5);
    if (depth_max_points <= 0 || depth_pixel_stride <= 0) {
      throw std::invalid_argument("depth.max_points and depth.pixel_stride must be positive");
    }
    if (!std::isfinite(depth_confidence_) || depth_confidence_ < 0.0 ||
        depth_confidence_ > 1.0) {
      throw std::invalid_argument("depth.confidence must be in [0, 1]");
    }
    depth_adapter_ = std::make_unique<DepthImageAdapter>(DepthImageAdapterOptions{
        static_cast<std::size_t>(depth_max_points),
        static_cast<std::size_t>(depth_pixel_stride),
        depth_min_range_,
        depth_max_range_,
        depth_camera_info_time_tolerance});
    tf_timeout_sec_ = declare_parameter<double>("tf.timeout_sec", 0.05);
    map_freshness_required_ = declare_parameter<bool>("map.require_fresh_observation", true);
    map_max_observation_age_ = declare_parameter<double>("map.max_observation_age", 1.0);
    if (!std::isfinite(map_max_observation_age_) || map_max_observation_age_ < 0.0) {
      throw std::invalid_argument(
          "map.max_observation_age must be finite and non-negative");
    }

    KinematicPredictorOptions prediction_options;
    prediction_options.sample_interval =
        declare_parameter<double>("prediction.sample_interval", 0.1);
    prediction_options.max_horizon = declare_parameter<double>("prediction.max_horizon", 15.0);
    const auto max_prediction_samples =
        declare_parameter<std::int64_t>("prediction.max_samples", 10000);
    if (max_prediction_samples <= 0) {
      throw std::invalid_argument("prediction.max_samples must be positive");
    }
    prediction_options.max_samples = static_cast<std::size_t>(max_prediction_samples);
    prediction_options.process_noise_acceleration = declare_parameter<double>(
        "prediction.process_noise_acceleration", 1.0);
    prediction_options.process_noise_jerk =
        declare_parameter<double>("prediction.process_noise_jerk", 1.0);
    prediction_options.default_position_variance = declare_parameter<double>(
        "prediction.default_position_variance", 0.25);
    prediction_options.default_velocity_variance = declare_parameter<double>(
        "prediction.default_velocity_variance", 1.0);
    prediction_options.default_acceleration_variance = declare_parameter<double>(
        "prediction.default_acceleration_variance", 4.0);
    prediction_options.covariance_tolerance = declare_parameter<double>(
        "prediction.covariance_tolerance", 1e-9);
    predictor_ = std::make_unique<KinematicPredictor>(prediction_options);

    DynamicRiskEvaluatorOptions risk_options;
    risk_options.vehicle_radius = vehicle_radius_;
    risk_options.sigma_multiplier = declare_parameter<double>("risk.sigma_multiplier", 3.0);
    risk_options.time_tolerance = declare_parameter<double>("risk.time_tolerance", 1e-6);
    risk_options.max_prediction_age =
        declare_parameter<double>("risk.max_prediction_age", 0.5);
    risk_options.warning_clearance = declare_parameter<double>("risk.warning_clearance", 0.5);
    const auto max_risk_samples = declare_parameter<std::int64_t>("risk.max_samples", 100000);
    const auto max_risk_obstacles = declare_parameter<std::int64_t>("risk.max_obstacles", 1000);
    if (max_risk_samples <= 0 || max_risk_obstacles <= 0) {
      throw std::invalid_argument("risk sample and obstacle limits must be positive");
    }
    risk_options.max_samples = static_cast<std::size_t>(max_risk_samples);
    risk_options.max_obstacles = static_cast<std::size_t>(max_risk_obstacles);
    risk_options.require_dynamic_information =
        declare_parameter<bool>("risk.require_dynamic_information", true);
    risk_options.require_map_quality = declare_parameter<bool>("risk.require_map_quality", false);
    risk_options.allow_unknown_space = declare_parameter<bool>("risk.allow_unknown_space", false);
    risk_options.map_free_probability = declare_parameter<double>(
        "risk.map_free_probability", 0.2);
    risk_options.map_occupancy_threshold = declare_parameter<double>(
        "risk.map_occupancy_threshold", 0.8);
    risk_options.map_max_observation_age = declare_parameter<double>(
        "risk.map_max_observation_age", 1.0);
    risk_options.map_occupancy_weight = declare_parameter<double>(
        "risk.map_occupancy_weight", 0.5);
    risk_options.map_age_weight = declare_parameter<double>("risk.map_age_weight", 0.25);
    risk_options.map_confidence_weight = declare_parameter<double>(
        "risk.map_confidence_weight", 0.25);
    risk_options.map_risk_limit = declare_parameter<double>("risk.map_risk_limit", 1.0);
    risk_soft_cost_enabled_ = declare_parameter<bool>("risk.enable_soft_cost", true);
    soft_risk_options_.vehicle_radius = vehicle_radius_;
    soft_risk_options_.sigma_multiplier = risk_options.sigma_multiplier;
    soft_risk_options_.warning_clearance = risk_options.warning_clearance;
    soft_risk_options_.max_obstacles = risk_options.max_obstacles;
    risk_map_quality_enabled_ = declare_parameter<bool>(
        "risk.enable_map_quality", risk_options.require_map_quality);
    risk_delay_.sensing_delay = declare_parameter<double>("risk.sensing_delay", 0.0);
    risk_delay_.tracking_delay = declare_parameter<double>("risk.tracking_delay", 0.0);
    risk_delay_.planning_delay = declare_parameter<double>("risk.planning_delay", 0.0);
    risk_delay_.execution_delay = declare_parameter<double>("risk.execution_delay", 0.0);
    risk_delay_.safety_margin = declare_parameter<double>("risk.safety_margin", 0.0);
    if (!std::isfinite(risk_delay_.sensing_delay) || risk_delay_.sensing_delay < 0.0 ||
        !std::isfinite(risk_delay_.tracking_delay) || risk_delay_.tracking_delay < 0.0 ||
        !std::isfinite(risk_delay_.planning_delay) || risk_delay_.planning_delay < 0.0 ||
        !std::isfinite(risk_delay_.execution_delay) || risk_delay_.execution_delay < 0.0 ||
        !std::isfinite(risk_delay_.safety_margin) || risk_delay_.safety_margin < 0.0 ||
        !std::isfinite(risk_delay_.total())) {
      throw std::invalid_argument(
          "risk delay components must be finite and non-negative");
    }
    execution_position_variance_ = declare_parameter<double>(
        "risk.execution_position_variance", 0.0);
    if (!std::isfinite(execution_position_variance_) || execution_position_variance_ < 0.0) {
      throw std::invalid_argument(
          "risk.execution_position_variance must be finite and non-negative");
    }
    risk_sample_interval_ = declare_parameter<double>("risk.sample_interval", 0.1);
    if (!std::isfinite(risk_sample_interval_) || risk_sample_interval_ <= 0.0) {
      throw std::invalid_argument("risk.sample_interval must be positive and finite");
    }
    stale_hold_duration_ =
        declare_parameter<double>("risk.stale_hold_duration", 0.5);
    information_watchdog_rate_hz_ = declare_parameter<double>(
        "risk.information_watchdog_rate_hz", 10.0);
    if (!std::isfinite(stale_hold_duration_) || stale_hold_duration_ < 0.0 ||
        !std::isfinite(information_watchdog_rate_hz_) ||
        information_watchdog_rate_hz_ <= 0.0) {
      throw std::invalid_argument(
          "risk.stale_hold_duration must be non-negative and watchdog rate must be positive");
    }
    risk_evaluator_ = std::make_unique<DynamicRiskEvaluator>(risk_options);
    dynamic_obstacle_adapter_ =
        std::make_unique<DynamicObstacleAdapter>(map_frame_, risk_options.time_tolerance);

    aurora::flight::AdapterOptions flight_options;
    flight_options.expected_frame = map_frame_;
    flight_options.time_tolerance = risk_options.time_tolerance;
    flight_options.setpoint_interval =
        declare_parameter<double>("execution.setpoint_interval", 0.05);
    const auto max_execution_setpoints = declare_parameter<std::int64_t>(
        "execution.max_setpoints", 2000);
    if (max_execution_setpoints <= 0) {
      throw std::invalid_argument("execution.max_setpoints must be positive");
    }
    flight_options.max_setpoints = static_cast<std::size_t>(max_execution_setpoints);
    flight_admission_ = std::make_unique<aurora::flight::TrajectoryAdmission>(flight_options);

    ObstacleTrackerOptions tracking_options;
    tracking_options.mahalanobis_gate = declare_parameter<double>(
        "tracking.mahalanobis_gate", tracking_options.mahalanobis_gate);
    tracking_options.euclidean_gate = declare_parameter<double>(
        "tracking.euclidean_gate", tracking_options.euclidean_gate);
    tracking_options.covariance_tolerance = declare_parameter<double>(
        "tracking.covariance_tolerance", tracking_options.covariance_tolerance);
    tracking_options.minimum_measurement_variance = declare_parameter<double>(
        "tracking.minimum_measurement_variance", tracking_options.minimum_measurement_variance);
    tracking_options.default_position_variance = declare_parameter<double>(
        "tracking.default_position_variance", tracking_options.default_position_variance);
    tracking_options.default_velocity_variance = declare_parameter<double>(
        "tracking.default_velocity_variance", tracking_options.default_velocity_variance);
    tracking_options.process_noise_acceleration = declare_parameter<double>(
        "tracking.process_noise_acceleration", tracking_options.process_noise_acceleration);
    const auto confirmation_match_count = declare_parameter<std::int64_t>(
        "tracking.confirmation_match_count",
        static_cast<std::int64_t>(tracking_options.confirmation_match_count));
    if (confirmation_match_count < 2) {
      throw std::invalid_argument("tracking.confirmation_match_count must be at least 2");
    }
    tracking_options.confirmation_match_count =
        static_cast<std::size_t>(confirmation_match_count);
    tracking_options.lost_after = declare_parameter<double>(
        "tracking.lost_after", tracking_options.lost_after);
    tracking_options.deleted_after = declare_parameter<double>(
        "tracking.deleted_after", tracking_options.deleted_after);
    const auto first_track_id = declare_parameter<std::int64_t>(
        "tracking.first_track_id", static_cast<std::int64_t>(tracking_options.first_track_id));
    if (first_track_id <= 0) {
      throw std::invalid_argument("tracking.first_track_id must be positive");
    }
    tracking_options.first_track_id = static_cast<std::uint64_t>(first_track_id);
    tracking_options.default_shape.type = aurora::prediction::ShapeType::SPHERE;
    tracking_options.default_shape.radius = declare_parameter<double>(
        "tracking.default_shape_radius", 0.5);
    tracker_ = std::make_unique<ObstacleTracker>(tracking_options);
    unassociated_obstacle_adapter_ =
        std::make_unique<UnassociatedObstacleAdapter>(map_frame_, risk_options.time_tolerance);
  }

  void createInterfaces() {
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto emergency_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    planning_request_subscription_ = create_subscription<aurora_msgs::msg::PlanningRequest>(
        planning_request_topic_, reliable_qos,
        std::bind(&AuroraPlannerNode::onPlanningRequest, this, std::placeholders::_1));
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&AuroraPlannerNode::onPointCloud, this, std::placeholders::_1));
    if (depth_image_enabled_) {
      camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_topic_, rclcpp::SensorDataQoS(),
          std::bind(&AuroraPlannerNode::onCameraInfo, this, std::placeholders::_1));
      depth_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          depth_image_topic_, rclcpp::SensorDataQoS(),
          std::bind(&AuroraPlannerNode::onDepthImage, this, std::placeholders::_1));
    }
    dynamic_obstacle_subscription_ =
        create_subscription<aurora_msgs::msg::DynamicObstacleTrackArray>(
            dynamic_obstacle_topic_, reliable_qos,
            std::bind(&AuroraPlannerNode::onDynamicObstacles, this, std::placeholders::_1));
    dynamic_detection_subscription_ =
        create_subscription<aurora_msgs::msg::UnassociatedObstacleDetectionArray>(
            dynamic_detection_topic_, rclcpp::SensorDataQoS(),
            std::bind(&AuroraPlannerNode::onUnassociatedObstacles, this,
                      std::placeholders::_1));
    execution_vehicle_state_subscription_ = create_subscription<aurora_msgs::msg::VehicleState>(
        vehicle_state_topic_, reliable_qos,
        std::bind(&AuroraPlannerNode::onExecutionVehicleState, this,
                  std::placeholders::_1));
    execution_status_subscription_ =
        create_subscription<aurora_msgs::msg::TrajectoryExecutionStatus>(
            execution_status_topic_, status_qos,
            std::bind(&AuroraPlannerNode::onExecutionStatus, this,
                      std::placeholders::_1));
    trajectory_publisher_ =
        create_publisher<aurora_msgs::msg::Trajectory>(trajectory_topic_, reliable_qos);
    planning_result_publisher_ = create_publisher<aurora_msgs::msg::PlanningResult>(
        planning_result_topic_, reliable_qos);
    planner_status_publisher_ =
        create_publisher<aurora_msgs::msg::PlannerStatus>(planner_status_topic_, status_qos);
    emergency_state_publisher_ = create_publisher<aurora_msgs::msg::EmergencyStopState>(
        emergency_state_topic_, emergency_qos);
    emergency_service_server_ = create_service<aurora_msgs::srv::SetEmergencyStop>(
        emergency_service_,
        std::bind(&AuroraPlannerNode::onEmergencyStop, this, std::placeholders::_1,
                  std::placeholders::_2));
    const auto watchdog_period = std::chrono::milliseconds(std::max<std::int64_t>(
        1, static_cast<std::int64_t>(std::llround(1000.0 / information_watchdog_rate_hz_))));
    information_watchdog_timer_ = create_wall_timer(
        watchdog_period, std::bind(&AuroraPlannerNode::onInformationWatchdog, this));
  }

  static PlanningRequest toCoreRequest(const aurora_msgs::msg::PlanningRequest &message,
                                       const std::string &map_frame) {
    if ((!message.header.frame_id.empty() && message.header.frame_id != map_frame) ||
        (!message.vehicle_state.header.frame_id.empty() &&
         message.vehicle_state.header.frame_id != map_frame)) {
      throw std::invalid_argument("planning request state/reference must use the map frame");
    }

    PlanningRequest request;
    request.request_id = message.request_id;
    request.planning_stamp = timeToSeconds(message.header.stamp);
    request.vehicle_state.stamp = timeToSeconds(message.vehicle_state.header.stamp);
    request.vehicle_state.position = toEigen(message.vehicle_state.position);
    request.vehicle_state.velocity = toEigen(message.vehicle_state.velocity);
    request.vehicle_state.acceleration = toEigen(message.vehicle_state.acceleration);
    request.vehicle_state.has_position_covariance = message.vehicle_state.has_state_covariance;
    if (request.vehicle_state.has_position_covariance) {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          request.vehicle_state.position_covariance(row, column) =
              message.vehicle_state.state_covariance[
                  static_cast<std::size_t>(row * 9 + column)];
        }
      }
    }
    request.global_reference.points.reserve(message.global_reference.size());
    for (const auto &point_message : message.global_reference) {
      aurora::planner::ReferencePoint point;
      point.time_from_start = point_message.time_from_start;
      point.has_time = point_message.has_time;
      point.position = toEigen(point_message.position);
      point.velocity = toEigen(point_message.velocity);
      point.acceleration = toEigen(point_message.acceleration);
      request.global_reference.points.push_back(point);
    }
    return request;
  }

  bool dynamicSnapshotUsableAt(const DynamicTrackSnapshot &snapshot,
                               double evaluation_stamp) const {
    const auto &options = risk_evaluator_->options();
    if (!snapshot.has_snapshot || !snapshot.valid_header || snapshot.occlusion_active ||
        snapshot.information_incomplete || snapshot.source_conflict ||
        snapshot.invalid_track_count > 0U || !std::isfinite(snapshot.stamp) ||
        !std::isfinite(evaluation_stamp)) {
      return false;
    }
    if (evaluation_stamp < snapshot.stamp - options.time_tolerance) {
      return false;
    }
    return evaluation_stamp - snapshot.stamp <=
           options.max_prediction_age + options.time_tolerance;
  }

  bool mapObservationUsableAtLocked(double evaluation_stamp) const {
    if (!map_freshness_required_) {
      return true;
    }
    if (!map_observation_available_ || !std::isfinite(last_map_observation_stamp_) ||
        !std::isfinite(evaluation_stamp)) {
      return false;
    }
    const double tolerance = risk_evaluator_->options().time_tolerance;
    if (evaluation_stamp < last_map_observation_stamp_ - tolerance) {
      return false;
    }
    return evaluation_stamp - last_map_observation_stamp_ <=
           map_max_observation_age_ + tolerance;
  }

  bool markMapInformationStaleLocked(const std::string &detail, double stamp) {
    if (map_information_stale_.load()) {
      return false;
    }
    map_information_stale_.store(true);
    map_stale_since_stamp_ = std::isfinite(stamp) ? stamp : currentTimeSeconds();
    map_recovery_pending_ = true;
    pending_request_.reset();
    dynamic_obstacle_update_pending_ = false;
    map_stale_detail_ = detail;
    return true;
  }

  bool sourceStampFreshAt(double source_stamp, double reference_stamp) const {
    const auto &options = risk_evaluator_->options();
    return std::isfinite(source_stamp) && std::isfinite(reference_stamp) &&
           reference_stamp >= source_stamp - options.time_tolerance &&
           reference_stamp - source_stamp <=
               options.max_prediction_age + options.time_tolerance;
  }

  bool dynamicInputSourcesConflictAt(double reference_stamp) const {
    return has_external_input_ && has_internal_input_ &&
           sourceStampFreshAt(last_external_input_stamp_, reference_stamp) &&
           sourceStampFreshAt(last_internal_input_stamp_, reference_stamp);
  }

  double sourceConflictReferenceStamp(double fallback) const {
    double result = fallback;
    if (has_external_input_ && std::isfinite(last_external_input_stamp_)) {
      result = std::max(result, last_external_input_stamp_);
    }
    if (has_internal_input_ && std::isfinite(last_internal_input_stamp_)) {
      result = std::max(result, last_internal_input_stamp_);
    }
    return result;
  }

  void recordInputStampLocked(DynamicInputSource source, double stamp, bool valid_header) {
    if (!valid_header || !std::isfinite(stamp)) {
      return;
    }
    const double tolerance = risk_evaluator_->options().time_tolerance;
    if (source == DynamicInputSource::EXTERNAL_TRACKS) {
      if (has_external_input_ && std::isfinite(last_external_input_stamp_) &&
          stamp < last_external_input_stamp_ - tolerance) {
        return;
      }
      has_external_input_ = true;
      last_external_input_stamp_ =
          std::isfinite(last_external_input_stamp_)
              ? std::max(last_external_input_stamp_, stamp)
              : stamp;
    } else {
      if (has_internal_input_ && std::isfinite(last_internal_input_stamp_) &&
          stamp < last_internal_input_stamp_ - tolerance) {
        return;
      }
      has_internal_input_ = true;
      last_internal_input_stamp_ =
          std::isfinite(last_internal_input_stamp_)
              ? std::max(last_internal_input_stamp_, stamp)
              : stamp;
    }
  }

  double currentTimeSeconds() const {
    return this->now().seconds();
  }

  bool markInformationStaleLocked(const std::string &detail, double stamp) {
    if (information_stale_.load()) {
      return false;
    }
    information_stale_.store(true);
    stale_since_stamp_ = std::isfinite(stamp) ? stamp : currentTimeSeconds();
    stale_recovery_pending_ = true;
    pending_request_.reset();
    dynamic_obstacle_update_pending_ = false;
    stale_detail_ = detail;
    return true;
  }

  static aurora_msgs::msg::RiskReport informationStaleRiskReport(
      double stamp, bool snapshot_available, double information_age,
      const std::string &detail) {
    aurora_msgs::msg::RiskReport message;
    message.stamp = secondsToTime(stamp);
    message.model_id = "aurora.conservative_3sigma_v1";
    message.dynamic_information_available = snapshot_available;
    message.dynamic_information_stale = true;
    message.information_age = std::isfinite(information_age) ? information_age : -1.0;
    message.information_risk = 1.0;
    message.total_risk = 1.0;
    message.risk_level = aurora_msgs::msg::RiskReport::UNKNOWN;
    message.detail = detail;
    return message;
  }

  static aurora_msgs::msg::RiskReport mapInformationStaleRiskReport(
      double stamp, double information_age, const std::string &detail) {
    aurora_msgs::msg::RiskReport message;
    message.stamp = secondsToTime(stamp);
    message.model_id = "aurora.conservative_3sigma_v1";
    message.static_risk = 1.0;
    message.information_risk = 1.0;
    message.total_risk = 1.0;
    message.information_age = std::isfinite(information_age) ? information_age : -1.0;
    message.risk_level = aurora_msgs::msg::RiskReport::UNKNOWN;
    message.detail = detail;
    return message;
  }

  void publishInformationStaleResult(double stamp, std::uint64_t request_id,
                                     bool snapshot_available, double information_age,
                                     const std::string &detail) {
    aurora_msgs::msg::PlanningResult message;
    message.header.stamp = secondsToTime(stamp);
    message.header.frame_id = map_frame_;
    message.request_id = request_id;
    message.status = aurora_msgs::msg::PlanningResult::VALIDATION_FAILED;
    message.detail = detail;
    message.has_trajectory = false;
    message.risk_report = informationStaleRiskReport(
        stamp, snapshot_available, information_age, detail);
    message.safety_report.stamp = secondsToTime(stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message.safety_report.map_version = map_ == nullptr ? 0U : map_->version();
    }
    message.safety_report.status = aurora_msgs::msg::SafetyReport::INFORMATION_STALE;
    message.safety_report.accepted = false;
    message.safety_report.detail = detail;
    planning_result_publisher_->publish(std::move(message));
  }

  void publishInformationStaleStatus(double stamp, std::uint64_t request_id,
                                     bool emergency, const std::string &detail) {
    aurora_msgs::msg::PlannerStatus message;
    message.header.stamp = secondsToTime(stamp);
    message.header.frame_id = map_frame_;
    message.request_id = request_id;
    message.trajectory_id = 0U;
    message.planner_state = emergency ? aurora_msgs::msg::PlannerStatus::EMERGENCY_STOP
                                      : aurora_msgs::msg::PlannerStatus::DEGRADED;
    message.planner_action = emergency
                                 ? aurora_msgs::msg::PlannerStatus::EMERGENCY_STOP_ACTION
                                 : aurora_msgs::msg::PlannerStatus::KEEP_CURRENT_TRAJECTORY;
    message.replan_trigger = aurora_msgs::msg::PlannerStatus::SAFETY_INFORMATION_STALE;
    message.valid = false;
    message.detail = detail;
    planner_status_publisher_->publish(std::move(message));
  }

  void publishMapInformationStaleResult(double stamp, std::uint64_t request_id,
                                         double information_age,
                                         const std::string &detail) {
    aurora_msgs::msg::PlanningResult message;
    message.header.stamp = secondsToTime(stamp);
    message.header.frame_id = map_frame_;
    message.request_id = request_id;
    message.status = aurora_msgs::msg::PlanningResult::VALIDATION_FAILED;
    message.detail = detail;
    message.has_trajectory = false;
    message.risk_report =
        mapInformationStaleRiskReport(stamp, information_age, detail);
    message.safety_report.stamp = secondsToTime(stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message.safety_report.map_version = map_ == nullptr ? 0U : map_->version();
    }
    message.safety_report.status = aurora_msgs::msg::SafetyReport::INFORMATION_STALE;
    message.safety_report.accepted = false;
    message.safety_report.detail = detail;
    planning_result_publisher_->publish(std::move(message));
  }

  void onInformationWatchdog() {
    const double now_stamp = currentTimeSeconds();
    if (!std::isfinite(now_stamp) || now_stamp <= 0.0) {
      return;
    }

    bool entered_stale = false;
    bool latched_stop = false;
    bool recovered = false;
    bool map_entered_stale = false;
    bool map_latched_stop = false;
    bool map_recovered = false;
    std::uint64_t request_id = 0U;
    bool snapshot_available = false;
    double information_age = std::numeric_limits<double>::quiet_NaN();
    double map_information_age = std::numeric_limits<double>::quiet_NaN();
    std::uint8_t latched_reason_code =
        aurora_msgs::msg::EmergencyStopState::UNSPECIFIED;
    std::string detail;
    std::string map_detail;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || emergency_stop_latched_.load()) {
        return;
      }
      // A timer can fire before the first simulated clock sample reaches the
      // node. Do not classify a future-stamped snapshot as stale in that gap.
      if (dynamic_snapshot_.has_snapshot && std::isfinite(dynamic_snapshot_.stamp) &&
          now_stamp + risk_evaluator_->options().time_tolerance < dynamic_snapshot_.stamp) {
        return;
      }
      const double source_reference_stamp = std::max(
          std::isfinite(last_external_input_stamp_) ? last_external_input_stamp_ : 0.0,
          std::isfinite(last_internal_input_stamp_) ? last_internal_input_stamp_ : 0.0);
      dynamic_snapshot_.source_conflict =
          dynamicInputSourcesConflictAt(std::max(now_stamp, source_reference_stamp));
      const bool usable = dynamicSnapshotUsableAt(dynamic_snapshot_, now_stamp);
      snapshot_available = dynamic_snapshot_.has_snapshot;
      if (snapshot_available && std::isfinite(dynamic_snapshot_.stamp)) {
        information_age = std::max(0.0, now_stamp - dynamic_snapshot_.stamp);
      }
      request_id = latest_request_.has_value() ? latest_request_->request_id : 0U;

      if (usable) {
        if (information_stale_.load()) {
          information_stale_.store(false);
          stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
          stale_recovery_pending_ = true;
          recovered = true;
          detail = "dynamic obstacle information recovered; explicit planning request required";
        }
      } else if (active_trajectory_.has_value()) {
        const std::string stale_reason = dynamic_snapshot_.source_conflict
                                             ? "both dynamic obstacle input sources are fresh; "
                                               "input mixing is rejected"
                                         : dynamic_snapshot_.information_incomplete
                                             ? (dynamic_snapshot_.incomplete_detail.empty()
                                                    ? "dynamic obstacle information is incomplete"
                                                    : dynamic_snapshot_.incomplete_detail)
                                         : dynamic_snapshot_.occlusion_active
                                             ? "dynamic obstacle information is occluded"
                                             : "dynamic obstacle information exceeded max age";
        entered_stale = markInformationStaleLocked(stale_reason, now_stamp);
        if (entered_stale) {
          detail = stale_reason + "; retaining the last validated trajectory temporarily";
        } else {
          detail = stale_detail_;
        }

        const bool active_remaining =
            active_trajectory_.has_value() && active_trajectory_->validated &&
            active_trajectory_->contains(now_stamp) &&
            active_trajectory_->endStamp() > now_stamp;
        const double stale_elapsed =
            std::isfinite(stale_since_stamp_)
                ? std::max(0.0, now_stamp - stale_since_stamp_)
                : std::numeric_limits<double>::infinity();
        if (active_trajectory_.has_value() &&
            (!active_remaining || stale_elapsed >= stale_hold_duration_)) {
          emergency_stop_latched_.store(true);
          emergency_reason_ =
              "dynamic obstacle information remained unusable beyond the safety hold window";
          latched_reason_code = aurora_msgs::msg::EmergencyStopState::INFORMATION_STALE;
          active_trajectory_safe_ = false;
          pending_request_.reset();
          dynamic_obstacle_update_pending_ = false;
          latched_stop = true;
          detail = emergency_reason_;
        }
      }

      const bool map_future = map_observation_available_ &&
                              std::isfinite(last_map_observation_stamp_) &&
                              now_stamp + risk_evaluator_->options().time_tolerance <
                                  last_map_observation_stamp_;
      if (!map_future) {
        const bool map_usable = mapObservationUsableAtLocked(now_stamp);
        if (map_observation_available_ && std::isfinite(last_map_observation_stamp_)) {
          map_information_age = std::max(0.0, now_stamp - last_map_observation_stamp_);
        }
        const bool map_has_work = active_trajectory_.has_value() || latest_request_.has_value() ||
                                  pending_request_.has_value();
        if (map_usable) {
          if (map_information_stale_.load()) {
            map_information_stale_.store(false);
            map_stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
            map_recovery_pending_ = true;
            map_recovered = true;
            map_detail =
                "map observation recovered; explicit planning request required";
          }
        } else if (map_has_work) {
          const std::string map_reason = map_observation_available_
                                             ? "map observation exceeded max age"
                                             : "no valid point-cloud map observation is available";
          map_entered_stale = markMapInformationStaleLocked(map_reason, now_stamp);
          if (map_entered_stale) {
            map_detail = map_reason +
                        "; retaining the last validated trajectory temporarily";
          } else {
            map_detail = map_stale_detail_;
          }

          const bool active_remaining =
              active_trajectory_.has_value() && active_trajectory_->validated &&
              active_trajectory_->contains(now_stamp) &&
              active_trajectory_->endStamp() > now_stamp;
          const double map_stale_elapsed =
              std::isfinite(map_stale_since_stamp_)
                  ? std::max(0.0, now_stamp - map_stale_since_stamp_)
                  : std::numeric_limits<double>::infinity();
          if (!emergency_stop_latched_.load() && active_trajectory_.has_value() &&
              (!active_remaining || map_stale_elapsed >= stale_hold_duration_)) {
            emergency_stop_latched_.store(true);
            emergency_reason_ =
                "map observation remained unusable beyond the safety hold window";
            active_trajectory_safe_ = false;
            pending_request_.reset();
            dynamic_obstacle_update_pending_ = false;
            map_latched_stop = true;
            map_detail = emergency_reason_;
          }
        }
      }
    }

    if (entered_stale) {
      publishInformationStaleStatus(now_stamp, request_id, false, detail);
      publishInformationStaleResult(now_stamp, request_id, snapshot_available,
                                    information_age, detail);
    } else if (recovered) {
      publishInformationStaleStatus(now_stamp, request_id, false, detail);
    }
    if (map_entered_stale) {
      publishInformationStaleStatus(now_stamp, request_id, false, map_detail);
      publishMapInformationStaleResult(now_stamp, request_id, map_information_age, map_detail);
    } else if (map_recovered) {
      publishInformationStaleStatus(now_stamp, request_id, false, map_detail);
    }
    if (latched_stop) {
      publishInformationStaleStatus(now_stamp, request_id, true, detail);
      publishEmergencyState(true, true, detail, latched_reason_code);
      condition_.notify_one();
    }
    if (map_latched_stop) {
      publishInformationStaleStatus(now_stamp, request_id, true, map_detail);
      publishEmergencyState(true, true, map_detail,
                            aurora_msgs::msg::EmergencyStopState::INFORMATION_STALE);
      condition_.notify_one();
    }
  }

  void onPlanningRequest(const aurora_msgs::msg::PlanningRequest::SharedPtr message) {
    try {
      PlanningRequest request = toCoreRequest(*message, map_frame_);
      bool rejected_rollback = false;
      bool rejected_map_stale = false;
      bool rejected_stale = false;
      bool snapshot_available = false;
      double map_information_age = std::numeric_limits<double>::quiet_NaN();
      std::string rollback_detail;
      std::string map_stale_detail;
      std::string stale_detail;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // A syntactically valid vehicle state is also the latest sensor state
        // used as the origin for subsequent point-cloud ray integration. This
        // state update is intentionally independent of planning authorization.
        latest_vehicle_position_ = request.vehicle_state.position;
        latest_vehicle_stamp_ = request.vehicle_state.stamp;
        has_latest_vehicle_state_ = true;
        if (emergency_stop_latched_.load()) {
          // Requests received while the latch is active are deliberately
          // silent and are cleared by reset; publishing a stale result would
          // make a discarded request appear to have been processed.
          pending_request_.reset();
          dynamic_obstacle_update_pending_ = false;
          return;
        }
        const double time_tolerance = risk_evaluator_->options().time_tolerance;
        if (has_last_request_planning_stamp_ &&
            request.planning_stamp < last_request_planning_stamp_ - time_tolerance) {
          rejected_rollback = true;
          rollback_detail = "planning request timestamp moved backwards from " +
                            std::to_string(last_request_planning_stamp_) + " to " +
                            std::to_string(request.planning_stamp);
          pending_request_.reset();
          dynamic_obstacle_update_pending_ = false;
        } else if (map_information_stale_.load() ||
                   !mapObservationUsableAtLocked(request.planning_stamp)) {
          const std::string reason = map_observation_available_
                                         ? "map observation exceeded max age"
                                         : "no valid point-cloud map observation is available";
          if (!map_information_stale_.load()) {
            (void)markMapInformationStaleLocked(reason, request.planning_stamp);
          }
          rejected_map_stale = true;
          map_stale_detail = map_stale_detail_.empty() ? reason : map_stale_detail_;
          if (map_observation_available_ && std::isfinite(last_map_observation_stamp_) &&
              std::isfinite(request.planning_stamp)) {
            map_information_age = std::max(
                0.0, request.planning_stamp - last_map_observation_stamp_);
          }
          pending_request_.reset();
          dynamic_obstacle_update_pending_ = false;
        } else if (information_stale_.load()) {
          pending_request_.reset();
          dynamic_obstacle_update_pending_ = false;
          rejected_stale = true;
          stale_detail = stale_detail_.empty()
                             ? "dynamic obstacle information is stale; request discarded"
                             : stale_detail_ + "; request discarded until recovery";
          snapshot_available = dynamic_snapshot_.has_snapshot;
        } else {
          // A request submitted after a recovered snapshot is the explicit
          // authorization required to leave the stale-recovery boundary.
          stale_recovery_pending_ = false;
          map_recovery_pending_ = false;
          last_request_planning_stamp_ = request.planning_stamp;
          has_last_request_planning_stamp_ = true;
          if (request.request_id >= next_internal_request_id_ &&
              request.request_id != std::numeric_limits<std::uint64_t>::max()) {
            next_internal_request_id_ = request.request_id + 1U;
          }
          latest_request_ = request;
          pending_request_ = std::move(request);
          work_pending_ = true;
        }
      }
      if (rejected_rollback) {
        publishInvalidRequest(message->header, message->request_id, rollback_detail);
        return;
      }
      if (rejected_map_stale) {
        publishMapInformationStaleResult(
            timeToSeconds(message->header.stamp), message->request_id,
            map_information_age, map_stale_detail);
        return;
      }
      if (rejected_stale) {
        publishInformationStaleResult(
            timeToSeconds(message->header.stamp), message->request_id,
            snapshot_available, std::numeric_limits<double>::quiet_NaN(),
            stale_detail);
        return;
      }
      condition_.notify_one();
    } catch (const std::exception &error) {
      publishInvalidRequest(message->header, message->request_id, error.what());
    }
  }

  void latchExecutionFault(const std::string &detail, std::uint64_t request_id) {
    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || emergency_stop_latched_.load()) {
        return;
      }
      emergency_stop_latched_.store(true);
      emergency_reason_ = detail;
      pending_request_.reset();
      dynamic_obstacle_update_pending_ = false;
      active_trajectory_.reset();
      active_trajectory_safe_ = false;
      publish = true;
    }
    if (publish) {
      publishEmergencyState(true, true, detail,
                            aurora_msgs::msg::EmergencyStopState::SAFETY_GATE,
                            request_id);
      condition_.notify_one();
    }
  }

  void onExecutionVehicleState(const aurora_msgs::msg::VehicleState::SharedPtr message) {
    const double stamp = timeToSeconds(message->header.stamp);
    const double now_stamp = currentTimeSeconds();
    if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U ||
        (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) ||
        !std::isfinite(stamp) ||
        (std::isfinite(now_stamp) && now_stamp > 0.0 &&
         stamp > now_stamp + risk_evaluator_->options().time_tolerance) ||
        !std::isfinite(message->position.x) || !std::isfinite(message->position.y) ||
        !std::isfinite(message->position.z) || !std::isfinite(message->velocity.x) ||
        !std::isfinite(message->velocity.y) || !std::isfinite(message->velocity.z) ||
        !std::isfinite(message->acceleration.x) ||
        !std::isfinite(message->acceleration.y) ||
        !std::isfinite(message->acceleration.z)) {
      latchExecutionFault("flight-controller vehicle state is invalid", 0U);
      return;
    }

    aurora::planner::VehicleState state;
    state.stamp = stamp;
    state.position = toEigen(message->position);
    state.velocity = toEigen(message->velocity);
    state.acceleration = toEigen(message->acceleration);
    state.has_position_covariance = message->has_state_covariance;
    if (state.has_position_covariance) {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          const double value = message->state_covariance[static_cast<std::size_t>(row * 9 + column)];
          if (!std::isfinite(value)) {
            latchExecutionFault("flight-controller vehicle covariance is invalid", 0U);
            return;
          }
          state.position_covariance(row, column) = value;
        }
      }
    }

    bool timestamp_rollback = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double tolerance = risk_evaluator_->options().time_tolerance;
      if (has_execution_state_ &&
          stamp < latest_execution_state_.stamp - tolerance) {
        timestamp_rollback = true;
      } else {
        latest_execution_state_ = state;
        has_execution_state_ = true;
        latest_vehicle_position_ = state.position;
        latest_vehicle_stamp_ = state.stamp;
        has_latest_vehicle_state_ = true;
      }
    }
    if (timestamp_rollback) {
      latchExecutionFault("flight-controller vehicle-state timestamp moved backwards", 0U);
    }
  }

  std::uint64_t nextInternalRequestIdLocked() {
    if (next_internal_request_id_ == 0U) {
      next_internal_request_id_ = 1U;
    }
    const std::uint64_t result = next_internal_request_id_++;
    if (next_internal_request_id_ == 0U) {
      next_internal_request_id_ = 1U;
    }
    return result;
  }

  bool scheduleExecutionReplanLocked(double feedback_stamp) {
    if (!latest_request_.has_value()) {
      return false;
    }
    PlanningRequest request = *latest_request_;
    const double tolerance = risk_evaluator_->options().time_tolerance;
    double stamp = feedback_stamp;
    if (!std::isfinite(stamp)) {
      stamp = currentTimeSeconds();
    }
    if (!std::isfinite(stamp)) {
      return false;
    }

    if (has_execution_state_ && std::isfinite(latest_execution_state_.stamp)) {
      request.vehicle_state = latest_execution_state_;
      stamp = latest_execution_state_.stamp;
    } else if (active_trajectory_.has_value() && active_trajectory_->contains(stamp)) {
      const auto state = active_trajectory_->evaluate(stamp);
      request.vehicle_state.stamp = state.stamp;
      request.vehicle_state.position = state.position;
      request.vehicle_state.velocity = state.velocity;
      request.vehicle_state.acceleration = state.acceleration;
    } else if (!std::isfinite(request.vehicle_state.stamp) ||
               std::abs(request.vehicle_state.stamp - stamp) > tolerance) {
      return false;
    }

    if (has_last_request_planning_stamp_ &&
        stamp < last_request_planning_stamp_ - tolerance) {
      stamp = last_request_planning_stamp_;
      if (has_execution_state_) {
        request.vehicle_state.stamp = stamp;
      }
    }
    request.request_id = nextInternalRequestIdLocked();
    request.planning_stamp = stamp;
    request.vehicle_state.stamp = stamp;
    latest_request_ = request;
    pending_request_ = std::move(request);
    last_request_planning_stamp_ = stamp;
    has_last_request_planning_stamp_ = true;
    work_pending_ = true;
    return true;
  }

  void onExecutionStatus(
      const aurora_msgs::msg::TrajectoryExecutionStatus::SharedPtr message) {
    const double now_stamp = currentTimeSeconds();
    if ((!message->header.frame_id.empty() && message->header.frame_id != map_frame_) ||
        message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U) {
      latchExecutionFault("flight-controller execution feedback has an invalid header", 0U);
      return;
    }

    std::uint64_t expected_id = message->trajectory_id;
    std::uint64_t request_id = 0U;
    bool active = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || emergency_stop_latched_.load()) {
        return;
      }
      active = active_trajectory_.has_value();
      expected_id = active ? active_trajectory_->trajectory_id : message->trajectory_id;
      request_id = latest_request_.has_value() ? latest_request_->request_id : 0U;
    }

    const auto observation = flight_admission_->observeFeedback(*message, expected_id, now_stamp);
    if (observation.action == aurora::flight::FeedbackAction::EMERGENCY_STOP ||
        (!observation.valid && observation.action != aurora::flight::FeedbackAction::REQUEST_REPLAN)) {
      latchExecutionFault(observation.detail.empty()
                              ? "flight-controller reported an invalid safety state"
                              : observation.detail,
                          request_id);
      return;
    }
    if (!active && message->status == aurora_msgs::msg::TrajectoryExecutionStatus::IDLE) {
      return;
    }

    const double feedback_stamp = timeToSeconds(message->header.stamp);
    if (observation.action == aurora::flight::FeedbackAction::COMPLETED) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_trajectory_.has_value() &&
          active_trajectory_->trajectory_id == message->trajectory_id) {
        active_trajectory_.reset();
        active_trajectory_safe_ = false;
      }
      return;
    }
    if (observation.action != aurora::flight::FeedbackAction::REQUEST_REPLAN) {
      return;
    }

    bool scheduled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!emergency_stop_latched_.load()) {
        scheduled = scheduleExecutionReplanLocked(feedback_stamp);
        active_trajectory_.reset();
        active_trajectory_safe_ = false;
      }
    }
    if (scheduled) {
      RCLCPP_WARN(get_logger(), "execution feedback requested replanning: %s",
                  observation.detail.c_str());
      condition_.notify_one();
    } else if (active) {
      latchExecutionFault(
          "flight-controller rejected the active trajectory without a valid state for replanning",
          request_id);
    }
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr message) {
    if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000U) {
      RCLCPP_WARN(get_logger(), "ignoring camera info with an invalid timestamp");
      return;
    }
    const double stamp = timeToSeconds(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    const bool message_stamp_is_zero =
        message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U;
    const bool latest_stamp_is_zero =
        latest_camera_info_.has_value() && latest_camera_info_->header.stamp.sec == 0 &&
        latest_camera_info_->header.stamp.nanosec == 0U;
    if (latest_camera_info_.has_value() && !message_stamp_is_zero && !latest_stamp_is_zero &&
        std::isfinite(latest_camera_info_stamp_) &&
        stamp < latest_camera_info_stamp_ - depth_adapter_->options().camera_info_time_tolerance) {
      RCLCPP_WARN(get_logger(),
                  "ignoring out-of-order camera info at %.9f; latest is %.9f",
                  stamp, latest_camera_info_stamp_);
      return;
    }
    latest_camera_info_ = *message;
    latest_camera_info_stamp_ = stamp;
  }

  void onDepthImage(const sensor_msgs::msg::Image::SharedPtr message) {
    std::optional<sensor_msgs::msg::CameraInfo> camera_info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      camera_info = latest_camera_info_;
    }
    if (!camera_info.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "ignoring depth image until CameraInfo is received");
      return;
    }

    const DepthImagePointCloud converted = depth_adapter_->convert(*message, *camera_info);
    if (!converted.valid) {
      RCLCPP_WARN(get_logger(), "ignoring depth image: %s", converted.detail.c_str());
      return;
    }
    if (converted.endpoints.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "depth image has no valid samples; map freshness is unchanged");
      return;
    }

    Eigen::Vector3d ray_origin = Eigen::Vector3d::Zero();
    geometry_msgs::msg::TransformStamped transform;
    const bool identity_transform = message->header.frame_id == map_frame_;
    if (!identity_transform) {
      try {
        const rclcpp::Time image_time(message->header.stamp);
        transform = tf_buffer_.lookupTransform(
            map_frame_, message->header.frame_id, image_time,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
        ray_origin = Eigen::Vector3d(
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z);
        Eigen::Vector3d ignored_origin;
        if (!transformPoint(transform, Eigen::Vector3d::Zero(), &ignored_origin)) {
          throw std::runtime_error("depth image TF contains a non-finite transform");
        }
      } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "ignoring depth image because TF conversion failed: %s",
                    error.what());
        return;
      }
    }

    const double observation_stamp = converted.stamp;
    double observation_age = 0.0;
    bool map_recovered = false;
    std::uint64_t map_recovery_request_id = 0U;
    std::size_t processed_points = 0U;
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      const double timestamp_tolerance = risk_evaluator_->options().time_tolerance;
      if (map_observation_available_ && std::isfinite(last_map_observation_stamp_) &&
          observation_stamp < last_map_observation_stamp_ - timestamp_tolerance) {
        RCLCPP_WARN(get_logger(),
                    "ignoring out-of-order depth image at %.9f; latest map observation is %.9f",
                    observation_stamp, last_map_observation_stamp_);
        return;
      }
      if (has_latest_vehicle_state_ && std::isfinite(latest_vehicle_stamp_)) {
        observation_age = std::max(0.0, latest_vehicle_stamp_ - observation_stamp);
      }
      for (const auto &endpoint_in_sensor_frame : converted.endpoints) {
        Eigen::Vector3d endpoint = endpoint_in_sensor_frame;
        if (!identity_transform && !transformPoint(transform, endpoint_in_sensor_frame, &endpoint)) {
          continue;
        }
        const double ray_length = (endpoint - ray_origin).norm();
        if (!endpoint.allFinite() || !ray_origin.allFinite() ||
            !std::isfinite(ray_length) || ray_length <= 0.0) {
          continue;
        }
        map_->integrateRay(aurora::map::RayObservation{
            ray_origin, endpoint, true, ray_length, observation_age, depth_confidence_});
        ++processed_points;
      }
      if (processed_points == 0U) {
        return;
      }
      map_->inflate(map_inflation_radius_);
      map_observation_available_ = true;
      last_map_observation_stamp_ = observation_stamp;
      if (map_information_stale_.load()) {
        map_information_stale_.store(false);
        map_stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
        map_recovery_pending_ = true;
        map_recovered = true;
        map_recovery_request_id =
            latest_request_.has_value() ? latest_request_->request_id : 0U;
      }
      if (pending_request_.has_value()) {
        work_pending_ = true;
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "depth image map update failed: %s", error.what());
      return;
    }
    condition_.notify_one();
    if (map_recovered) {
      publishInformationStaleStatus(
          observation_stamp, map_recovery_request_id, false,
          "map observation recovered; explicit planning request required");
    }
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (!hasPointField(*message, "x") || !hasPointField(*message, "y") ||
        !hasPointField(*message, "z")) {
      RCLCPP_WARN(get_logger(), "ignoring point cloud without x/y/z fields");
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed;
    Eigen::Vector3d ray_origin = Eigen::Vector3d::Zero();
    try {
      if (message->header.frame_id == map_frame_) {
        transformed = *message;
        std::lock_guard<std::mutex> lock(mutex_);
        ray_origin = has_latest_vehicle_state_ ? latest_vehicle_position_ : map_->origin();
      } else {
        const rclcpp::Time cloud_time(message->header.stamp);
        const auto transform = tf_buffer_.lookupTransform(
            map_frame_, message->header.frame_id, cloud_time,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
        tf2::doTransform(*message, transformed, transform);
        ray_origin = Eigen::Vector3d(transform.transform.translation.x,
                                     transform.transform.translation.y,
                                     transform.transform.translation.z);
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "ignoring point cloud because TF conversion failed: %s",
                  error.what());
      return;
    }

    double observation_age = 0.0;
    const double cloud_stamp = timeToSeconds(message->header.stamp);
    bool map_recovered = false;
    std::uint64_t map_recovery_request_id = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!std::isfinite(cloud_stamp)) {
        RCLCPP_WARN(get_logger(), "ignoring point cloud with a non-finite timestamp");
        return;
      }
      const double timestamp_tolerance = risk_evaluator_->options().time_tolerance;
      if (map_observation_available_ && std::isfinite(last_map_observation_stamp_) &&
          cloud_stamp < last_map_observation_stamp_ - timestamp_tolerance) {
        RCLCPP_WARN(get_logger(),
                    "ignoring out-of-order point cloud at %.9f; latest map observation is %.9f",
                    cloud_stamp, last_map_observation_stamp_);
        return;
      }
      if (has_latest_vehicle_state_ && std::isfinite(latest_vehicle_stamp_) &&
          std::isfinite(cloud_stamp)) {
        observation_age = std::max(0.0, latest_vehicle_stamp_ - cloud_stamp);
      }
    }

    std::size_t processed_points = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed, "z");
      std::lock_guard<std::mutex> lock(mutex_);
      for (; iter_x != iter_x.end() && processed_points < static_cast<std::size_t>(
                                               std::max<std::int64_t>(0, pointcloud_max_points_));
           ++iter_x, ++iter_y, ++iter_z) {
        const Eigen::Vector3d endpoint(static_cast<double>(*iter_x), static_cast<double>(*iter_y),
                                       static_cast<double>(*iter_z));
        if (!endpoint.allFinite()) {
          continue;
        }
        map_->integrateRay(aurora::map::RayObservation{
            ray_origin, endpoint, true, pointcloud_max_range_, observation_age,
            pointcloud_confidence_});
        ++processed_points;
      }
      map_->inflate(map_inflation_radius_);
      if (processed_points > 0U) {
        map_observation_available_ = true;
        last_map_observation_stamp_ = cloud_stamp;
        if (map_information_stale_.load()) {
          map_information_stale_.store(false);
          map_stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
          map_recovery_pending_ = true;
          map_recovered = true;
          map_recovery_request_id =
              latest_request_.has_value() ? latest_request_->request_id : 0U;
        }
      }
      if (pending_request_.has_value()) {
        work_pending_ = true;
      }
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "point cloud map update failed: %s", error.what());
      return;
    }
    condition_.notify_one();
    if (map_recovered) {
      publishInformationStaleStatus(
          cloud_stamp, map_recovery_request_id, false,
          "map observation recovered; explicit planning request required");
    }
  }

  DynamicTrackSnapshot makeInternalSnapshotLocked(
      const UnassociatedDetectionBatch &batch) {
    DynamicTrackSnapshot snapshot;
    snapshot.has_snapshot = batch.has_batch;
    snapshot.valid_header = batch.valid_header;
    snapshot.stamp = batch.stamp;
    snapshot.invalid_track_count = batch.invalid_detection_count;
    snapshot.occlusion_active = batch.occlusion_active;
    if (!batch.valid_header) {
      return snapshot;
    }

    const TrackingResult tracking_result = tracker_->update(batch.stamp, batch.detections);
    if (tracking_result.status != aurora::tracking::TrackingStatus::SUCCESS &&
        tracking_result.status != aurora::tracking::TrackingStatus::PARTIAL_INPUT) {
      ++snapshot.invalid_track_count;
    }
    for (const auto &estimate : tracking_result.tracks) {
      snapshot.tracks.push_back(estimate.state);
      if (estimate.lifecycle == LifecycleState::LOST) {
        snapshot.information_incomplete = true;
      }
    }
    if (snapshot.information_incomplete) {
      snapshot.incomplete_detail =
          "one or more internally tracked obstacles exceeded the lost threshold";
    }
    return snapshot;
  }

  void onDynamicObstacles(
      const aurora_msgs::msg::DynamicObstacleTrackArray::SharedPtr message) {
    DynamicTrackSnapshot snapshot = dynamic_obstacle_adapter_->convert(*message);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      recordInputStampLocked(DynamicInputSource::EXTERNAL_TRACKS,
                             snapshot.stamp, snapshot.valid_header);
      if (dynamic_input_mode_ != DynamicInputMode::EXTERNAL_TRACKS) {
        // The selected source remains authoritative; this source is retained
        // only to detect an unsafe simultaneous-input configuration.
        snapshot = DynamicTrackSnapshot{};
      } else {
        snapshot.source_conflict = dynamicInputSourcesConflictAt(
            sourceConflictReferenceStamp(snapshot.stamp));
      }
    }
    if (dynamic_input_mode_ != DynamicInputMode::EXTERNAL_TRACKS) {
      onIgnoredDynamicInput(DynamicInputSource::EXTERNAL_TRACKS);
      return;
    }
    if (snapshot.invalid_track_count > 0U) {
      RCLCPP_WARN(get_logger(),
                  "dynamic obstacle snapshot contains %zu invalid batch/track entries; "
                  "dynamic risk will be gated",
                  snapshot.invalid_track_count);
    }
    onDynamicSnapshot(std::move(snapshot), DynamicInputSource::EXTERNAL_TRACKS);
  }

  void onUnassociatedObstacles(
      const aurora_msgs::msg::UnassociatedObstacleDetectionArray::SharedPtr message) {
    const UnassociatedDetectionBatch batch =
        unassociated_obstacle_adapter_->convert(*message);
    if (batch.invalid_detection_count > 0U) {
      RCLCPP_WARN(get_logger(),
                  "unassociated obstacle batch contains %zu invalid entries; "
                  "dynamic risk will be gated",
                  batch.invalid_detection_count);
    }
    if (dynamic_input_mode_ != DynamicInputMode::INTERNAL_DETECTIONS) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        recordInputStampLocked(DynamicInputSource::INTERNAL_DETECTIONS,
                               batch.stamp, batch.valid_header);
      }
      onIgnoredDynamicInput(DynamicInputSource::INTERNAL_DETECTIONS);
      return;
    }

    DynamicTrackSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (batch.valid_header && has_internal_input_ &&
          batch.stamp <= last_internal_input_stamp_ +
                              risk_evaluator_->options().time_tolerance) {
        // ObstacleTracker intentionally rejects repeated or out-of-order
        // batches. Do not replace the last valid internal snapshot with that
        // rejection, otherwise a repeated DDS sample becomes false stale data.
        return;
      }
      recordInputStampLocked(DynamicInputSource::INTERNAL_DETECTIONS,
                             batch.stamp, batch.valid_header);
      snapshot = makeInternalSnapshotLocked(batch);
      snapshot.source_conflict = dynamicInputSourcesConflictAt(
          sourceConflictReferenceStamp(snapshot.stamp));
    }
    onDynamicSnapshot(std::move(snapshot), DynamicInputSource::INTERNAL_DETECTIONS);
  }

  void onIgnoredDynamicInput(DynamicInputSource source) {
    bool entered_stale = false;
    bool snapshot_available = false;
    double stamp = currentTimeSeconds();
    std::uint64_t request_id = 0U;
    std::string detail;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!dynamic_snapshot_.has_snapshot) {
        return;
      }
      const double reference_stamp = std::max(
          std::isfinite(last_external_input_stamp_) ? last_external_input_stamp_ : 0.0,
          std::isfinite(last_internal_input_stamp_) ? last_internal_input_stamp_ : 0.0);
      dynamic_snapshot_.source_conflict = dynamicInputSourcesConflictAt(reference_stamp);
      if (!dynamic_snapshot_.source_conflict) {
        return;
      }
      stamp = dynamic_snapshot_.stamp;
      snapshot_available = dynamic_snapshot_.has_snapshot;
      request_id = latest_request_.has_value() ? latest_request_->request_id : 0U;
      detail = "both dynamic obstacle input sources are fresh; input mixing is rejected";
      if (active_trajectory_.has_value() || latest_request_.has_value()) {
        entered_stale = markInformationStaleLocked(detail, stamp);
        if (!entered_stale) {
          detail = stale_detail_;
        }
      }
      pending_request_.reset();
      dynamic_obstacle_update_pending_ = false;
    }
    (void)source;
    if (entered_stale) {
      publishInformationStaleStatus(stamp, request_id, false, detail);
      publishInformationStaleResult(stamp, request_id, snapshot_available, 0.0, detail);
    }
  }

  void onDynamicSnapshot(DynamicTrackSnapshot snapshot, DynamicInputSource source) {
    bool entered_stale = false;
    bool recovered = false;
    bool stale_snapshot_available = false;
    double stale_stamp = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t stale_request_id = 0U;
    std::string state_detail;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double timestamp_tolerance = dynamic_obstacle_adapter_->timeTolerance();
      if (dynamic_snapshot_.has_snapshot && std::isfinite(dynamic_snapshot_.stamp) &&
          snapshot.has_snapshot && std::isfinite(snapshot.stamp) &&
          snapshot.stamp < dynamic_snapshot_.stamp - timestamp_tolerance) {
        RCLCPP_WARN(get_logger(),
                    "ignoring out-of-order dynamic snapshot at %.9f; latest is %.9f",
                    snapshot.stamp, dynamic_snapshot_.stamp);
        return;
      }
      active_dynamic_source_ = source;
      dynamic_snapshot_ = std::move(snapshot);
      stale_snapshot_available = dynamic_snapshot_.has_snapshot;
      stale_stamp = dynamic_snapshot_.stamp;
      stale_request_id = latest_request_.has_value() ? latest_request_->request_id : 0U;
      const bool usable_at_snapshot = dynamicSnapshotUsableAt(
          dynamic_snapshot_, dynamic_snapshot_.stamp);
      if (!usable_at_snapshot) {
        std::string stale_reason;
        if (dynamic_snapshot_.source_conflict) {
          stale_reason =
              "both dynamic obstacle input sources are fresh; input mixing is rejected";
        } else if (dynamic_snapshot_.information_incomplete) {
          stale_reason = dynamic_snapshot_.incomplete_detail.empty()
                             ? "internal dynamic obstacle information is incomplete"
                             : dynamic_snapshot_.incomplete_detail;
        } else if (dynamic_snapshot_.occlusion_active) {
          stale_reason = "dynamic obstacle information is occluded";
        } else {
          stale_reason = "dynamic obstacle snapshot is invalid";
        }
        if (active_trajectory_.has_value() || latest_request_.has_value() ||
            dynamic_snapshot_.source_conflict || dynamic_snapshot_.information_incomplete) {
          entered_stale = markInformationStaleLocked(stale_reason, dynamic_snapshot_.stamp);
          state_detail = entered_stale ? stale_reason +
                                             "; retaining the last validated trajectory temporarily"
                                       : stale_detail_;
        }
        pending_request_.reset();
        dynamic_obstacle_update_pending_ = false;
      } else if (information_stale_.load()) {
        information_stale_.store(false);
        stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
        stale_recovery_pending_ = true;
        recovered = true;
        state_detail =
            "dynamic obstacle information recovered; explicit planning request required";
      }

      if (!usable_at_snapshot || stale_recovery_pending_ ||
          emergency_stop_latched_.load()) {
        work_pending_ = false;
      } else {
        const bool newer_than_latest_request =
            latest_request_.has_value() && dynamic_snapshot_.has_snapshot &&
            std::isfinite(dynamic_snapshot_.stamp) &&
            dynamic_snapshot_.stamp >
                latest_request_->planning_stamp + dynamic_obstacle_adapter_->timeTolerance();
        if (pending_request_.has_value()) {
          if (newer_than_latest_request) {
            if (active_trajectory_.has_value() &&
                synchronizeRequestStateToTrajectory(
                    &*latest_request_, *active_trajectory_, dynamic_snapshot_.stamp) &&
                synchronizeRequestStateToTrajectory(
                    &*pending_request_, *active_trajectory_, dynamic_snapshot_.stamp)) {
              last_request_planning_stamp_ = dynamic_snapshot_.stamp;
              has_last_request_planning_stamp_ = true;
              dynamic_obstacle_update_pending_ = true;
            } else {
              RCLCPP_WARN(
                  get_logger(),
                  "dynamic snapshot at %.9f cannot be synchronized to the active trajectory; "
                  "waiting for an explicit vehicle-state request",
                  dynamic_snapshot_.stamp);
            }
          }
          work_pending_ = true;
        } else if (!emergency_stop_latched_.load() && active_trajectory_.has_value() &&
                   newer_than_latest_request) {
          // Dynamic observations are planning events. Reuse the latest reference
          // and synchronize the aircraft state to the observation time from the
          // active validated trajectory. Never plan from a stale vehicle state.
          PlanningRequest dynamic_replan = *latest_request_;
          if (synchronizeRequestStateToTrajectory(
                  &dynamic_replan, *active_trajectory_, dynamic_snapshot_.stamp)) {
            latest_request_ = dynamic_replan;
            last_request_planning_stamp_ = dynamic_snapshot_.stamp;
            has_last_request_planning_stamp_ = true;
            pending_request_ = std::move(dynamic_replan);
            dynamic_obstacle_update_pending_ = true;
          } else {
            RCLCPP_WARN(
                get_logger(),
                "dynamic snapshot at %.9f is outside the active trajectory; "
                "waiting for an explicit vehicle-state request",
                dynamic_snapshot_.stamp);
          }
          work_pending_ = true;
        }
      }
    }
    if (entered_stale) {
      publishInformationStaleStatus(stale_stamp, stale_request_id, false, state_detail);
      publishInformationStaleResult(stale_stamp, stale_request_id, stale_snapshot_available,
                                    0.0, state_detail);
    } else if (recovered) {
      publishInformationStaleStatus(stale_stamp, stale_request_id, false, state_detail);
    }
    condition_.notify_one();
  }

  static bool synchronizeRequestStateToTrajectory(
      PlanningRequest *request, const PlannedTrajectory &trajectory, double stamp) {
    constexpr double kTimeEpsilon = 1e-12;
    if (request == nullptr || !std::isfinite(stamp) || !trajectory.validated ||
        !trajectory.contains(stamp) || trajectory.endStamp() <= stamp + kTimeEpsilon) {
      return false;
    }
    try {
      const auto state = trajectory.evaluate(stamp);
      if (!std::isfinite(state.stamp) || !state.position.allFinite() ||
          !state.velocity.allFinite() || !state.acceleration.allFinite()) {
        return false;
      }
      request->planning_stamp = stamp;
      request->vehicle_state.stamp = stamp;
      request->vehicle_state.position = state.position;
      request->vehicle_state.velocity = state.velocity;
      request->vehicle_state.acceleration = state.acceleration;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void onEmergencyStop(
      const std::shared_ptr<aurora_msgs::srv::SetEmergencyStop::Request> request,
      std::shared_ptr<aurora_msgs::srv::SetEmergencyStop::Response> response) {
    if (request->engage) {
      emergency_stop_latched_.store(true);
      std::string reason;
      std::uint64_t request_id = 0U;
      std::uint8_t reason_code = aurora_msgs::msg::EmergencyStopState::OPERATOR;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        emergency_reason_ = request->reason.empty() ? "emergency stop requested" : request->reason;
        reason = emergency_reason_;
        request_id = latest_request_.has_value() ? latest_request_->request_id : 0U;
        // Do not allow work queued before the stop request to execute after
        // the safety latch is eventually reset.
        pending_request_.reset();
        dynamic_obstacle_update_pending_ = false;
        active_trajectory_.reset();
        active_trajectory_safe_ = false;
        work_pending_ = true;
      }
      response->accepted = true;
      response->active = true;
      response->latched = true;
      response->detail = "emergency stop latched";
      publishEmergencyState(true, true, reason, reason_code, request_id);
      condition_.notify_one();
      return;
    }

    emergency_stop_latched_.store(false);
    reset_requested_.store(true);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++reset_generation_;
      pending_request_.reset();
      latest_request_.reset();
      active_trajectory_.reset();
      active_trajectory_safe_ = false;
      dynamic_snapshot_ = DynamicTrackSnapshot{};
      tracker_->reset();
      has_external_input_ = false;
      has_internal_input_ = false;
      last_external_input_stamp_ = std::numeric_limits<double>::quiet_NaN();
      last_internal_input_stamp_ = std::numeric_limits<double>::quiet_NaN();
      dynamic_obstacle_update_pending_ = false;
      map_information_stale_.store(false);
      map_recovery_pending_ = false;
      map_stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
      map_stale_detail_.clear();
      if (map_freshness_required_) {
        map_observation_available_ = false;
        last_map_observation_stamp_ = std::numeric_limits<double>::quiet_NaN();
      }
      information_stale_.store(false);
      stale_recovery_pending_ = false;
      stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
      stale_detail_.clear();
      has_last_request_planning_stamp_ = false;
      last_request_planning_stamp_ = std::numeric_limits<double>::quiet_NaN();
      work_pending_ = true;
    }
    response->accepted = true;
    response->active = false;
    response->latched = false;
    response->detail = "emergency stop reset requested";
    publishEmergencyState(false, false, "emergency stop reset requested",
                          aurora_msgs::msg::EmergencyStopState::UNSPECIFIED);
    condition_.notify_one();
  }

  std::vector<TrajectorySample> sampleTrajectory(const PlannedTrajectory &trajectory) const {
    if (trajectory.empty() || !std::isfinite(trajectory.startStamp()) ||
        !std::isfinite(trajectory.endStamp()) || trajectory.endStamp() <= trajectory.startStamp()) {
      throw std::invalid_argument("cannot sample an empty or invalid trajectory");
    }
    const double start_stamp = trajectory.startStamp();
    const double duration = trajectory.endStamp() - start_stamp;
    const long double estimated_intervals = std::ceil(
        static_cast<long double>(duration) / static_cast<long double>(risk_sample_interval_));
    if (estimated_intervals > static_cast<long double>(risk_evaluator_->options().max_samples - 1U)) {
      throw std::invalid_argument("risk trajectory sample count exceeds configured maximum");
    }
    const std::size_t interval_count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(estimated_intervals));
    std::vector<TrajectorySample> samples;
    samples.reserve(interval_count + 1U);
    const double sample_time_tolerance =
        std::max(1e-12, risk_evaluator_->options().time_tolerance);
    const auto append_sample = [&](double stamp) {
      if (!samples.empty() && stamp <= samples.back().stamp + sample_time_tolerance) {
        // ceil(duration / interval) can produce a regular sample that is
        // numerically equal to the explicit endpoint. Replace a nearly equal
        // final sample with the exact endpoint rather than emitting a duplicate
        // timestamp that the risk evaluator correctly rejects.
        if (stamp > samples.back().stamp) {
          const auto state = trajectory.evaluate(stamp);
          samples.back().stamp = stamp;
          samples.back().position = state.position;
        }
        return;
      }
      const auto state = trajectory.evaluate(stamp);
      TrajectorySample sample;
      sample.stamp = stamp;
      sample.position = state.position;
      sample.position_covariance.setZero();
      samples.push_back(std::move(sample));
    };
    for (std::size_t index = 0U; index < interval_count; ++index) {
      append_sample(std::min(
          trajectory.endStamp(),
          start_stamp + risk_sample_interval_ * static_cast<double>(index)));
    }
    append_sample(trajectory.endStamp());
    return samples;
  }

  DynamicRiskInput makeDynamicRiskInput(
      const DynamicTrackSnapshot &snapshot, double trajectory_end_stamp,
      double evaluation_stamp,
      const aurora::planner::VehicleState &vehicle_state,
      const VoxelMap &map,
      const std::vector<TrajectorySample> &trajectory_samples) const {
    DynamicRiskInput input;
    input.has_snapshot = snapshot.has_snapshot;
    input.snapshot_stamp = snapshot.stamp;
    input.evaluation_stamp = evaluation_stamp;
    input.invalid_track_count = snapshot.invalid_track_count;
    input.occlusion_active = snapshot.occlusion_active ||
                             snapshot.information_incomplete ||
                             snapshot.source_conflict;
    input.occluded_track_count = snapshot.occluded_track_ids.size();
    input.context.vehicle.has_localization_position_covariance =
        vehicle_state.has_position_covariance;
    if (vehicle_state.has_position_covariance) {
      input.context.vehicle.localization_position_covariance =
          vehicle_state.position_covariance;
    }
    input.context.vehicle.has_execution_position_covariance =
        execution_position_variance_ > 0.0;
    if (input.context.vehicle.has_execution_position_covariance) {
      input.context.vehicle.execution_position_covariance =
          execution_position_variance_ * Eigen::Matrix3d::Identity();
    }
    input.context.delay = risk_delay_;
    if (risk_map_quality_enabled_) {
      input.context.map.available = true;
      input.context.map.snapshot_stamp = std::isfinite(evaluation_stamp)
                                             ? evaluation_stamp
                                             : trajectory_samples.front().stamp;
      input.context.map.map_version = map.version();
      input.context.map.samples.reserve(trajectory_samples.size());
      for (const auto &trajectory_sample : trajectory_samples) {
        const auto query = map.query(trajectory_sample.position);
        MapQualitySample map_sample;
        map_sample.stamp = trajectory_sample.stamp;
        map_sample.position = trajectory_sample.position;
        switch (query.state) {
          case aurora::map::MapState::FREE:
            map_sample.state = MapRiskState::FREE;
            break;
          case aurora::map::MapState::OCCUPIED:
            map_sample.state = MapRiskState::OCCUPIED;
            break;
          case aurora::map::MapState::UNKNOWN:
            map_sample.state = MapRiskState::UNKNOWN;
            break;
          case aurora::map::MapState::OUT_OF_MAP:
            map_sample.state = MapRiskState::OUT_OF_MAP;
            break;
        }
        map_sample.occupancy_probability = query.occupancy_probability;
        map_sample.observation_age = query.observation_age;
        map_sample.confidence = query.confidence;
        map_sample.inflated = query.inflated;
        map_sample.map_version = query.map_version;
        input.context.map.samples.push_back(std::move(map_sample));
      }
    }
    if (!snapshot.has_snapshot || !std::isfinite(trajectory_end_stamp)) {
      return input;
    }
    input.predictions.reserve(snapshot.tracks.size());
    for (const auto &track : snapshot.tracks) {
      const double effective_end_stamp = trajectory_end_stamp + risk_delay_.total();
      const double horizon = effective_end_stamp - track.stamp;
      input.predictions.push_back(predictor_->predict(track, horizon));
    }
    return input;
  }

  DynamicRiskEvaluation evaluateDynamicRisk(const PlannedTrajectory &trajectory,
                                            const DynamicTrackSnapshot &snapshot,
                                            double evaluation_stamp,
                                            const aurora::planner::VehicleState &vehicle_state,
                                            const VoxelMap &map) const {
    DynamicRiskEvaluation result;
    try {
      const auto samples = sampleTrajectory(trajectory);
      const auto input = makeDynamicRiskInput(snapshot, trajectory.endStamp(), evaluation_stamp,
                                              vehicle_state, map, samples);
      return risk_evaluator_->evaluate(samples, input);
    } catch (const std::exception &error) {
      result.status = RiskStatus::INVALID_INPUT;
      result.level = aurora::risk::RiskLevel::UNKNOWN;
      result.accepted = false;
      result.detail = std::string("dynamic risk evaluation failed: ") + error.what();
      return result;
    }
  }

  std::optional<RiskCostFunction> makeDynamicRiskCostFunction(
      const DynamicTrackSnapshot &snapshot,
      const aurora::planner::VehicleState &vehicle_state,
      double planning_stamp) const {
    if (!risk_soft_cost_enabled_ || planner_->options().optimizer.lambda_risk <= 0.0 ||
        !dynamicSnapshotUsableAt(snapshot, planning_stamp) || snapshot.tracks.empty()) {
      return std::nullopt;
    }

    std::vector<PredictionResult> predictions;
    predictions.reserve(snapshot.tracks.size());
    for (const auto &track : snapshot.tracks) {
      const PredictionResult prediction =
          predictor_->predict(track, predictor_->options().max_horizon);
      if (prediction.status != aurora::prediction::PredictionStatus::SUCCESS) {
        const std::string detail = "dynamic soft-risk prediction failed: " +
                                   prediction.detail;
        return RiskCostFunction([detail](double, const Eigen::Vector3d &) {
          RiskCostEvaluation result;
          result.valid = false;
          result.detail = detail;
          return result;
        });
      }
      predictions.push_back(prediction);
    }

    RiskContext context;
    context.vehicle.has_localization_position_covariance =
        vehicle_state.has_position_covariance;
    if (vehicle_state.has_position_covariance) {
      context.vehicle.localization_position_covariance =
          vehicle_state.position_covariance;
    }
    context.vehicle.has_execution_position_covariance = execution_position_variance_ > 0.0;
    if (context.vehicle.has_execution_position_covariance) {
      context.vehicle.execution_position_covariance =
          execution_position_variance_ * Eigen::Matrix3d::Identity();
    }
    context.delay = risk_delay_;

    try {
      auto field = std::make_shared<DynamicRiskCostField>(
          std::move(predictions), std::move(context), soft_risk_options_);
      return RiskCostFunction([field](double stamp, const Eigen::Vector3d &position) {
        return field->evaluate(stamp, position);
      });
    } catch (const std::exception &error) {
      const std::string detail =
          std::string("dynamic soft-risk field construction failed: ") + error.what();
      return RiskCostFunction([detail](double, const Eigen::Vector3d &) {
        RiskCostEvaluation result;
        result.valid = false;
        result.detail = detail;
        return result;
      });
    }
  }

  void planningLoop() {
    while (true) {
      std::optional<PlanningRequest> request;
      std::optional<PlannedTrajectory> current;
      std::optional<VoxelMap> map_snapshot;
      DynamicTrackSnapshot dynamic_snapshot;
      bool dynamic_obstacle_updated = false;
      std::uint64_t request_generation = 0U;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopping_ || work_pending_; });
        if (stopping_) {
          return;
        }
        work_pending_ = false;
        if (reset_requested_.exchange(false)) {
          fsm_.reset();
          active_trajectory_.reset();
          active_trajectory_safe_ = false;
          pending_request_.reset();
          latest_request_.reset();
          dynamic_snapshot_ = DynamicTrackSnapshot{};
          tracker_->reset();
          has_external_input_ = false;
          has_internal_input_ = false;
          last_external_input_stamp_ = std::numeric_limits<double>::quiet_NaN();
          last_internal_input_stamp_ = std::numeric_limits<double>::quiet_NaN();
          dynamic_obstacle_update_pending_ = false;
          map_information_stale_.store(false);
          map_recovery_pending_ = false;
          map_stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
          map_stale_detail_.clear();
          if (map_freshness_required_) {
            map_observation_available_ = false;
            last_map_observation_stamp_ = std::numeric_limits<double>::quiet_NaN();
          }
          information_stale_.store(false);
          stale_recovery_pending_ = false;
          stale_since_stamp_ = std::numeric_limits<double>::quiet_NaN();
          stale_detail_.clear();
          has_processed_request_ = false;
          has_last_request_planning_stamp_ = false;
          last_request_planning_stamp_ = std::numeric_limits<double>::quiet_NaN();
          publishEmergencyState(false, false, "emergency stop reset",
                                aurora_msgs::msg::EmergencyStopState::UNSPECIFIED);
        }
        if (emergency_stop_latched_.load()) {
          continue;
        }
        request = pending_request_;
        // A request is a one-shot planning trigger. Keeping it in the slot
        // would let a later map or dynamic-obstacle update replay an old
        // request before the caller has submitted a new vehicle state.
        pending_request_.reset();
        current = active_trajectory_;
        map_snapshot = *map_;
        dynamic_snapshot = dynamic_snapshot_;
        dynamic_obstacle_updated = dynamic_obstacle_update_pending_;
        dynamic_obstacle_update_pending_ = false;
        request_generation = reset_generation_.load();
      }
      if (request.has_value() && map_snapshot.has_value()) {
        processRequest(*request, current, *map_snapshot, dynamic_snapshot,
                       dynamic_obstacle_updated, request_generation);
      }
    }
  }

  bool validateActiveTrajectory(const PlannedTrajectory &trajectory, double now,
                                const VoxelMap &map,
                                const DynamicTrackSnapshot &dynamic_snapshot,
                                const aurora::planner::VehicleState &vehicle_state) const {
    if (!trajectory.contains(now) || trajectory.endStamp() <= now) {
      return false;
    }
    try {
      PlannedTrajectory remaining = trajectory;
      remaining.segments = trajectory.slice(now, trajectory.endStamp());
      const auto static_result = safety_gate_->evaluate(map, remaining, now);
      if (!static_result.accepted) {
        RCLCPP_WARN(get_logger(),
                    "active trajectory failed static validation at %.9f: %s (%s)", now,
                    aurora::planner::toString(static_result.status),
                    static_result.detail.c_str());
        return false;
      }
      const auto dynamic_result =
          evaluateDynamicRisk(remaining, dynamic_snapshot, now, vehicle_state, map);
      if (!dynamic_result.accepted) {
        RCLCPP_WARN(get_logger(),
                    "active trajectory failed dynamic validation at %.9f: %s (%s)", now,
                    aurora::risk::toString(dynamic_result.status),
                    dynamic_result.detail.c_str());
      }
      return dynamic_result.accepted;
    } catch (const std::exception &) {
      return false;
    }
  }

  void processRequest(const PlanningRequest &request,
                      const std::optional<PlannedTrajectory> &current,
                      const VoxelMap &map_snapshot,
                      const DynamicTrackSnapshot &dynamic_snapshot,
                      bool dynamic_obstacle_updated,
    std::uint64_t request_generation) {
    if (information_stale_.load() || map_information_stale_.load()) {
      return;
    }
    bool active_available = false;
    bool active_safe = false;
    double active_end_stamp = std::numeric_limits<double>::quiet_NaN();
    if (current.has_value()) {
      active_available = current->contains(request.planning_stamp);
      active_end_stamp = current->endStamp();
      active_safe = validateActiveTrajectory(*current, request.planning_stamp, map_snapshot,
                                             dynamic_snapshot, request.vehicle_state);
    }

    ReplanObservation observation;
    observation.now = request.planning_stamp;
    observation.has_global_reference = request.global_reference.points.size() >= 2U;
    observation.active_trajectory_available = active_available;
    observation.active_trajectory_safe = active_safe;
    observation.active_trajectory_end_stamp = active_end_stamp;
    observation.current_trajectory_collision = active_available && !active_safe;
    observation.dynamic_obstacle_updated = dynamic_obstacle_updated;
    observation.map_updated = current.has_value() && map_snapshot.version() != current->map_version;
    observation.local_goal_expired =
        !has_processed_request_ || request.request_id != processed_request_id_;

    FsmDecision decision = fsm_.step(observation);
    if (decision.action == aurora::planner::PlannerAction::REQUEST_REPLAN) {
      decision = fsm_.step(observation);
    }
    publishPlannerStatus(request, decision, current);
    if (decision.action != aurora::planner::PlannerAction::START_PLANNING) {
      return;
    }

    const auto risk_cost = makeDynamicRiskCostFunction(
        dynamic_snapshot, request.vehicle_state, request.planning_stamp);
    PlanningResult result = planner_->plan(map_snapshot, request, current, risk_cost);
    if (result.status != PlanningStatus::SUCCESS && result.status != PlanningStatus::GOAL_REACHED) {
      RCLCPP_WARN(get_logger(),
                  "planning request %lu failed before ROS gate: %s (%s), reject_unknown=%s, "
                  "map_version=%lu",
                  request.request_id, aurora::planner::toString(result.status),
                  result.detail.c_str(), planner_->options().validation.reject_unknown ? "true" : "false",
                  map_snapshot.version());
    }
    std::optional<StaticSafetyGateResult> gate_result;
    std::optional<DynamicRiskEvaluation> risk_result;
    if (result.trajectory.has_value()) {
      gate_result = safety_gate_->evaluate(map_snapshot, *result.trajectory,
                                           request.planning_stamp, current);
      if (!gate_result->accepted) {
        result.status = PlanningStatus::VALIDATION_FAILED;
        result.detail = std::string("ROS 2 publish gate rejected trajectory with ") +
                        aurora::planner::toString(gate_result->status) + ": " +
                        gate_result->detail;
        result.validation = gate_result->validation;
        result.trajectory.reset();
      } else {
        risk_result = evaluateDynamicRisk(*result.trajectory, dynamic_snapshot,
                                          request.planning_stamp, request.vehicle_state,
                                          map_snapshot);
        if (!risk_result->accepted) {
          result.status = PlanningStatus::VALIDATION_FAILED;
          result.detail = std::string("ROS 2 dynamic risk gate rejected trajectory with ") +
                          aurora::risk::toString(risk_result->status) + ": " +
                          risk_result->detail;
          result.trajectory.reset();
        }
      }
    }

    if (result.status != PlanningStatus::SUCCESS &&
        result.status != PlanningStatus::GOAL_REACHED) {
      RCLCPP_WARN(
          get_logger(),
          "planning request %lu final result: %s (%s), core_validation=%s, "
          "unknown_samples=%zu, gate=%s, risk=%s",
          request.request_id, aurora::planner::toString(result.status), result.detail.c_str(),
          aurora::trajectory::toString(result.validation.status),
          result.validation.unknown_samples,
          gate_result.has_value() ? aurora::planner::toString(gate_result->status) : "NOT_RUN",
          risk_result.has_value() ? aurora::risk::toString(risk_result->status) : "NOT_RUN");
    }

    // A stop or reset may arrive while the core planner is running. Its
    // result is no longer an executable output and must not replace the
    // active trajectory or produce a stale planning result.
    if (emergency_stop_latched_.load() || information_stale_.load() || reset_requested_.load() ||
        reset_generation_.load() != request_generation) {
      return;
    }

    const FsmDecision result_decision = fsm_.onPlanningResult(result, request.planning_stamp);
    if (result.status == PlanningStatus::SUCCESS && result.trajectory.has_value() &&
        gate_result.has_value() && gate_result->accepted && risk_result.has_value() &&
        risk_result->accepted) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        active_trajectory_ = result.trajectory;
        active_trajectory_safe_ = true;
        processed_request_id_ = request.request_id;
        has_processed_request_ = true;
      }
      trajectory_publisher_->publish(toTrajectoryMessage(
          *result.trajectory, *gate_result, *risk_result, request, dynamic_snapshot.has_snapshot,
          map_snapshot.version()));
    } else if (result.status == PlanningStatus::GOAL_REACHED) {
      std::lock_guard<std::mutex> lock(mutex_);
      processed_request_id_ = request.request_id;
      has_processed_request_ = true;
    }
    publishPlanningResult(request, result, gate_result, risk_result, dynamic_snapshot.has_snapshot,
                          map_snapshot.version());
    const auto result_safety_status =
        result.status == PlanningStatus::SUCCESS && result.trajectory.has_value() &&
                gate_result.has_value() && gate_result->accepted && risk_result.has_value() &&
                risk_result->accepted
            ? aurora_msgs::msg::SafetyReport::ACCEPTED
            : safetyStatusForResult(result, risk_result);
    publishPlannerStatus(request, result_decision, result.trajectory, result.status,
                         result_safety_status);
    if (result_decision.state == aurora::planner::PlannerState::EMERGENCY_STOP) {
      emergency_stop_latched_.store(true);
      const auto reason_code = aurora_msgs::msg::EmergencyStopState::SAFETY_GATE;
      publishEmergencyState(true, true, result_decision.detail, reason_code,
                            request.request_id);
    }
  }

  aurora_msgs::msg::Trajectory toTrajectoryMessage(
      const PlannedTrajectory &trajectory, const StaticSafetyGateResult &gate_result,
      const DynamicRiskEvaluation &risk_result, const PlanningRequest &request,
      bool dynamic_snapshot_available, std::uint64_t map_version) const {
    aurora_msgs::msg::Trajectory message;
    message.header.stamp = secondsToTime(request.planning_stamp);
    message.header.frame_id = map_frame_;
    message.trajectory_id = trajectory.trajectory_id;
    message.map_version = map_version;
    message.validation_state = aurora_msgs::msg::Trajectory::VALIDATED;
    message.constraints.max_velocity = planner_->options().validation.max_velocity;
    message.constraints.max_acceleration = planner_->options().validation.max_acceleration;
    message.constraints.observed_max_velocity = gate_result.maximum_velocity;
    message.constraints.observed_max_acceleration = gate_result.maximum_acceleration;
    message.safety_report = toSafetyReport(gate_result, map_version, request.planning_stamp);
    message.risk_report =
        toRiskReport(risk_result, request.planning_stamp, dynamic_snapshot_available);

    message.segments.reserve(trajectory.segments.size());
    for (const auto &segment : trajectory.segments) {
      aurora_msgs::msg::TrajectorySegment segment_message;
      segment_message.start_stamp = secondsToTime(segment.start_stamp);
      segment_message.source_start_time = segment.source_start_time;
      segment_message.duration = segment.duration;
      segment_message.dt = segment.spline.dt();
      segment_message.degree = segment.spline.degree();
      segment_message.knot_mode = segment.spline.knotMode() ==
                                           aurora::math::UniformBsplineKnotMode::CLAMPED
                                       ? aurora_msgs::msg::TrajectorySegment::CLAMPED
                                       : aurora_msgs::msg::TrajectorySegment::EGO_UNCLAMPED;
      const auto &control_points = segment.spline.controlPoints();
      segment_message.control_points.reserve(static_cast<std::size_t>(control_points.cols()));
      for (int index = 0; index < control_points.cols(); ++index) {
        segment_message.control_points.push_back(toPoint(control_points.col(index)));
      }
      message.segments.push_back(std::move(segment_message));
    }
    return message;
  }

  static aurora_msgs::msg::SafetyReport toSafetyReport(const StaticSafetyGateResult &result,
                                                       std::uint64_t map_version,
                                                       double stamp) {
    aurora_msgs::msg::SafetyReport message;
    message.stamp = secondsToTime(stamp);
    message.map_version = map_version;
    message.status = static_cast<std::uint8_t>(result.status);
    message.accepted = result.accepted;
    message.failed_segment = result.failed_segment == std::numeric_limits<std::size_t>::max()
                                 ? -1
                                 : static_cast<std::int64_t>(result.failed_segment);
    message.checked_segments = result.checked_segments;
    message.checked_samples = result.checked_samples;
    message.occupied_samples = result.occupied_samples;
    message.unknown_samples = result.unknown_samples;
    message.maximum_velocity = result.maximum_velocity;
    message.maximum_acceleration = result.maximum_acceleration;
    message.candidate_start_stamp = secondsToTime(result.candidate_start_stamp);
    message.candidate_end_stamp = secondsToTime(result.candidate_end_stamp);
    message.current_trajectory_available = result.current_trajectory_available;
    message.current_state_checked = result.current_state_checked;
    message.current_state_continuous = result.current_state_continuous;
    message.current_trajectory_fallback_available = result.current_trajectory_fallback_available;
    message.detail = result.detail;
    return message;
  }

  static aurora_msgs::msg::RiskReport toRiskReport(const DynamicRiskEvaluation &result,
                                                    double stamp, bool snapshot_available) {
    aurora_msgs::msg::RiskReport message;
    const bool information_stale =
        !snapshot_available || result.status == RiskStatus::NO_DYNAMIC_INFORMATION ||
        result.status == RiskStatus::INFORMATION_STALE;
    const bool information_unusable =
        information_stale || result.status == RiskStatus::INVALID_OPTIONS ||
        result.status == RiskStatus::INVALID_INPUT ||
        result.status == RiskStatus::PREDICTION_INVALID ||
        result.status == RiskStatus::NO_MAP_INFORMATION;
    message.stamp = secondsToTime(stamp);
    message.model_id = "aurora.conservative_3sigma_v1";
    message.dynamic_information_available = snapshot_available;
    message.dynamic_information_stale = information_stale;
    message.dynamic_risk = std::isfinite(result.dynamic_risk) ? result.dynamic_risk : 0.0;
    message.information_risk = information_unusable
                                   ? 1.0
                                   : (std::isfinite(result.information_risk)
                                          ? result.information_risk
                                          : 0.0);
    message.tracking_risk = 0.0;
    message.static_risk = std::isfinite(result.map_risk) ? result.map_risk : 0.0;
    message.information_age = std::isfinite(result.information_age)
                                  ? result.information_age
                                  : -1.0;
    message.total_risk = std::max({message.static_risk, message.dynamic_risk,
                                   message.information_risk});
    message.minimum_clearance = result.minimum_clearance;
    message.worst_obstacle_id = result.worst_obstacle_id;
    message.worst_time = secondsToTime(result.worst_time);
    switch (result.level) {
      case aurora::risk::RiskLevel::LOW:
        message.risk_level = aurora_msgs::msg::RiskReport::LOW;
        break;
      case aurora::risk::RiskLevel::MEDIUM:
        message.risk_level = aurora_msgs::msg::RiskReport::MEDIUM;
        break;
      case aurora::risk::RiskLevel::HIGH:
        message.risk_level = aurora_msgs::msg::RiskReport::HIGH;
        break;
      case aurora::risk::RiskLevel::UNKNOWN:
        message.risk_level = aurora_msgs::msg::RiskReport::UNKNOWN;
        break;
    }
    message.detail = result.detail;
    return message;
  }

  static aurora_msgs::msg::RiskReport notEvaluatedRiskReport(double stamp,
                                                              const std::string &detail) {
    aurora_msgs::msg::RiskReport message;
    message.stamp = secondsToTime(stamp);
    message.model_id = "aurora.conservative_3sigma_v1";
    message.risk_level = aurora_msgs::msg::RiskReport::UNKNOWN;
    message.information_age = -1.0;
    message.dynamic_information_stale = true;
    message.information_risk = 1.0;
    message.total_risk = 1.0;
    message.detail = detail;
    return message;
  }

  static std::uint8_t safetyStatusForResult(
      const PlanningResult &result,
      const std::optional<DynamicRiskEvaluation> &risk_result = std::nullopt) {
    if (risk_result.has_value() && !risk_result->accepted) {
      switch (risk_result->status) {
        case RiskStatus::DYNAMIC_COLLISION:
          return aurora_msgs::msg::SafetyReport::DYNAMIC_COLLISION;
        case RiskStatus::RISK_LIMIT:
        case RiskStatus::MAP_RISK:
          return aurora_msgs::msg::SafetyReport::RISK_LIMIT;
        case RiskStatus::MAP_UNKNOWN:
          return aurora_msgs::msg::SafetyReport::UNKNOWN_SPACE;
        case RiskStatus::MAP_OUT_OF_MAP:
          return aurora_msgs::msg::SafetyReport::OUT_OF_MAP;
        case RiskStatus::MAP_COLLISION:
          return aurora_msgs::msg::SafetyReport::STATIC_COLLISION;
        case RiskStatus::NO_MAP_INFORMATION:
          return aurora_msgs::msg::SafetyReport::INFORMATION_STALE;
        case RiskStatus::NO_DYNAMIC_INFORMATION:
        case RiskStatus::INFORMATION_STALE:
          return aurora_msgs::msg::SafetyReport::INFORMATION_STALE;
        case RiskStatus::INVALID_OPTIONS:
        case RiskStatus::INVALID_INPUT:
        case RiskStatus::PREDICTION_INVALID:
          return aurora_msgs::msg::SafetyReport::INVALID_INPUT;
        case RiskStatus::ACCEPTED:
          break;
      }
    }
    if (!result.trajectory.has_value() && result.validation.status ==
                                             aurora::trajectory::ValidationStatus::VALID) {
      return result.status == PlanningStatus::GOAL_REACHED
                 ? aurora_msgs::msg::SafetyReport::EMPTY_CANDIDATE
                 : aurora_msgs::msg::SafetyReport::INVALID_INPUT;
    }
    switch (result.validation.status) {
      case aurora::trajectory::ValidationStatus::VALID:
        return aurora_msgs::msg::SafetyReport::INVALID_INPUT;
      case aurora::trajectory::ValidationStatus::INVALID_OPTIONS:
        return aurora_msgs::msg::SafetyReport::INVALID_OPTIONS;
      case aurora::trajectory::ValidationStatus::NONFINITE:
        return aurora_msgs::msg::SafetyReport::INVALID_SEGMENT;
      case aurora::trajectory::ValidationStatus::OUT_OF_MAP:
        return aurora_msgs::msg::SafetyReport::OUT_OF_MAP;
      case aurora::trajectory::ValidationStatus::OCCUPIED:
        return aurora_msgs::msg::SafetyReport::STATIC_COLLISION;
      case aurora::trajectory::ValidationStatus::UNKNOWN:
        return aurora_msgs::msg::SafetyReport::UNKNOWN_SPACE;
      case aurora::trajectory::ValidationStatus::VELOCITY_LIMIT:
        return aurora_msgs::msg::SafetyReport::VELOCITY_LIMIT;
      case aurora::trajectory::ValidationStatus::ACCELERATION_LIMIT:
        return aurora_msgs::msg::SafetyReport::ACCELERATION_LIMIT;
    }
    return aurora_msgs::msg::SafetyReport::INVALID_INPUT;
  }

  void publishPlanningResult(const PlanningRequest &request, const PlanningResult &result,
                             const std::optional<StaticSafetyGateResult> &gate_result,
                             const std::optional<DynamicRiskEvaluation> &risk_result,
                             bool dynamic_snapshot_available,
                             std::uint64_t map_version) {
    aurora_msgs::msg::PlanningResult message;
    message.header.stamp = secondsToTime(request.planning_stamp);
    message.header.frame_id = map_frame_;
    message.request_id = request.request_id;
    message.status = static_cast<std::uint8_t>(result.status);
    message.detail = result.detail;
    message.has_trajectory = result.trajectory.has_value();
    message.risk_report = risk_result.has_value()
                              ? toRiskReport(*risk_result, request.planning_stamp,
                                             dynamic_snapshot_available)
                              : notEvaluatedRiskReport(request.planning_stamp,
                                                       "dynamic risk was not evaluated");
    if (result.trajectory.has_value() && gate_result.has_value() && risk_result.has_value() &&
        gate_result->accepted && risk_result->accepted) {
      message.trajectory = toTrajectoryMessage(
          *result.trajectory, *gate_result, *risk_result, request, dynamic_snapshot_available,
          map_version);
      message.safety_report = toSafetyReport(*gate_result, map_version, request.planning_stamp);
    } else {
      if (gate_result.has_value()) {
        message.safety_report =
            toSafetyReport(*gate_result, map_version, request.planning_stamp);
      } else {
        message.safety_report.stamp = secondsToTime(request.planning_stamp);
        message.safety_report.map_version = map_version;
      }
      message.safety_report.status = safetyStatusForResult(result, risk_result);
      message.safety_report.accepted = false;
      message.safety_report.detail = result.detail;
    }
    planning_result_publisher_->publish(std::move(message));
  }

  void publishInvalidRequest(const std_msgs::msg::Header &header, std::uint64_t request_id,
                             const std::string &detail) {
    aurora_msgs::msg::PlanningResult message;
    message.header = header;
    message.request_id = request_id;
    message.status = aurora_msgs::msg::PlanningResult::INVALID_REQUEST;
    message.detail = detail;
    message.has_trajectory = false;
    message.risk_report = notEvaluatedRiskReport(timeToSeconds(header.stamp),
                                                 "dynamic risk was not evaluated");
    message.safety_report.status = aurora_msgs::msg::SafetyReport::INVALID_INPUT;
    message.safety_report.accepted = false;
    message.safety_report.detail = detail;
    planning_result_publisher_->publish(std::move(message));
  }

  void publishPlannerStatus(const PlanningRequest &request, const FsmDecision &decision,
                            const std::optional<PlannedTrajectory> &trajectory,
                            std::optional<PlanningStatus> planning_status = std::nullopt,
                            std::optional<std::uint8_t> safety_status = std::nullopt) {
    aurora_msgs::msg::PlannerStatus message;
    message.header.stamp = secondsToTime(request.planning_stamp);
    message.header.frame_id = map_frame_;
    message.request_id = request.request_id;
    message.trajectory_id = trajectory.has_value() ? trajectory->trajectory_id : 0U;
    message.planner_state = static_cast<std::uint8_t>(decision.state);
    message.planner_action = static_cast<std::uint8_t>(decision.action);
    message.replan_trigger = static_cast<std::uint8_t>(decision.trigger);
    message.planning_status = static_cast<std::uint8_t>(
        planning_status.value_or(trajectory.has_value() ? PlanningStatus::SUCCESS
                                                        : PlanningStatus::INVALID_REQUEST));
    message.safety_status = safety_status.value_or(
        trajectory.has_value() ? aurora_msgs::msg::SafetyReport::ACCEPTED
                               : aurora_msgs::msg::SafetyReport::INVALID_INPUT);
    message.consecutive_failures = static_cast<std::uint32_t>(decision.consecutive_failures);
    message.valid = decision.valid;
    message.detail = decision.detail;
    planner_status_publisher_->publish(std::move(message));
  }

  void publishEmergencyState(bool active, bool latched, const std::string &detail,
                             std::uint8_t reason_code, std::uint64_t request_id = 0U) {
    if (!emergency_state_publisher_) {
      return;
    }
    aurora_msgs::msg::EmergencyStopState message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.active = active;
    message.latched = latched;
    message.reason = latched ? reason_code : aurora_msgs::msg::EmergencyStopState::UNSPECIFIED;
    message.request_id = request_id;
    message.detail = detail;
    emergency_state_publisher_->publish(std::move(message));
  }

  std::string map_frame_;
  std::string pointcloud_topic_;
  std::string depth_image_topic_;
  std::string camera_info_topic_;
  std::string planning_request_topic_;
  std::string trajectory_topic_;
  std::string planning_result_topic_;
  std::string planner_status_topic_;
  std::string emergency_state_topic_;
  std::string dynamic_obstacle_topic_;
  std::string dynamic_detection_topic_;
  std::string vehicle_state_topic_;
  std::string execution_status_topic_;
  std::string emergency_service_;

  DynamicInputMode dynamic_input_mode_{DynamicInputMode::EXTERNAL_TRACKS};

  double vehicle_radius_{0.65};
  double map_inflation_radius_{0.0};
  std::int64_t pointcloud_max_points_{100000};
  double pointcloud_max_range_{30.0};
  double pointcloud_confidence_{1.0};
  bool depth_image_enabled_{false};
  double depth_min_range_{0.1};
  double depth_max_range_{30.0};
  double depth_confidence_{1.0};
  double tf_timeout_sec_{0.05};
  bool map_freshness_required_{true};
  double map_max_observation_age_{1.0};
  double risk_sample_interval_{0.1};
  double execution_position_variance_{0.0};
  bool risk_map_quality_enabled_{false};
  bool risk_soft_cost_enabled_{true};
  DynamicRiskCostFieldOptions soft_risk_options_;
  aurora::risk::DelayBudget risk_delay_;
  double stale_hold_duration_{0.5};
  double information_watchdog_rate_hz_{10.0};

  std::unique_ptr<VoxelMap> map_;
  std::unique_ptr<StaticLocalPlanner> planner_;
  std::unique_ptr<StaticSafetyGate> safety_gate_;
  std::unique_ptr<KinematicPredictor> predictor_;
  std::unique_ptr<DynamicRiskEvaluator> risk_evaluator_;
  std::unique_ptr<DynamicObstacleAdapter> dynamic_obstacle_adapter_;
  std::unique_ptr<DepthImageAdapter> depth_adapter_;
  std::unique_ptr<UnassociatedObstacleAdapter> unassociated_obstacle_adapter_;
  std::unique_ptr<ObstacleTracker> tracker_;
  std::unique_ptr<aurora::flight::TrajectoryAdmission> flight_admission_;
  StaticReplanFsm fsm_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<aurora_msgs::msg::PlanningRequest>::SharedPtr
      planning_request_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<aurora_msgs::msg::DynamicObstacleTrackArray>::SharedPtr
      dynamic_obstacle_subscription_;
  rclcpp::Subscription<aurora_msgs::msg::UnassociatedObstacleDetectionArray>::SharedPtr
      dynamic_detection_subscription_;
  rclcpp::Subscription<aurora_msgs::msg::VehicleState>::SharedPtr
      execution_vehicle_state_subscription_;
  rclcpp::Subscription<aurora_msgs::msg::TrajectoryExecutionStatus>::SharedPtr
      execution_status_subscription_;
  rclcpp::Publisher<aurora_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<aurora_msgs::msg::PlanningResult>::SharedPtr planning_result_publisher_;
  rclcpp::Publisher<aurora_msgs::msg::PlannerStatus>::SharedPtr planner_status_publisher_;
  rclcpp::Publisher<aurora_msgs::msg::EmergencyStopState>::SharedPtr emergency_state_publisher_;
  rclcpp::Service<aurora_msgs::srv::SetEmergencyStop>::SharedPtr emergency_service_server_;
  rclcpp::TimerBase::SharedPtr information_watchdog_timer_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread planner_thread_;
  bool stopping_{false};
  bool work_pending_{false};
  std::optional<PlanningRequest> pending_request_;
  std::optional<PlanningRequest> latest_request_;
  std::optional<PlannedTrajectory> active_trajectory_;
  DynamicTrackSnapshot dynamic_snapshot_;
  DynamicInputSource active_dynamic_source_{DynamicInputSource::EXTERNAL_TRACKS};
  bool has_external_input_{false};
  bool has_internal_input_{false};
  double last_external_input_stamp_{std::numeric_limits<double>::quiet_NaN()};
  double last_internal_input_stamp_{std::numeric_limits<double>::quiet_NaN()};
  bool dynamic_obstacle_update_pending_{false};
  bool active_trajectory_safe_{false};
  std::atomic_bool map_information_stale_{false};
  bool map_recovery_pending_{false};
  double map_stale_since_stamp_{std::numeric_limits<double>::quiet_NaN()};
  std::string map_stale_detail_;
  bool map_observation_available_{false};
  double last_map_observation_stamp_{std::numeric_limits<double>::quiet_NaN()};
  std::optional<sensor_msgs::msg::CameraInfo> latest_camera_info_;
  double latest_camera_info_stamp_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic_bool information_stale_{false};
  bool stale_recovery_pending_{false};
  double stale_since_stamp_{std::numeric_limits<double>::quiet_NaN()};
  std::string stale_detail_;
  std::uint64_t processed_request_id_{0};
  bool has_processed_request_{false};
  double last_request_planning_stamp_{std::numeric_limits<double>::quiet_NaN()};
  bool has_last_request_planning_stamp_{false};
  Eigen::Vector3d latest_vehicle_position_{Eigen::Vector3d::Zero()};
  double latest_vehicle_stamp_{0.0};
  bool has_latest_vehicle_state_{false};
  aurora::planner::VehicleState latest_execution_state_;
  bool has_execution_state_{false};
  std::uint64_t next_internal_request_id_{1U};
  std::string emergency_reason_;
  std::atomic_bool emergency_stop_latched_{false};
  std::atomic_bool reset_requested_{false};
  std::atomic<std::uint64_t> reset_generation_{0U};
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AuroraPlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception &error) {
    fprintf(stderr, "aurora_planner_node failed to start: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
