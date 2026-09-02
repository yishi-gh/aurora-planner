#include "aurora_risk/dynamic_risk_evaluator.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace aurora::risk {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kMinimumExistenceProbability = 0.05;

bool finite(double value) { return std::isfinite(value); }

struct InterpolatedPrediction {
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Covariance3 covariance{Covariance3::Zero()};
  double existence_probability{0.0};
  aurora::prediction::ObstacleShape shape;
  std::string detail;
};

bool covarianceIsValid(const Covariance3 &covariance, double tolerance) {
  if (!covariance.allFinite()) {
    return false;
  }
  if ((covariance - covariance.transpose()).cwiseAbs().maxCoeff() > tolerance) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Covariance3> solver(
      0.5 * (covariance + covariance.transpose()));
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -tolerance;
}

double obstacleRadius(const aurora::prediction::ObstacleShape &shape) {
  if (!shape.dimensions.allFinite() || !finite(shape.radius) || shape.radius < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  switch (shape.type) {
    case aurora::prediction::ShapeType::SPHERE:
      return shape.radius;
    case aurora::prediction::ShapeType::BOX:
      if ((shape.dimensions.array() <= 0.0).any()) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return 0.5 * shape.dimensions.norm();
    case aurora::prediction::ShapeType::CAPSULE:
      if ((shape.dimensions.array() < 0.0).any() || shape.dimensions.norm() <= kEpsilon) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return shape.radius + 0.5 * shape.dimensions.norm();
    case aurora::prediction::ShapeType::MULTI_SPHERE:
      if (shape.dimensions.norm() <= kEpsilon && shape.radius <= kEpsilon) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return std::max(shape.radius, 0.5 * shape.dimensions.norm());
  }
  return std::numeric_limits<double>::quiet_NaN();
}

InterpolatedPrediction interpolate(const aurora::prediction::PredictionResult &prediction,
                                   double stamp, double tolerance) {
  InterpolatedPrediction result;
  if (prediction.status != aurora::prediction::PredictionStatus::SUCCESS ||
      prediction.states.empty() || !finite(stamp)) {
    result.detail = "prediction result is not successful or contains no states";
    return result;
  }
  if (!finite(prediction.reference_stamp) ||
      std::abs(prediction.states.front().stamp - prediction.reference_stamp) > tolerance) {
    result.detail = "prediction reference time does not match its first state";
    return result;
  }
  if (stamp < prediction.states.front().stamp - tolerance ||
      stamp > prediction.states.back().stamp + tolerance) {
    result.detail = "prediction does not cover the trajectory sample time";
    return result;
  }

  std::size_t upper_index = 0U;
  while (upper_index < prediction.states.size() &&
         prediction.states[upper_index].stamp < stamp - tolerance) {
    ++upper_index;
  }
  if (upper_index == 0U || upper_index >= prediction.states.size()) {
    upper_index = upper_index == 0U ? 0U : prediction.states.size() - 1U;
  }
  const auto &upper = prediction.states[upper_index];
  if (upper_index == 0U || std::abs(upper.stamp - stamp) <= tolerance) {
    result.position = upper.position;
    result.covariance = upper.covariance.block<3, 3>(0, 0);
    result.existence_probability = upper.existence_probability;
    result.shape = upper.shape;
  } else {
    const auto &lower = prediction.states[upper_index - 1U];
    const double interval = upper.stamp - lower.stamp;
    if (!finite(interval) || interval <= tolerance) {
      result.detail = "prediction states are not strictly time ordered";
      return result;
    }
    const double alpha = std::clamp((stamp - lower.stamp) / interval, 0.0, 1.0);
    result.position = (1.0 - alpha) * lower.position + alpha * upper.position;
    result.covariance = (1.0 - alpha) * lower.covariance.block<3, 3>(0, 0) +
                        alpha * upper.covariance.block<3, 3>(0, 0);
    result.existence_probability = (1.0 - alpha) * lower.existence_probability +
                                   alpha * upper.existence_probability;
    const double lower_radius = obstacleRadius(lower.shape);
    const double upper_radius = obstacleRadius(upper.shape);
    if (!finite(lower_radius) || !finite(upper_radius)) {
      result.detail = "prediction state contains an invalid obstacle shape";
      return result;
    }
    // Shape changes are not expected for a single track, but selecting the
    // larger equivalent sphere keeps interpolation conservative if they occur.
    result.shape = lower_radius >= upper_radius ? lower.shape : upper.shape;
  }
  if (!result.position.allFinite() || !covarianceIsValid(result.covariance, tolerance) ||
      !finite(result.existence_probability) || result.existence_probability < 0.0 ||
      result.existence_probability > 1.0) {
    result.detail = "interpolated prediction state is invalid";
    return result;
  }
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
  result.valid = true;
  return result;
}

void setFailure(DynamicRiskEvaluation *result, RiskStatus status, const std::string &detail) {
  result->status = status;
  result->level = status == RiskStatus::DYNAMIC_COLLISION || status == RiskStatus::RISK_LIMIT ||
                          status == RiskStatus::MAP_COLLISION ||
                          status == RiskStatus::MAP_OUT_OF_MAP ||
                          status == RiskStatus::MAP_UNKNOWN || status == RiskStatus::MAP_RISK
                      ? RiskLevel::HIGH
                      : RiskLevel::UNKNOWN;
  result->accepted = false;
  result->detail = detail;
}

double effectivePredictionStamp(double trajectory_stamp, const DelayBudget &delay) {
  if (!finite(trajectory_stamp) || !finite(delay.total())) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double effective_stamp = trajectory_stamp + delay.total();
  return finite(effective_stamp) ? effective_stamp
                                 : std::numeric_limits<double>::quiet_NaN();
}

Covariance3 vehiclePositionCovariance(const TrajectorySample &sample,
                                      const VehicleUncertaintyInput &vehicle) {
  Covariance3 covariance = sample.position_covariance;
  if (vehicle.has_localization_position_covariance) {
    covariance += vehicle.localization_position_covariance;
  }
  if (vehicle.has_execution_position_covariance) {
    covariance += vehicle.execution_position_covariance;
  }
  return 0.5 * (covariance + covariance.transpose());
}

}  // namespace

const char *toString(RiskStatus status) noexcept {
  switch (status) {
    case RiskStatus::ACCEPTED:
      return "ACCEPTED";
    case RiskStatus::INVALID_OPTIONS:
      return "INVALID_OPTIONS";
    case RiskStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case RiskStatus::NO_DYNAMIC_INFORMATION:
      return "NO_DYNAMIC_INFORMATION";
    case RiskStatus::INFORMATION_STALE:
      return "INFORMATION_STALE";
    case RiskStatus::PREDICTION_INVALID:
      return "PREDICTION_INVALID";
    case RiskStatus::NO_MAP_INFORMATION:
      return "NO_MAP_INFORMATION";
    case RiskStatus::MAP_UNKNOWN:
      return "MAP_UNKNOWN";
    case RiskStatus::MAP_OUT_OF_MAP:
      return "MAP_OUT_OF_MAP";
    case RiskStatus::MAP_COLLISION:
      return "MAP_COLLISION";
    case RiskStatus::MAP_RISK:
      return "MAP_RISK";
    case RiskStatus::DYNAMIC_COLLISION:
      return "DYNAMIC_COLLISION";
    case RiskStatus::RISK_LIMIT:
      return "RISK_LIMIT";
  }
  return "UNKNOWN_STATUS";
}

const char *toString(RiskLevel level) noexcept {
  switch (level) {
    case RiskLevel::LOW:
      return "LOW";
    case RiskLevel::MEDIUM:
      return "MEDIUM";
    case RiskLevel::HIGH:
      return "HIGH";
    case RiskLevel::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNKNOWN_LEVEL";
}

void DynamicRiskEvaluator::validateOptions(const DynamicRiskEvaluatorOptions &options) {
  if (!finite(options.vehicle_radius) || options.vehicle_radius < 0.0 ||
      !finite(options.sigma_multiplier) || options.sigma_multiplier <= 0.0 ||
      !finite(options.time_tolerance) || options.time_tolerance < 0.0 ||
      !finite(options.max_prediction_age) || options.max_prediction_age < 0.0 ||
      !finite(options.warning_clearance) || options.warning_clearance <= 0.0 ||
      !finite(options.map_free_probability) || options.map_free_probability < 0.0 ||
      !finite(options.map_occupancy_threshold) ||
      options.map_occupancy_threshold <= options.map_free_probability ||
      options.map_occupancy_threshold > 1.0 ||
      !finite(options.map_max_observation_age) || options.map_max_observation_age <= 0.0 ||
      !finite(options.map_occupancy_weight) || options.map_occupancy_weight < 0.0 ||
      !finite(options.map_age_weight) || options.map_age_weight < 0.0 ||
      !finite(options.map_confidence_weight) || options.map_confidence_weight < 0.0 ||
      options.map_occupancy_weight + options.map_age_weight +
              options.map_confidence_weight <= 0.0 ||
      !finite(options.map_risk_limit) || options.map_risk_limit < 0.0 ||
      options.map_risk_limit > 1.0 ||
      options.max_samples == 0U || options.max_obstacles == 0U) {
    throw std::invalid_argument("invalid dynamic risk evaluator options");
  }
}

DynamicRiskEvaluator::DynamicRiskEvaluator(DynamicRiskEvaluatorOptions options)
    : options_(std::move(options)) {
  validateOptions(options_);
}

DynamicRiskEvaluation DynamicRiskEvaluator::evaluate(
    const std::vector<TrajectorySample> &trajectory, const DynamicRiskInput &input) const {
  DynamicRiskEvaluation result;
  if (trajectory.empty() || trajectory.size() > options_.max_samples) {
    setFailure(&result, RiskStatus::INVALID_INPUT,
               "trajectory sample count is empty or exceeds the configured maximum");
    return result;
  }
  double previous_stamp = -std::numeric_limits<double>::infinity();
  for (const auto &sample : trajectory) {
    if (!finite(sample.stamp) || !sample.position.allFinite() ||
        !covarianceIsValid(sample.position_covariance, options_.time_tolerance) ||
        (finite(previous_stamp) && sample.stamp <= previous_stamp + options_.time_tolerance)) {
      setFailure(&result, RiskStatus::INVALID_INPUT,
                 "trajectory samples must be finite, ordered and covariance-valid");
      return result;
    }
    previous_stamp = sample.stamp;
  }

  const auto context_validation =
      validateRiskContext(input.context, options_.time_tolerance);
  if (!context_validation.valid) {
    setFailure(&result, RiskStatus::INVALID_INPUT,
               "invalid risk context: " + context_validation.detail);
    return result;
  }

  if (options_.require_map_quality && !input.context.map.available) {
    setFailure(&result, RiskStatus::NO_MAP_INFORMATION,
               "map quality snapshot is required but not available");
    return result;
  }
  if (input.context.map.available) {
    if (input.context.map.samples.size() != trajectory.size()) {
      setFailure(&result, RiskStatus::INVALID_INPUT,
                 "map quality samples must align one-to-one with trajectory samples");
      return result;
    }
    const double weight_sum = options_.map_occupancy_weight + options_.map_age_weight +
                              options_.map_confidence_weight;
    result.map_risk = 0.0;
    for (std::size_t index = 0U; index < trajectory.size(); ++index) {
      const auto &trajectory_sample = trajectory[index];
      const auto &map_sample = input.context.map.samples[index];
      if (std::abs(map_sample.stamp - trajectory_sample.stamp) > options_.time_tolerance ||
          (map_sample.position - trajectory_sample.position).norm() > options_.time_tolerance) {
        setFailure(&result, RiskStatus::INVALID_INPUT,
                   "map quality samples are not aligned with trajectory samples");
        return result;
      }
      if (map_sample.state == MapRiskState::OCCUPIED || map_sample.inflated) {
        result.map_risk = 1.0;
        result.total_risk = 1.0;
        setFailure(&result, RiskStatus::MAP_COLLISION,
                   "trajectory enters an occupied or inflated map voxel");
        return result;
      }
      if (map_sample.state == MapRiskState::OUT_OF_MAP) {
        result.map_risk = 1.0;
        result.total_risk = 1.0;
        setFailure(&result, RiskStatus::MAP_OUT_OF_MAP,
                   "trajectory leaves the map bounds");
        return result;
      }
      if (map_sample.state == MapRiskState::UNKNOWN && !options_.allow_unknown_space) {
        result.map_risk = 1.0;
        result.total_risk = 1.0;
        setFailure(&result, RiskStatus::MAP_UNKNOWN,
                   "trajectory enters unknown map space");
        return result;
      }
      if (map_sample.state == MapRiskState::UNKNOWN) {
        result.map_risk = std::max(result.map_risk, 1.0);
        continue;
      }
      const double occupancy_risk = std::clamp(
          (map_sample.occupancy_probability - options_.map_free_probability) /
              (options_.map_occupancy_threshold - options_.map_free_probability),
          0.0, 1.0);
      const double age_risk = std::clamp(
          map_sample.observation_age / options_.map_max_observation_age, 0.0, 1.0);
      const double confidence_risk = 1.0 - map_sample.confidence;
      const double quality_risk =
          (options_.map_occupancy_weight * occupancy_risk +
           options_.map_age_weight * age_risk +
           options_.map_confidence_weight * confidence_risk) /
          weight_sum;
      result.map_risk = std::max(result.map_risk, quality_risk);
    }
    result.map_risk = std::clamp(result.map_risk, 0.0, 1.0);
    if (result.map_risk > options_.map_risk_limit + options_.time_tolerance) {
      result.total_risk = result.map_risk;
      setFailure(&result, RiskStatus::MAP_RISK,
                 "map quality risk exceeds the configured limit");
      return result;
    }
  } else {
    result.map_risk = 0.0;
  }

  if (!input.has_snapshot) {
    if (options_.require_dynamic_information) {
      setFailure(&result, RiskStatus::NO_DYNAMIC_INFORMATION,
                 "dynamic obstacle snapshot is not available");
      return result;
    }
    result.status = RiskStatus::ACCEPTED;
    result.level = RiskLevel::UNKNOWN;
    result.accepted = true;
    result.total_risk = 0.0;
    result.dynamic_risk = 0.0;
    result.minimum_clearance = std::numeric_limits<double>::infinity();
    result.total_risk = std::max(result.map_risk, result.dynamic_risk);
    result.detail = "dynamic risk is disabled because no snapshot is required";
    return result;
  }
  if (!finite(input.snapshot_stamp)) {
    setFailure(&result, RiskStatus::INVALID_INPUT,
               "dynamic obstacle snapshot time is not finite");
    return result;
  }
  const double effective_trajectory_start =
      effectivePredictionStamp(trajectory.front().stamp, input.context.delay);
  if (!finite(effective_trajectory_start)) {
    setFailure(&result, RiskStatus::INVALID_INPUT,
               "effective prediction start time is not finite");
    return result;
  }
  if (finite(input.evaluation_stamp)) {
    if (input.evaluation_stamp < input.snapshot_stamp - options_.time_tolerance) {
      setFailure(&result, RiskStatus::INFORMATION_STALE,
                 "dynamic obstacle snapshot is newer than the evaluation time");
      return result;
    }
    result.information_age = std::max(0.0, input.evaluation_stamp - input.snapshot_stamp);
    if (result.information_age > options_.max_prediction_age + options_.time_tolerance) {
      setFailure(&result, RiskStatus::INFORMATION_STALE,
                 "dynamic obstacle snapshot exceeded the maximum information age");
      return result;
    }
  } else {
    result.information_age = std::max(0.0, effective_trajectory_start - input.snapshot_stamp);
  }
  if (effective_trajectory_start < input.snapshot_stamp - options_.time_tolerance ||
      effective_trajectory_start - input.snapshot_stamp > options_.max_prediction_age +
                                                             options_.time_tolerance) {
    setFailure(&result, RiskStatus::INFORMATION_STALE,
               "dynamic obstacle snapshot is not aligned with the trajectory start time");
    return result;
  }
  if (input.occlusion_active || input.occluded_track_count > 0U) {
    setFailure(&result, RiskStatus::INFORMATION_STALE,
               "dynamic obstacle information is explicitly marked occluded");
    return result;
  }
  if (input.invalid_track_count > 0U) {
    setFailure(&result, RiskStatus::INFORMATION_STALE,
               "dynamic obstacle snapshot contains invalid tracks");
    return result;
  }
  if (input.predictions.size() > options_.max_obstacles) {
    setFailure(&result, RiskStatus::INVALID_INPUT,
               "dynamic obstacle count exceeds the configured maximum");
    return result;
  }
  if (options_.require_dynamic_information && input.predictions.empty()) {
    // An explicitly received empty batch is a valid no-target heartbeat. It
    // carries information only when the caller has provided that snapshot.
    result.status = RiskStatus::ACCEPTED;
    result.level = RiskLevel::LOW;
    result.accepted = true;
    result.total_risk = 0.0;
    result.dynamic_risk = 0.0;
    result.minimum_clearance = std::numeric_limits<double>::infinity();
    result.total_risk = std::max(result.map_risk, result.dynamic_risk);
    result.level = result.total_risk > 0.5 ? RiskLevel::MEDIUM : RiskLevel::LOW;
    result.detail = "dynamic snapshot is valid and contains no tracked obstacles";
    result.checked_samples = trajectory.size();
    return result;
  }

  for (const auto &prediction : input.predictions) {
    if (prediction.status != aurora::prediction::PredictionStatus::SUCCESS) {
      setFailure(&result, RiskStatus::PREDICTION_INVALID,
                 "dynamic prediction failed before risk evaluation");
      return result;
    }
    if (!finite(prediction.reference_stamp) ||
        prediction.reference_stamp > input.snapshot_stamp + options_.time_tolerance ||
        prediction.states.empty() ||
        effective_trajectory_start < prediction.reference_stamp - options_.time_tolerance ||
        effective_trajectory_start - prediction.reference_stamp > options_.max_prediction_age +
                                                                       options_.time_tolerance) {
      setFailure(&result, RiskStatus::INFORMATION_STALE,
                 "dynamic prediction is stale or not aligned with the snapshot time");
      return result;
    }
    double previous_prediction_stamp = -std::numeric_limits<double>::infinity();
    for (const auto &state : prediction.states) {
      if (!finite(state.stamp) ||
          (finite(previous_prediction_stamp) &&
           state.stamp <= previous_prediction_stamp + options_.time_tolerance) ||
          !state.position.allFinite() || !state.covariance.allFinite() ||
          !covarianceIsValid(state.covariance.block<3, 3>(0, 0), options_.time_tolerance) ||
          !finite(state.existence_probability) || state.existence_probability < 0.0 ||
          state.existence_probability > 1.0 || !finite(state.mode_probability) ||
          state.mode_probability < 0.0 || state.mode_probability > 1.0 ||
          !finite(obstacleRadius(state.shape))) {
        setFailure(&result, RiskStatus::PREDICTION_INVALID,
                   "dynamic prediction contains an invalid or unordered state");
        return result;
      }
      previous_prediction_stamp = state.stamp;
    }
    if (std::abs(prediction.states.front().stamp - prediction.reference_stamp) >
        options_.time_tolerance) {
      setFailure(&result, RiskStatus::PREDICTION_INVALID,
                 "prediction reference time does not match its first state");
      return result;
    }
  }

  result.minimum_clearance = std::numeric_limits<double>::infinity();
  double maximum_normalized_risk = 0.0;
  for (const auto &sample : trajectory) {
    ++result.checked_samples;
    const double effective_stamp =
        effectivePredictionStamp(sample.stamp, input.context.delay);
    if (!finite(effective_stamp)) {
      setFailure(&result, RiskStatus::INVALID_INPUT,
                 "effective prediction sample time is not finite");
      return result;
    }
    const Covariance3 vehicle_covariance =
        vehiclePositionCovariance(sample, input.context.vehicle);
    for (const auto &prediction : input.predictions) {
      ++result.checked_obstacles;
      const auto interpolated = interpolate(prediction, effective_stamp, options_.time_tolerance);
      if (!interpolated.valid) {
        setFailure(&result, RiskStatus::PREDICTION_INVALID, interpolated.detail);
        return result;
      }
      const double radius = obstacleRadius(interpolated.shape);
      if (!finite(radius)) {
        setFailure(&result, RiskStatus::PREDICTION_INVALID,
                   "dynamic prediction has an invalid obstacle shape");
        return result;
      }
      Eigen::SelfAdjointEigenSolver<Covariance3> covariance_solver(
          0.5 * (vehicle_covariance + interpolated.covariance +
                 (vehicle_covariance + interpolated.covariance).transpose()));
      if (covariance_solver.info() != Eigen::Success ||
          covariance_solver.eigenvalues().minCoeff() < -options_.time_tolerance) {
        setFailure(&result, RiskStatus::PREDICTION_INVALID,
                   "relative position covariance is not positive semidefinite");
        return result;
      }
      const double sigma_radius =
          options_.sigma_multiplier * std::sqrt(std::max(0.0, covariance_solver.eigenvalues().maxCoeff()));
      const double safety_radius = options_.vehicle_radius + radius + sigma_radius;
      const double center_distance = (sample.position - interpolated.position).norm();
      const double clearance = center_distance - safety_radius;
      if (clearance < result.minimum_clearance) {
        result.minimum_clearance = clearance;
        result.worst_obstacle_id = static_cast<std::int64_t>(prediction.track_id);
        result.worst_time = sample.stamp;
      }
      const double normalized_risk =
          std::max(kMinimumExistenceProbability, interpolated.existence_probability) *
          std::clamp((options_.warning_clearance - clearance) / options_.warning_clearance,
                     0.0, 1.0);
      maximum_normalized_risk = std::max(maximum_normalized_risk, normalized_risk);
      if (clearance <= kEpsilon) {
        result.dynamic_risk = 1.0;
        result.total_risk = std::max(result.map_risk, result.dynamic_risk);
        setFailure(&result, RiskStatus::DYNAMIC_COLLISION,
                   "trajectory enters a conservative 3-sigma dynamic obstacle envelope");
        return result;
      }
    }
  }

  result.dynamic_risk = maximum_normalized_risk;
  result.total_risk = std::max(result.map_risk, result.dynamic_risk);
  result.status = RiskStatus::ACCEPTED;
  result.accepted = true;
  result.level = result.total_risk > 0.5 ? RiskLevel::MEDIUM : RiskLevel::LOW;
  result.detail = "trajectory passed conservative dynamic risk gating";
  return result;
}

}  // namespace aurora::risk
