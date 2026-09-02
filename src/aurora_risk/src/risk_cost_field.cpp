#include "aurora_risk/risk_cost_field.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace aurora::risk {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kMinimumExistenceProbability = 0.05;

bool finite(double value) { return std::isfinite(value); }

bool covarianceIsValid(const Covariance3 &covariance, double tolerance) {
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > tolerance) {
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
      return (shape.dimensions.array() > 0.0).all() ? 0.5 * shape.dimensions.norm()
                                                     : std::numeric_limits<double>::quiet_NaN();
    case aurora::prediction::ShapeType::CAPSULE:
      return ((shape.dimensions.array() >= 0.0).all() && shape.dimensions.norm() > kEpsilon)
                 ? shape.radius + 0.5 * shape.dimensions.norm()
                 : std::numeric_limits<double>::quiet_NaN();
    case aurora::prediction::ShapeType::MULTI_SPHERE:
      return (shape.dimensions.norm() > kEpsilon || shape.radius > kEpsilon)
                 ? std::max(shape.radius, 0.5 * shape.dimensions.norm())
                 : std::numeric_limits<double>::quiet_NaN();
  }
  return std::numeric_limits<double>::quiet_NaN();
}

struct InterpolatedState {
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Covariance3 covariance{Covariance3::Zero()};
  double existence_probability{0.0};
  aurora::prediction::ObstacleShape shape;
  std::string detail;
};

InterpolatedState interpolate(const aurora::prediction::PredictionResult &prediction,
                              double stamp, double tolerance) {
  InterpolatedState result;
  if (prediction.status != aurora::prediction::PredictionStatus::SUCCESS ||
      prediction.states.empty() || !finite(stamp) || !finite(prediction.reference_stamp) ||
      std::abs(prediction.states.front().stamp - prediction.reference_stamp) > tolerance ||
      stamp < prediction.states.front().stamp - tolerance ||
      stamp > prediction.states.back().stamp + tolerance) {
    result.detail = "dynamic soft-risk prediction does not cover the query time";
    return result;
  }

  std::size_t upper_index = 0U;
  while (upper_index < prediction.states.size() &&
         prediction.states[upper_index].stamp < stamp - tolerance) {
    ++upper_index;
  }
  if (upper_index == 0U) {
    const auto &state = prediction.states.front();
    result.position = state.position;
    result.covariance = state.covariance.block<3, 3>(0, 0);
    result.existence_probability = state.existence_probability;
    result.shape = state.shape;
  } else if (upper_index >= prediction.states.size()) {
    const auto &state = prediction.states.back();
    result.position = state.position;
    result.covariance = state.covariance.block<3, 3>(0, 0);
    result.existence_probability = state.existence_probability;
    result.shape = state.shape;
  } else {
    const auto &lower = prediction.states[upper_index - 1U];
    const auto &upper = prediction.states[upper_index];
    const double interval = upper.stamp - lower.stamp;
    if (!finite(interval) || interval <= tolerance) {
      result.detail = "dynamic soft-risk prediction states are not time ordered";
      return result;
    }
    const double alpha = std::clamp((stamp - lower.stamp) / interval, 0.0, 1.0);
    result.position = (1.0 - alpha) * lower.position + alpha * upper.position;
    result.covariance = (1.0 - alpha) * lower.covariance.block<3, 3>(0, 0) +
                        alpha * upper.covariance.block<3, 3>(0, 0);
    result.existence_probability = (1.0 - alpha) * lower.existence_probability +
                                   alpha * upper.existence_probability;
    result.shape = obstacleRadius(lower.shape) >= obstacleRadius(upper.shape)
                       ? lower.shape
                       : upper.shape;
  }

  if (!result.position.allFinite() || !covarianceIsValid(result.covariance, 1e-9) ||
      !finite(result.existence_probability) || result.existence_probability < 0.0 ||
      result.existence_probability > 1.0 || !finite(obstacleRadius(result.shape))) {
    result.detail = "dynamic soft-risk prediction contains invalid state data";
    return result;
  }
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
  result.valid = true;
  return result;
}

void fail(aurora::trajectory::RiskCostEvaluation *result, std::string detail) {
  result->valid = false;
  result->detail = std::move(detail);
}

}  // namespace

void DynamicRiskCostField::validateOptions(const DynamicRiskCostFieldOptions &options) {
  if (!finite(options.vehicle_radius) || options.vehicle_radius < 0.0 ||
      !finite(options.sigma_multiplier) || options.sigma_multiplier <= 0.0 ||
      !finite(options.warning_clearance) || options.warning_clearance <= 0.0 ||
      options.max_obstacles == 0U) {
    throw std::invalid_argument("invalid dynamic risk cost field options");
  }
}

DynamicRiskCostField::DynamicRiskCostField(
    std::vector<aurora::prediction::PredictionResult> predictions, RiskContext context,
    DynamicRiskCostFieldOptions options)
    : predictions_(std::move(predictions)), context_(std::move(context)),
      options_(std::move(options)) {
  validateOptions(options_);
  const auto validation = validateRiskContext(context_);
  if (!validation.valid) {
    throw std::invalid_argument("invalid dynamic risk cost field context: " + validation.detail);
  }
  if (predictions_.size() > options_.max_obstacles) {
    throw std::invalid_argument("dynamic risk cost field obstacle count exceeds the limit");
  }
}

aurora::trajectory::RiskCostEvaluation DynamicRiskCostField::evaluate(
    double absolute_stamp, const Eigen::Vector3d &position) const {
  aurora::trajectory::RiskCostEvaluation result;
  if (!finite(absolute_stamp) || !position.allFinite()) {
    fail(&result, "dynamic soft-risk query is non-finite");
    return result;
  }

  Covariance3 vehicle_covariance = Covariance3::Zero();
  if (context_.vehicle.has_localization_position_covariance) {
    vehicle_covariance += context_.vehicle.localization_position_covariance;
  }
  if (context_.vehicle.has_execution_position_covariance) {
    vehicle_covariance += context_.vehicle.execution_position_covariance;
  }
  vehicle_covariance = 0.5 * (vehicle_covariance + vehicle_covariance.transpose());

  double maximum_risk = 0.0;
  std::uint64_t maximum_track_id = std::numeric_limits<std::uint64_t>::max();
  for (const auto &prediction : predictions_) {
    const auto state = interpolate(prediction, absolute_stamp + context_.delay.total(), 1e-9);
    if (!state.valid) {
      fail(&result, state.detail);
      return result;
    }
    const Covariance3 relative_covariance = vehicle_covariance + state.covariance;
    Eigen::SelfAdjointEigenSolver<Covariance3> solver(
        0.5 * (relative_covariance + relative_covariance.transpose()));
    if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-9) {
      fail(&result, "dynamic soft-risk relative covariance is not positive semidefinite");
      return result;
    }
    const double radius = obstacleRadius(state.shape);
    const double sigma_radius = options_.sigma_multiplier *
                                std::sqrt(std::max(0.0, solver.eigenvalues().maxCoeff()));
    const double clearance = (position - state.position).norm() -
                             options_.vehicle_radius - radius - sigma_radius;
    const double geometric_risk = std::clamp(
        (options_.warning_clearance - clearance) / options_.warning_clearance, 0.0, 1.0);
    const double weighted_risk =
        std::max(kMinimumExistenceProbability, state.existence_probability) * geometric_risk;
    if (weighted_risk > maximum_risk + kEpsilon ||
        (std::abs(weighted_risk - maximum_risk) <= kEpsilon &&
         prediction.track_id < maximum_track_id)) {
      maximum_risk = weighted_risk;
      maximum_track_id = prediction.track_id;
      if (geometric_risk > 0.0) {
        const Eigen::Vector3d displacement = position - state.position;
        const double distance = displacement.norm();
        if (distance > kEpsilon) {
          // Keep an escape direction in the saturated region; hard safety is
          // still decided by DynamicRiskEvaluator after optimization.
          result.gradient = -std::max(kMinimumExistenceProbability,
                                      state.existence_probability) /
                            options_.warning_clearance * displacement / distance;
        } else {
          result.gradient.setZero();
        }
      } else {
        result.gradient.setZero();
      }
    }
  }
  result.value = maximum_risk;
  return result;
}

}  // namespace aurora::risk
