#include "aurora_trajectory/static_bspline_optimizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::trajectory {
namespace {

constexpr double kEpsilon = 1e-12;

bool isFinite(double value) { return std::isfinite(value); }

void validateWeight(double value, const char *name) {
  if (!isFinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

}  // namespace

const char *toString(OptimizationStatus status) noexcept {
  switch (status) {
    case OptimizationStatus::CONVERGED:
      return "CONVERGED";
    case OptimizationStatus::MAX_ITERATIONS:
      return "MAX_ITERATIONS";
    case OptimizationStatus::STALLED:
      return "STALLED";
    case OptimizationStatus::TIMEOUT:
      return "TIMEOUT";
  }
  return "UNKNOWN_STATUS";
}

const char *toString(ValidationStatus status) noexcept {
  switch (status) {
    case ValidationStatus::VALID:
      return "VALID";
    case ValidationStatus::INVALID_OPTIONS:
      return "INVALID_OPTIONS";
    case ValidationStatus::NONFINITE:
      return "NONFINITE";
    case ValidationStatus::OUT_OF_MAP:
      return "OUT_OF_MAP";
    case ValidationStatus::OCCUPIED:
      return "OCCUPIED";
    case ValidationStatus::UNKNOWN:
      return "UNKNOWN";
    case ValidationStatus::VELOCITY_LIMIT:
      return "VELOCITY_LIMIT";
    case ValidationStatus::ACCELERATION_LIMIT:
      return "ACCELERATION_LIMIT";
  }
  return "UNKNOWN_STATUS";
}

void StaticBsplineOptimizer::validateOptions(const StaticOptimizerOptions &options) {
  if (!isFinite(options.interval) || options.interval <= 0.0 ||
      !isFinite(options.clearance) || options.clearance < 0.0 ||
      !isFinite(options.max_velocity) || options.max_velocity <= 0.0 ||
      !isFinite(options.max_acceleration) || options.max_acceleration <= 0.0) {
    throw std::invalid_argument("invalid static B-spline optimizer limits");
  }
  validateWeight(options.lambda_smooth, "lambda_smooth");
  validateWeight(options.lambda_obstacle, "lambda_obstacle");
  validateWeight(options.lambda_feasibility, "lambda_feasibility");
  validateWeight(options.lambda_fitness, "lambda_fitness");
  validateWeight(options.lambda_risk, "lambda_risk");
  if (!isFinite(options.risk_time_origin) || options.max_risk_evaluations == 0U) {
    throw std::invalid_argument("invalid static B-spline risk options");
  }
  if (!isFinite(options.max_compute_time_sec) || options.max_compute_time_sec < 0.0 ||
      options.max_iterations <= 0 || options.max_line_search_iterations <= 0 ||
      options.samples_per_span == 0U || !isFinite(options.initial_step) ||
      options.initial_step <= 0.0 || !isFinite(options.gradient_clip) ||
      options.gradient_clip <= 0.0 || !isFinite(options.convergence_gradient_norm) ||
      options.convergence_gradient_norm < 0.0 || !isFinite(options.improvement_tolerance) ||
      options.improvement_tolerance < 0.0) {
    throw std::invalid_argument("invalid static B-spline optimizer iteration options");
  }
}

void StaticBsplineOptimizer::validateControlPoints(const ControlPointMatrix &control_points,
                                                   const char *name) {
  if (control_points.rows() != 3 || control_points.cols() < 7) {
    throw std::invalid_argument(std::string(name) + " must be a 3 x N matrix with N >= 7");
  }
  if (!control_points.allFinite()) {
    throw std::invalid_argument(std::string(name) + " contains non-finite values");
  }
}

std::vector<Eigen::Vector3d> StaticBsplineOptimizer::collectOccupiedCenters(
    const aurora::map::VoxelMap &map) {
  std::vector<Eigen::Vector3d> centers;
  const auto &config = map.config();
  for (int x = 0; x < config.dimensions.x(); ++x) {
    for (int y = 0; y < config.dimensions.y(); ++y) {
      for (int z = 0; z < config.dimensions.z(); ++z) {
        const aurora::map::GridIndex index{x, y, z};
        const auto query = map.query(index);
        if (query.occupancy_probability >= config.occupancy_threshold) {
          centers.push_back(map.indexToWorld(index));
        }
      }
    }
  }
  return centers;
}

StaticBsplineOptimizer::StaticBsplineOptimizer(
    const aurora::map::VoxelMap &map, ControlPointMatrix initial_control_points,
    ControlPointMatrix reference_control_points, StaticOptimizerOptions options)
    : map_(map), options_(std::move(options)),
      control_points_(std::move(initial_control_points)),
      reference_control_points_(std::move(reference_control_points)),
      occupied_centers_(collectOccupiedCenters(map)) {
  validateOptions(options_);
  validateControlPoints(control_points_, "initial_control_points");
  validateControlPoints(reference_control_points_, "reference_control_points");
  if (control_points_.cols() != reference_control_points_.cols()) {
    throw std::invalid_argument("initial and reference control point counts must match");
  }
}

StaticBsplineOptimizer::ObstaclePotential StaticBsplineOptimizer::obstaclePotential(
    const Eigen::Vector3d &position) const {
  ObstaclePotential result;
  const Eigen::Vector3d map_min = map_.origin();
  const Eigen::Vector3d map_max =
      map_.origin() + map_.resolution() * map_.dimensions().cast<double>();
  if (!map_.isInMap(position)) {
    const Eigen::Vector3d clamped = position.cwiseMax(map_min).cwiseMin(map_max);
    const Eigen::Vector3d displacement = position - clamped;
    const double distance = displacement.norm();
    const double margin = options_.clearance + distance;
    result.value = margin * margin;
    if (distance > kEpsilon) {
      result.gradient = 2.0 * margin * displacement / distance;
    }
    return result;
  }

  double nearest_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d nearest_center = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &center : occupied_centers_) {
    const double distance = (position - center).norm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_center = center;
    }
  }
  if (!isFinite(nearest_distance)) {
    return result;
  }

  const double margin = options_.clearance - nearest_distance;
  if (margin <= 0.0) {
    return result;
  }
  result.value = margin * margin;

  Eigen::Vector3d away = position - nearest_center;
  if (away.norm() <= kEpsilon) {
    // A control point exactly at a voxel center has no radial derivative. Use
    // the first free cardinal direction to escape the occupied cell.
    const std::array<Eigen::Vector3d, 6> directions = {
        Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()};
    for (const Eigen::Vector3d &direction : directions) {
      if (map_.query(position + map_.resolution() * direction).state ==
          aurora::map::MapState::FREE) {
        away = direction;
        break;
      }
    }
    if (away.norm() <= kEpsilon) {
      away = Eigen::Vector3d::UnitY();
    }
  } else {
    away.normalize();
  }
  result.gradient = -2.0 * margin * away;
  return result;
}

CostBreakdown StaticBsplineOptimizer::evaluateInternal(const ControlPointMatrix &control_points,
                                                        bool with_gradient,
                                                        bool include_risk) const {
  validateControlPoints(control_points, "control_points");
  if (control_points.cols() != reference_control_points_.cols()) {
    throw std::invalid_argument("control point count does not match reference");
  }

  CostBreakdown result;
  result.gradient = ControlPointMatrix::Zero(3, control_points.cols());
  const auto add_gradient = [&](int index, const Eigen::Vector3d &value) {
    if (with_gradient) {
      result.gradient.col(index) += value;
    }
  };

  for (int i = 0; i + 3 < control_points.cols(); ++i) {
    const Eigen::Vector3d jerk = control_points.col(i + 3) - 3.0 * control_points.col(i + 2) +
                                 3.0 * control_points.col(i + 1) - control_points.col(i);
    result.smoothness += options_.lambda_smooth * jerk.squaredNorm();
    const Eigen::Vector3d derivative = 2.0 * options_.lambda_smooth * jerk;
    add_gradient(i, -derivative);
    add_gradient(i + 1, 3.0 * derivative);
    add_gradient(i + 2, -3.0 * derivative);
    add_gradient(i + 3, derivative);
  }

  for (int i = 0; i < control_points.cols(); ++i) {
    const Eigen::Vector3d error = control_points.col(i) - reference_control_points_.col(i);
    result.fitness += options_.lambda_fitness * error.squaredNorm();
    add_gradient(i, 2.0 * options_.lambda_fitness * error);
  }

  const double dt = options_.interval;
  const double inverse_dt = 1.0 / dt;
  const double inverse_dt_squared = inverse_dt * inverse_dt;
  const auto add_limit_cost = [&](const Eigen::Vector3d &value, double limit, double scale,
                                  int i0, int i1, int i2, bool acceleration) {
    const double value_norm = value.norm();
    if (value_norm <= limit) {
      return;
    }
    const double excess = value_norm - limit;
    result.feasibility += options_.lambda_feasibility * scale * excess * excess;
    if (!with_gradient) {
      return;
    }
    const Eigen::Vector3d direction = value / std::max(value_norm, 1e-9);
    const Eigen::Vector3d derivative =
        2.0 * options_.lambda_feasibility * scale * excess * direction;
    if (acceleration) {
      add_gradient(i0, derivative);
      add_gradient(i1, -2.0 * derivative);
      add_gradient(i2, derivative);
    } else {
      add_gradient(i0, -derivative);
      add_gradient(i1, derivative);
    }
  };

  for (int i = 0; i + 1 < control_points.cols(); ++i) {
    add_limit_cost((control_points.col(i + 1) - control_points.col(i)) * inverse_dt,
                   options_.max_velocity, inverse_dt, i, i + 1, 0, false);
  }
  for (int i = 0; i + 2 < control_points.cols(); ++i) {
    add_limit_cost((control_points.col(i + 2) - 2.0 * control_points.col(i + 1) +
                    control_points.col(i)) * inverse_dt_squared,
                   options_.max_acceleration, 1.0, i, i + 1, i + 2, true);
  }

  const aurora::math::UniformBspline spline(control_points, options_.interval,
                                             options_.knot_mode);
  const std::size_t sample_count =
      std::max<std::size_t>(1U, static_cast<std::size_t>(spline.controlPointCount() -
                                                         spline.degree()) *
                                 options_.samples_per_span);
  std::size_t risk_evaluations = 0U;
  for (std::size_t sample = 0; sample <= sample_count; ++sample) {
    const double time = spline.duration() * static_cast<double>(sample) /
                        static_cast<double>(sample_count);
    const Eigen::Vector3d position = spline.evaluate(time);
    const ObstaclePotential potential = obstaclePotential(position);
    result.obstacle += options_.lambda_obstacle * potential.value;
    if (with_gradient && potential.value > 0.0) {
      const Eigen::VectorXd basis = spline.basisFunctions(time);
      for (int i = 0; i < basis.size(); ++i) {
        add_gradient(i, options_.lambda_obstacle * potential.gradient * basis(i));
      }
    }

    if (include_risk && options_.lambda_risk > 0.0 && options_.risk_cost) {
      if (risk_evaluations >= options_.max_risk_evaluations) {
        result.risk_available = false;
        result.risk_evaluation_failed = true;
        result.risk_detail = "risk evaluation budget exceeded";
        break;
      }
      ++risk_evaluations;
      RiskCostEvaluation risk;
      try {
        risk = options_.risk_cost(options_.risk_time_origin + time, position);
      } catch (const std::exception &error) {
        result.risk_available = false;
        result.risk_evaluation_failed = true;
        result.risk_detail = std::string("risk callback threw an exception: ") + error.what();
        break;
      } catch (...) {
        result.risk_available = false;
        result.risk_evaluation_failed = true;
        result.risk_detail = "risk callback threw an unknown exception";
        break;
      }
      if (!risk.valid || !isFinite(risk.value) || risk.value < 0.0 ||
          !risk.gradient.allFinite()) {
        result.risk_available = false;
        result.risk_evaluation_failed = true;
        result.risk_detail = risk.detail.empty() ? "risk callback returned an invalid value"
                                                  : risk.detail;
        break;
      }
      result.risk += options_.lambda_risk * risk.value;
      if (with_gradient && risk.value > 0.0) {
        const Eigen::VectorXd basis = spline.basisFunctions(time);
        for (int i = 0; i < basis.size(); ++i) {
          add_gradient(i, options_.lambda_risk * risk.gradient * basis(i));
        }
      }
    }
  }

  result.total = result.smoothness + result.obstacle + result.feasibility + result.fitness;
  result.total += result.risk;
  return result;
}

CostBreakdown StaticBsplineOptimizer::evaluate(const ControlPointMatrix &control_points,
                                               bool with_gradient) const {
  return evaluateInternal(control_points, with_gradient, true);
}

OptimizationResult StaticBsplineOptimizer::optimize() {
  OptimizationResult result;
  const auto optimization_start = std::chrono::steady_clock::now();
  const auto budget_exceeded = [&]() {
    return options_.max_compute_time_sec > 0.0 &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         optimization_start)
                   .count() >= options_.max_compute_time_sec;
  };
  bool risk_enabled = options_.lambda_risk > 0.0 && static_cast<bool>(options_.risk_cost);
  bool risk_fallback = false;
  CostBreakdown current = evaluateInternal(control_points_, true, risk_enabled);
  if (current.risk_evaluation_failed) {
    risk_enabled = false;
    risk_fallback = true;
    current = evaluateInternal(control_points_, true, false);
  }
  const auto make_timeout_result = [&](int iteration) {
    result.status = OptimizationStatus::TIMEOUT;
    result.iterations = iteration;
    result.control_points = control_points_;
    result.cost = std::move(current);
    result.risk_enabled = risk_enabled;
    result.risk_fallback = risk_fallback;
    return result;
  };
  if (budget_exceeded()) {
    return make_timeout_result(0);
  }
  const int free_begin = 3;
  const int free_end = control_points_.cols() - 3;
  for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
    if (budget_exceeded()) {
      return make_timeout_result(iteration);
    }
    double largest_gradient = 0.0;
    for (int i = free_begin; i < free_end; ++i) {
      largest_gradient = std::max(largest_gradient, current.gradient.col(i).norm());
    }
    if (largest_gradient <= options_.convergence_gradient_norm) {
      result.status = OptimizationStatus::CONVERGED;
      result.iterations = iteration;
      result.control_points = control_points_;
      result.cost = std::move(current);
      result.risk_enabled = risk_enabled;
      result.risk_fallback = risk_fallback;
      return result;
    }

    bool accepted = false;
    double step = options_.initial_step;
    for (int line_search = 0; line_search < options_.max_line_search_iterations; ++line_search) {
      if (budget_exceeded()) {
        return make_timeout_result(iteration);
      }
      ControlPointMatrix candidate = control_points_;
      for (int i = free_begin; i < free_end; ++i) {
        Eigen::Vector3d direction = current.gradient.col(i);
        const double gradient_norm = direction.norm();
        if (gradient_norm > options_.gradient_clip) {
          direction *= options_.gradient_clip / gradient_norm;
        }
        candidate.col(i) -= step * direction;
      }
      const CostBreakdown candidate_cost = evaluateInternal(candidate, false, risk_enabled);
      if (budget_exceeded()) {
        return make_timeout_result(iteration);
      }
      if (candidate_cost.risk_evaluation_failed) {
        risk_enabled = false;
        risk_fallback = true;
        current = evaluateInternal(control_points_, true, false);
        accepted = true;
        break;
      }
      if (candidate_cost.total + options_.improvement_tolerance < current.total) {
        control_points_ = std::move(candidate);
        current = evaluateInternal(control_points_, true, risk_enabled);
        if (current.risk_evaluation_failed) {
          risk_enabled = false;
          risk_fallback = true;
          current = evaluateInternal(control_points_, true, false);
        }
        accepted = true;
        break;
      }
      step *= 0.5;
    }

    if (!accepted) {
      result.status = OptimizationStatus::STALLED;
      result.iterations = iteration;
      result.control_points = control_points_;
      result.cost = std::move(current);
      result.risk_enabled = risk_enabled;
      result.risk_fallback = risk_fallback;
      return result;
    }
  }

  result.status = OptimizationStatus::MAX_ITERATIONS;
  result.iterations = options_.max_iterations;
  result.control_points = control_points_;
  result.cost = evaluateInternal(control_points_, true, risk_enabled);
  if (result.cost.risk_evaluation_failed) {
    result.risk_enabled = false;
    result.risk_fallback = true;
    result.cost = evaluateInternal(control_points_, true, false);
  }
  result.risk_enabled = risk_enabled;
  result.risk_fallback = risk_fallback;
  return result;
}

StaticTrajectoryValidationResult validateStaticTrajectoryWindow(
    const aurora::map::VoxelMap &map, const aurora::math::UniformBspline &spline,
    double start_time, double duration, const StaticTrajectoryValidationOptions &options) {
  StaticTrajectoryValidationResult result;
  if (options.samples_per_span == 0U || !isFinite(options.max_velocity) ||
      options.max_velocity <= 0.0 || !isFinite(options.max_acceleration) ||
      options.max_acceleration <= 0.0 || !isFinite(options.tolerance) ||
      options.tolerance < 0.0 || !isFinite(start_time) || !isFinite(duration) || start_time < 0.0 ||
      duration <= 0.0 || start_time + duration > spline.duration() + options.tolerance) {
    result.status = ValidationStatus::INVALID_OPTIONS;
    result.detail = "invalid static trajectory validation options";
    return result;
  }

  const std::size_t sample_count =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(duration / spline.dt())) *
                                 options.samples_per_span);
  for (std::size_t sample = 0; sample <= sample_count; ++sample) {
    const double time = start_time + duration * static_cast<double>(sample) /
                                        static_cast<double>(sample_count);
    const Eigen::Vector3d position = spline.evaluate(time);
    if (!position.allFinite()) {
      result.status = ValidationStatus::NONFINITE;
      result.detail = "trajectory position is non-finite";
      return result;
    }
    ++result.checked_samples;
    const auto query = map.query(position);
    if (query.state == aurora::map::MapState::OUT_OF_MAP) {
      result.status = ValidationStatus::OUT_OF_MAP;
      result.detail = "trajectory leaves the map";
      return result;
    }
    if (query.state == aurora::map::MapState::OCCUPIED) {
      ++result.occupied_samples;
    } else if (query.state == aurora::map::MapState::UNKNOWN) {
      ++result.unknown_samples;
    }

    const Eigen::Vector3d velocity = spline.evaluate(time, 1);
    const Eigen::Vector3d acceleration = spline.evaluate(time, 2);
    if (!velocity.allFinite() || !acceleration.allFinite()) {
      result.status = ValidationStatus::NONFINITE;
      result.detail = "trajectory derivative is non-finite";
      return result;
    }
    result.maximum_velocity = std::max(result.maximum_velocity, velocity.norm());
    result.maximum_acceleration = std::max(result.maximum_acceleration, acceleration.norm());
  }

  if (result.occupied_samples > 0U) {
    result.status = ValidationStatus::OCCUPIED;
    result.detail = "trajectory intersects occupied or inflated voxels";
    return result;
  }
  if (options.reject_unknown && result.unknown_samples > 0U) {
    result.status = ValidationStatus::UNKNOWN;
    result.detail = "trajectory enters unknown voxels";
    return result;
  }
  if (result.maximum_velocity > options.max_velocity + options.tolerance) {
    result.status = ValidationStatus::VELOCITY_LIMIT;
    result.detail = "sampled trajectory velocity exceeds the limit";
    return result;
  }
  if (result.maximum_acceleration > options.max_acceleration + options.tolerance) {
    result.status = ValidationStatus::ACCELERATION_LIMIT;
    result.detail = "sampled trajectory acceleration exceeds the limit";
    return result;
  }
  result.status = ValidationStatus::VALID;
  result.valid = true;
  result.detail = "trajectory passed static sampled validation";
  return result;
}

StaticTrajectoryValidationResult validateStaticTrajectory(
    const aurora::map::VoxelMap &map, const aurora::math::UniformBspline &spline,
    const StaticTrajectoryValidationOptions &options) {
  return validateStaticTrajectoryWindow(map, spline, 0.0, spline.duration(), options);
}

}  // namespace aurora::trajectory
