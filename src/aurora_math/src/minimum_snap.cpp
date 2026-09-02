#include "aurora_math/minimum_snap.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aurora::math {
namespace {

constexpr int kPolynomialOrder = 6;
constexpr int kJerkOrder = 3;
constexpr double kNumericalEpsilon = 1e-12;

bool isFinite(double value) { return std::isfinite(value); }

double fallingFactorial(int exponent, int derivative_order) {
  double result = 1.0;
  for (int i = 0; i < derivative_order; ++i) {
    result *= static_cast<double>(exponent - i);
  }
  return result;
}

double powerForDerivative(int exponent, int derivative_order, double time) {
  if (exponent < derivative_order) {
    return 0.0;
  }
  return fallingFactorial(exponent, derivative_order) *
         std::pow(time, static_cast<double>(exponent - derivative_order));
}

void setDerivativeRow(Eigen::MatrixXd &constraints, int row, int segment, int derivative_order,
                      double time, double sign) {
  const int base = 6 * segment;
  for (int coefficient = derivative_order; coefficient < kPolynomialOrder; ++coefficient) {
    constraints(row, base + coefficient) =
        sign * powerForDerivative(coefficient, derivative_order, time);
  }
}

Eigen::MatrixXd buildConstraintMatrix(const Eigen::VectorXd &segment_times) {
  const int segment_count = static_cast<int>(segment_times.size());
  const int coefficient_count = kPolynomialOrder * segment_count;
  const int constraint_count = 5 * segment_count + 1;
  Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(constraint_count, coefficient_count);

  int row = 0;
  for (int segment = 0; segment < segment_count; ++segment) {
    setDerivativeRow(constraints, row++, segment, 0, 0.0, 1.0);
    setDerivativeRow(constraints, row++, segment, 0, segment_times(segment), 1.0);
  }

  setDerivativeRow(constraints, row++, 0, 1, 0.0, 1.0);
  setDerivativeRow(constraints, row++, segment_count - 1, 1,
                   segment_times(segment_count - 1), 1.0);
  setDerivativeRow(constraints, row++, 0, 2, 0.0, 1.0);
  setDerivativeRow(constraints, row++, segment_count - 1, 2,
                   segment_times(segment_count - 1), 1.0);

  for (int segment = 0; segment + 1 < segment_count; ++segment) {
    setDerivativeRow(constraints, row, segment, 1, segment_times(segment), 1.0);
    setDerivativeRow(constraints, row++, segment + 1, 1, 0.0, -1.0);
    setDerivativeRow(constraints, row, segment, 2, segment_times(segment), 1.0);
    setDerivativeRow(constraints, row++, segment + 1, 2, 0.0, -1.0);
    setDerivativeRow(constraints, row, segment, kJerkOrder, segment_times(segment), 1.0);
    setDerivativeRow(constraints, row++, segment + 1, kJerkOrder, 0.0, -1.0);
  }

  if (row != constraint_count) {
    throw std::logic_error("minimum snap constraint count mismatch");
  }
  return constraints;
}

Eigen::MatrixXd buildJerkCostMatrix(const Eigen::VectorXd &segment_times) {
  const int segment_count = static_cast<int>(segment_times.size());
  Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(kPolynomialOrder * segment_count,
                                                kPolynomialOrder * segment_count);
  for (int segment = 0; segment < segment_count; ++segment) {
    const double duration = segment_times(segment);
    const int base = kPolynomialOrder * segment;
    for (int i = kJerkOrder; i < kPolynomialOrder; ++i) {
      for (int j = kJerkOrder; j < kPolynomialOrder; ++j) {
        const int exponent = i + j - 2 * kJerkOrder;
        cost(base + i, base + j) =
            fallingFactorial(i, kJerkOrder) * fallingFactorial(j, kJerkOrder) *
            std::pow(duration, static_cast<double>(exponent + 1)) /
            static_cast<double>(exponent + 1);
      }
    }
  }
  return cost;
}

Eigen::VectorXd buildAxisConstraints(const MinimumSnapTrajectory::WaypointMatrix &waypoints,
                                     int axis, const Eigen::VectorXd &segment_times,
                                     const Eigen::Vector3d &start_velocity,
                                     const Eigen::Vector3d &end_velocity,
                                     const Eigen::Vector3d &start_acceleration,
                                     const Eigen::Vector3d &end_acceleration) {
  const int segment_count = static_cast<int>(segment_times.size());
  Eigen::VectorXd values = Eigen::VectorXd::Zero(5 * segment_count + 1);
  int row = 0;
  for (int segment = 0; segment < segment_count; ++segment) {
    values(row++) = waypoints(axis, segment);
    values(row++) = waypoints(axis, segment + 1);
  }
  values(row++) = start_velocity(axis);
  values(row++) = end_velocity(axis);
  values(row++) = start_acceleration(axis);
  values(row++) = end_acceleration(axis);
  for (int segment = 0; segment + 1 < segment_count; ++segment) {
    values(row++) = 0.0;
    values(row++) = 0.0;
    values(row++) = 0.0;
  }
  return values;
}

Eigen::VectorXd solveAxis(const Eigen::MatrixXd &constraints, const Eigen::MatrixXd &jerk_cost,
                          const Eigen::VectorXd &constraint_values) {
  const int coefficient_count = static_cast<int>(jerk_cost.rows());
  const int constraint_count = static_cast<int>(constraints.rows());
  Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(coefficient_count + constraint_count,
                                               coefficient_count + constraint_count);
  kkt.topLeftCorner(coefficient_count, coefficient_count) = jerk_cost;
  kkt.topRightCorner(coefficient_count, constraint_count) = constraints.transpose();
  kkt.bottomLeftCorner(constraint_count, coefficient_count) = constraints;

  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(coefficient_count + constraint_count);
  rhs.tail(constraint_count) = constraint_values;
  const Eigen::VectorXd solution = kkt.fullPivLu().solve(rhs);
  if (!solution.allFinite()) {
    throw std::runtime_error("minimum snap KKT solve produced non-finite coefficients");
  }
  const double residual = (kkt * solution - rhs).norm();
  if (residual > 1e-8 * (1.0 + rhs.norm())) {
    throw std::runtime_error("minimum snap KKT solve residual is too large");
  }
  return solution.head(coefficient_count);
}

}  // namespace

MinimumSnapTrajectory::MinimumSnapTrajectory(std::vector<CoefficientMatrix> coefficients,
                                             Eigen::VectorXd segment_times)
    : coefficients_(std::move(coefficients)), segment_times_(std::move(segment_times)),
      cumulative_end_times_(segment_times_.size()) {
  for (int segment = 0; segment < segment_times_.size(); ++segment) {
    total_duration_ += segment_times_(segment);
    cumulative_end_times_(segment) = total_duration_;
  }
}

void MinimumSnapTrajectory::validateInputs(
    const WaypointMatrix &waypoints, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration, const Eigen::VectorXd &segment_times) {
  if (waypoints.rows() != 3 || waypoints.cols() < 2) {
    throw std::invalid_argument("minimum snap requires a 3 x N waypoint matrix with N >= 2");
  }
  if (!waypoints.allFinite() || !start_velocity.allFinite() || !end_velocity.allFinite() ||
      !start_acceleration.allFinite() || !end_acceleration.allFinite()) {
    throw std::invalid_argument("minimum snap states and waypoints must be finite");
  }
  if (segment_times.size() != waypoints.cols() - 1) {
    throw std::invalid_argument("minimum snap segment time count does not match waypoints");
  }
  for (int segment = 0; segment < segment_times.size(); ++segment) {
    if (!isFinite(segment_times(segment)) || segment_times(segment) <= 0.0) {
      throw std::invalid_argument("minimum snap segment times must be finite and positive");
    }
  }
}

MinimumSnapTrajectory MinimumSnapTrajectory::fromWaypoints(
    const WaypointMatrix &waypoints, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration, const Eigen::VectorXd &segment_times) {
  validateInputs(waypoints, start_velocity, end_velocity, start_acceleration, end_acceleration,
                 segment_times);

  const Eigen::MatrixXd constraints = buildConstraintMatrix(segment_times);
  const Eigen::MatrixXd jerk_cost = buildJerkCostMatrix(segment_times);
  std::vector<CoefficientMatrix> coefficients(
      static_cast<std::size_t>(segment_times.size()));
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::VectorXd axis_coefficients =
        solveAxis(constraints, jerk_cost,
                  buildAxisConstraints(waypoints, axis, segment_times, start_velocity, end_velocity,
                                       start_acceleration, end_acceleration));
    for (int segment = 0; segment < segment_times.size(); ++segment) {
      coefficients[static_cast<std::size_t>(segment)].row(axis) =
          axis_coefficients.segment<kPolynomialOrder>(kPolynomialOrder * segment).transpose();
    }
  }
  return MinimumSnapTrajectory(std::move(coefficients), segment_times);
}

MinimumSnapTrajectory MinimumSnapTrajectory::oneSegment(
    const Eigen::Vector3d &start_position, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &start_acceleration, const Eigen::Vector3d &end_position,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &end_acceleration,
    double duration) {
  if (!isFinite(duration) || duration <= 0.0) {
    throw std::invalid_argument("minimum snap duration must be finite and positive");
  }
  WaypointMatrix waypoints(3, 2);
  waypoints.col(0) = start_position;
  waypoints.col(1) = end_position;
  Eigen::VectorXd segment_times(1);
  segment_times(0) = duration;
  return fromWaypoints(waypoints, start_velocity, end_velocity, start_acceleration,
                       end_acceleration, segment_times);
}

Eigen::Vector3d MinimumSnapTrajectory::evaluate(double time, int derivative_order) const {
  if (!isFinite(time)) {
    throw std::invalid_argument("minimum snap evaluation time must be finite");
  }
  if (derivative_order < 0) {
    throw std::invalid_argument("minimum snap derivative order must be non-negative");
  }
  if (derivative_order >= kPolynomialOrder) {
    return Eigen::Vector3d::Zero();
  }

  const double clamped_time = std::clamp(time, 0.0, total_duration_);
  int segment = 0;
  double local_time = clamped_time;
  if (clamped_time >= total_duration_) {
    segment = segmentCount() - 1;
    local_time = segment_times_(segment);
  } else {
    while (clamped_time > cumulative_end_times_(segment) + kNumericalEpsilon &&
           segment + 1 < segmentCount()) {
      local_time -= segment_times_(segment);
      ++segment;
    }
    if (clamped_time >= cumulative_end_times_(segment) - kNumericalEpsilon &&
        segment + 1 < segmentCount()) {
      local_time = segment_times_(segment);
    }
  }

  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  const CoefficientMatrix &coefficient = coefficients_[static_cast<std::size_t>(segment)];
  for (int power = derivative_order; power < kPolynomialOrder; ++power) {
    result += coefficient.col(power) * powerForDerivative(power, derivative_order, local_time);
  }
  return result;
}

double MinimumSnapTrajectory::jerkCost() const noexcept {
  double total_cost = 0.0;
  for (int segment = 0; segment < segmentCount(); ++segment) {
    const double duration = segment_times_(segment);
    const CoefficientMatrix &coefficient = coefficients_[static_cast<std::size_t>(segment)];
    for (int axis = 0; axis < 3; ++axis) {
      for (int i = kJerkOrder; i < kPolynomialOrder; ++i) {
        for (int j = kJerkOrder; j < kPolynomialOrder; ++j) {
          const int exponent = i + j - 2 * kJerkOrder;
          total_cost += coefficient(axis, i) * coefficient(axis, j) *
                        fallingFactorial(i, kJerkOrder) * fallingFactorial(j, kJerkOrder) *
                        std::pow(duration, static_cast<double>(exponent + 1)) /
                        static_cast<double>(exponent + 1);
        }
      }
    }
  }
  return total_cost;
}

}  // namespace aurora::math
