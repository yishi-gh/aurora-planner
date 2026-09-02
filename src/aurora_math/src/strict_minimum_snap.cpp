#include "aurora_math/strict_minimum_snap.hpp"

#include <Eigen/LU>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::math {
namespace {

constexpr int kCoefficientCountPerSegment = 8;
constexpr int kSnapOrder = 4;
constexpr double kEpsilon = 1e-12;

bool finite(double value) { return std::isfinite(value); }

double fallingFactorial(int exponent, int derivative_order) {
  double result = 1.0;
  for (int index = 0; index < derivative_order; ++index) {
    result *= static_cast<double>(exponent - index);
  }
  return result;
}

double derivativeBasis(int exponent, int derivative_order, double time) {
  if (exponent < derivative_order) {
    return 0.0;
  }
  return fallingFactorial(exponent, derivative_order) *
         std::pow(time, static_cast<double>(exponent - derivative_order));
}

Eigen::Matrix<double, kCoefficientCountPerSegment, kCoefficientCountPerSegment>
buildLocalHermiteMatrix() {
  Eigen::Matrix<double, kCoefficientCountPerSegment, kCoefficientCountPerSegment> matrix =
      Eigen::Matrix<double, kCoefficientCountPerSegment, kCoefficientCountPerSegment>::Zero();
  for (int derivative_order = 0; derivative_order <= 3; ++derivative_order) {
    for (int coefficient = derivative_order;
         coefficient < kCoefficientCountPerSegment; ++coefficient) {
      matrix(2 * derivative_order, coefficient) =
          derivativeBasis(coefficient, derivative_order, 0.0);
      matrix(2 * derivative_order + 1, coefficient) =
          derivativeBasis(coefficient, derivative_order, 1.0);
    }
  }
  return matrix;
}

Eigen::Matrix<double, kCoefficientCountPerSegment, kCoefficientCountPerSegment>
buildNormalizedSnapGramian() {
  Eigen::Matrix<double, kCoefficientCountPerSegment, kCoefficientCountPerSegment> gramian =
      Eigen::Matrix<double, kCoefficientCountPerSegment, kCoefficientCountPerSegment>::Zero();
  for (int i = kSnapOrder; i < kCoefficientCountPerSegment; ++i) {
    for (int j = kSnapOrder; j < kCoefficientCountPerSegment; ++j) {
      const int exponent = i + j - 2 * kSnapOrder;
      gramian(i, j) = fallingFactorial(i, kSnapOrder) *
                      fallingFactorial(j, kSnapOrder) /
                      static_cast<double>(exponent + 1);
    }
  }
  return gramian;
}

std::vector<Eigen::Matrix<double, kCoefficientCountPerSegment, 1>> solveAxisByInternalDerivatives(
    const StrictMinimumSnapTrajectory::WaypointMatrix &waypoints, int axis,
    const Eigen::VectorXd &segment_times, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &start_jerk,
    const Eigen::Vector3d &end_jerk) {
  const int segment_count = static_cast<int>(segment_times.size());
  const int internal_derivative_count = 3 * std::max(0, segment_count - 1);
  const auto local_matrix = buildLocalHermiteMatrix();
  Eigen::FullPivLU<Eigen::Matrix<double, kCoefficientCountPerSegment,
                                  kCoefficientCountPerSegment>>
      local_decomposition(local_matrix);
  if (!local_decomposition.isInvertible()) {
    throw std::runtime_error("strict minimum snap local Hermite system is singular");
  }
  const auto normalized_gramian = buildNormalizedSnapGramian();

  std::vector<Eigen::Matrix<double, kCoefficientCountPerSegment, 1>> bases(
      static_cast<std::size_t>(segment_count));
  std::vector<Eigen::Matrix<double, kCoefficientCountPerSegment, Eigen::Dynamic>> maps(
      static_cast<std::size_t>(segment_count));
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(internal_derivative_count,
                                                   internal_derivative_count);
  Eigen::VectorXd linear = Eigen::VectorXd::Zero(internal_derivative_count);

  for (int segment = 0; segment < segment_count; ++segment) {
    const double duration = segment_times(segment);
    const double duration_squared = duration * duration;
    const double duration_cubed = duration_squared * duration;
    Eigen::Matrix<double, kCoefficientCountPerSegment, 1> values =
        Eigen::Matrix<double, kCoefficientCountPerSegment, 1>::Zero();
    values(0) = waypoints(axis, segment);
    values(1) = waypoints(axis, segment + 1);
    values(2) = segment == 0 ? start_velocity(axis) * duration : 0.0;
    values(3) = segment + 1 == segment_count ? end_velocity(axis) * duration : 0.0;
    values(4) = segment == 0 ? start_acceleration(axis) * duration_squared : 0.0;
    values(5) = segment + 1 == segment_count ? end_acceleration(axis) * duration_squared : 0.0;
    values(6) = segment == 0 ? start_jerk(axis) * duration_cubed : 0.0;
    values(7) = segment + 1 == segment_count ? end_jerk(axis) * duration_cubed : 0.0;

    Eigen::Matrix<double, kCoefficientCountPerSegment, Eigen::Dynamic> rhs_map =
        Eigen::Matrix<double, kCoefficientCountPerSegment, Eigen::Dynamic>::Zero(
            kCoefficientCountPerSegment, internal_derivative_count);
    if (segment > 0) {
      const int base = 3 * (segment - 1);
      rhs_map(2, base) = duration;
      rhs_map(4, base + 1) = duration_squared;
      rhs_map(6, base + 2) = duration_cubed;
    }
    if (segment + 1 < segment_count) {
      const int base = 3 * segment;
      rhs_map(3, base) = duration;
      rhs_map(5, base + 1) = duration_squared;
      rhs_map(7, base + 2) = duration_cubed;
    }

    bases[static_cast<std::size_t>(segment)] = local_decomposition.solve(values);
    maps[static_cast<std::size_t>(segment)] = local_decomposition.solve(rhs_map);
    if (!bases[static_cast<std::size_t>(segment)].allFinite() ||
        !maps[static_cast<std::size_t>(segment)].allFinite()) {
      throw std::runtime_error("strict minimum snap local solve produced non-finite values");
    }

    const double inverse_duration_seventh = 1.0 / std::pow(duration, 7.0);
    if (!finite(inverse_duration_seventh)) {
      throw std::runtime_error("strict minimum snap time scale is not finite");
    }
    const auto weighted_gramian = normalized_gramian * inverse_duration_seventh;
    hessian += maps[static_cast<std::size_t>(segment)].transpose() * weighted_gramian *
               maps[static_cast<std::size_t>(segment)];
    linear += maps[static_cast<std::size_t>(segment)].transpose() * weighted_gramian *
              bases[static_cast<std::size_t>(segment)];
  }

  Eigen::VectorXd internal_derivatives = Eigen::VectorXd::Zero(internal_derivative_count);
  if (internal_derivative_count > 0) {
    if (!hessian.allFinite() || !linear.allFinite()) {
      throw std::runtime_error("strict minimum snap reduced system is non-finite");
    }
    Eigen::VectorXd diagonal_scale = Eigen::VectorXd::Ones(internal_derivative_count);
    for (int index = 0; index < internal_derivative_count; ++index) {
      const double diagonal = std::abs(hessian(index, index));
      if (!finite(diagonal) || diagonal <= std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("strict minimum snap reduced system is singular");
      }
      diagonal_scale(index) = 1.0 / std::sqrt(diagonal);
    }
    const Eigen::MatrixXd scaled_hessian =
        diagonal_scale.asDiagonal() * hessian * diagonal_scale.asDiagonal();
    const Eigen::VectorXd scaled_rhs =
        diagonal_scale.asDiagonal() * (-linear);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(scaled_hessian);
    decomposition.setThreshold(1e-14);
    const Eigen::VectorXd scaled_solution = decomposition.solve(scaled_rhs);
    internal_derivatives = diagonal_scale.asDiagonal() * scaled_solution;
    const Eigen::VectorXd stationarity = hessian * internal_derivatives + linear;
    if (!internal_derivatives.allFinite() || !stationarity.allFinite() ||
        stationarity.norm() > 1e-7 * (1.0 + linear.norm())) {
      throw std::runtime_error("strict minimum snap reduced solve residual is too large");
    }
  }

  std::vector<Eigen::Matrix<double, kCoefficientCountPerSegment, 1>> result(
      static_cast<std::size_t>(segment_count));
  for (int segment = 0; segment < segment_count; ++segment) {
    result[static_cast<std::size_t>(segment)] =
        bases[static_cast<std::size_t>(segment)] +
        maps[static_cast<std::size_t>(segment)] * internal_derivatives;
    if (!result[static_cast<std::size_t>(segment)].allFinite()) {
      throw std::runtime_error("strict minimum snap coefficients are non-finite");
    }
  }
  return result;
}

}  // namespace

StrictMinimumSnapTrajectory::StrictMinimumSnapTrajectory(
    std::vector<CoefficientMatrix> coefficients,
    std::vector<CoefficientMatrix> normalized_coefficients,
    std::array<Eigen::Vector3d, 4> start_boundary,
    std::array<Eigen::Vector3d, 4> end_boundary, Eigen::VectorXd segment_times)
    : coefficients_(std::move(coefficients)),
      normalized_coefficients_(std::move(normalized_coefficients)),
      start_boundary_(std::move(start_boundary)), end_boundary_(std::move(end_boundary)),
      segment_times_(std::move(segment_times)),
      cumulative_end_times_(segment_times_.size()) {
  for (int segment = 0; segment < segment_times_.size(); ++segment) {
    total_duration_ += segment_times_(segment);
    cumulative_end_times_(segment) = total_duration_;
  }
}

void StrictMinimumSnapTrajectory::validateInputs(
    const WaypointMatrix &waypoints, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &start_jerk,
    const Eigen::Vector3d &end_jerk, const Eigen::VectorXd &segment_times) {
  if (waypoints.rows() != 3 || waypoints.cols() < 2) {
    throw std::invalid_argument("strict minimum snap requires a 3 x N waypoint matrix with N >= 2");
  }
  if (!waypoints.allFinite() || !start_velocity.allFinite() || !end_velocity.allFinite() ||
      !start_acceleration.allFinite() || !end_acceleration.allFinite() ||
      !start_jerk.allFinite() || !end_jerk.allFinite()) {
    throw std::invalid_argument("strict minimum snap states and waypoints must be finite");
  }
  if (segment_times.size() != waypoints.cols() - 1) {
    throw std::invalid_argument("strict minimum snap segment time count does not match waypoints");
  }
  for (int segment = 0; segment < segment_times.size(); ++segment) {
    if (!finite(segment_times(segment)) || segment_times(segment) <= 0.0) {
      throw std::invalid_argument("strict minimum snap segment times must be finite and positive");
    }
  }
}

StrictMinimumSnapTrajectory StrictMinimumSnapTrajectory::fromWaypoints(
    const WaypointMatrix &waypoints, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &start_jerk,
    const Eigen::Vector3d &end_jerk, const Eigen::VectorXd &segment_times) {
  validateInputs(waypoints, start_velocity, end_velocity, start_acceleration,
                 end_acceleration, start_jerk, end_jerk, segment_times);

  std::vector<CoefficientMatrix> coefficient_sets(
      static_cast<std::size_t>(segment_times.size()));
  std::vector<CoefficientMatrix> normalized_coefficients(
      static_cast<std::size_t>(segment_times.size()));
  std::array<Eigen::Vector3d, 4> start_boundary;
  std::array<Eigen::Vector3d, 4> end_boundary;
  start_boundary[0] = waypoints.col(0);
  start_boundary[1] = start_velocity;
  start_boundary[2] = start_acceleration;
  start_boundary[3] = start_jerk;
  end_boundary[0] = waypoints.col(waypoints.cols() - 1);
  end_boundary[1] = end_velocity;
  end_boundary[2] = end_acceleration;
  end_boundary[3] = end_jerk;
  for (int axis = 0; axis < 3; ++axis) {
    const auto normalized_axis_coefficients = solveAxisByInternalDerivatives(
        waypoints, axis, segment_times, start_velocity, end_velocity, start_acceleration,
        end_acceleration, start_jerk, end_jerk);
    for (int segment = 0; segment < segment_times.size(); ++segment) {
      const auto &normalized = normalized_axis_coefficients[static_cast<std::size_t>(segment)];
      const double segment_duration = segment_times(segment);
      for (int power = 0; power < kCoefficientCountPerSegment; ++power) {
        normalized_coefficients[static_cast<std::size_t>(segment)](axis, power) = normalized(power);
        coefficient_sets[static_cast<std::size_t>(segment)](axis, power) =
            normalized(power) / std::pow(segment_duration, static_cast<double>(power));
      }
    }
  }
  return StrictMinimumSnapTrajectory(std::move(coefficient_sets), std::move(normalized_coefficients),
                                    std::move(start_boundary), std::move(end_boundary),
                                    segment_times);
}

StrictMinimumSnapTrajectory StrictMinimumSnapTrajectory::oneSegment(
    const Eigen::Vector3d &start_position, const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &start_acceleration, const Eigen::Vector3d &start_jerk,
    const Eigen::Vector3d &end_position, const Eigen::Vector3d &end_velocity,
    const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &end_jerk,
    double duration) {
  if (!finite(duration) || duration <= 0.0) {
    throw std::invalid_argument("strict minimum snap duration must be finite and positive");
  }
  WaypointMatrix waypoints(3, 2);
  waypoints.col(0) = start_position;
  waypoints.col(1) = end_position;
  Eigen::VectorXd segment_times(1);
  segment_times(0) = duration;
  return fromWaypoints(waypoints, start_velocity, end_velocity, start_acceleration,
                       end_acceleration, start_jerk, end_jerk, segment_times);
}

Eigen::Vector3d StrictMinimumSnapTrajectory::evaluate(double time, int derivative_order) const {
  if (!finite(time)) {
    throw std::invalid_argument("strict minimum snap evaluation time must be finite");
  }
  if (derivative_order < 0) {
    throw std::invalid_argument("strict minimum snap derivative order must be non-negative");
  }
  if (derivative_order >= kCoefficientCountPerSegment) {
    return Eigen::Vector3d::Zero();
  }

  const double clamped_time = std::clamp(time, 0.0, total_duration_);
  if (derivative_order <= 3 && clamped_time <= kEpsilon) {
    return start_boundary_[static_cast<std::size_t>(derivative_order)];
  }
  if (derivative_order <= 3 && clamped_time >= total_duration_ - kEpsilon) {
    return end_boundary_[static_cast<std::size_t>(derivative_order)];
  }
  int segment = 0;
  double segment_start = 0.0;
  if (clamped_time >= total_duration_) {
    segment = segmentCount() - 1;
    segment_start = total_duration_ - segment_times_(segment);
  } else {
    while (segment + 1 < segmentCount() &&
           clamped_time > cumulative_end_times_(segment) + kEpsilon) {
      segment_start = cumulative_end_times_(segment);
      ++segment;
    }
  }
  const double local_time = std::clamp(clamped_time - segment_start, 0.0,
                                       segment_times_(segment));
  const double segment_duration = segment_times_(segment);
  const double normalized_time = local_time / segment_duration;
  const double derivative_scale =
      std::pow(segment_duration, static_cast<double>(derivative_order));
  if (!finite(normalized_time) || !finite(derivative_scale) || derivative_scale <= 0.0) {
    throw std::runtime_error("strict minimum snap evaluation scale is not finite");
  }
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  const CoefficientMatrix &coefficient =
      normalized_coefficients_[static_cast<std::size_t>(segment)];
  for (int power = derivative_order; power < kCoefficientCountPerSegment; ++power) {
    result += coefficient.col(power) *
              (derivativeBasis(power, derivative_order, normalized_time) / derivative_scale);
  }
  return result;
}

double StrictMinimumSnapTrajectory::snapCost() const noexcept {
  double total_cost = 0.0;
  for (int segment = 0; segment < segmentCount(); ++segment) {
    const double segment_duration = segment_times_(segment);
    const CoefficientMatrix &coefficient =
        normalized_coefficients_[static_cast<std::size_t>(segment)];
    const double inverse_duration_seventh = 1.0 / std::pow(segment_duration, 7.0);
    for (int axis = 0; axis < 3; ++axis) {
      for (int i = kSnapOrder; i < kCoefficientCountPerSegment; ++i) {
        for (int j = kSnapOrder; j < kCoefficientCountPerSegment; ++j) {
          const int exponent = i + j - 2 * kSnapOrder;
          total_cost += coefficient(axis, i) * coefficient(axis, j) *
                        fallingFactorial(i, kSnapOrder) * fallingFactorial(j, kSnapOrder) *
                        inverse_duration_seventh /
                        static_cast<double>(exponent + 1);
        }
      }
    }
  }
  return std::max(0.0, total_cost);
}

}  // namespace aurora::math
