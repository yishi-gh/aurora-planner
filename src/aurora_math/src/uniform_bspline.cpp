#include "aurora_math/uniform_bspline.hpp"

#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::math {
namespace {

constexpr int kCubicDegree = 3;
constexpr double kNumericalEpsilon = 1e-12;

bool isFinite(double value) { return std::isfinite(value); }

}  // namespace

UniformBspline::UniformBspline(ControlPointMatrix control_points, double dt,
                               UniformBsplineKnotMode knot_mode)
  : control_points_(std::move(control_points)), dt_(dt), degree_(kCubicDegree),
    knot_mode_(knot_mode) {
  validateControlPoints(control_points_, degree_);
  if (!isFinite(dt_) || dt_ <= 0.0) {
    throw std::invalid_argument("B-spline dt must be finite and positive");
  }
  knots_ = knot_mode_ == UniformBsplineKnotMode::CLAMPED
               ? makeClampedUniformKnots(controlPointCount(), degree_)
               : makeEgoUniformKnots(controlPointCount(), degree_);
}

UniformBspline::UniformBspline(ControlPointMatrix control_points, double dt, int degree,
                               std::vector<double> knots, UniformBsplineKnotMode knot_mode,
                               DerivedSplineTag)
    : control_points_(std::move(control_points)), dt_(dt), degree_(degree),
      knot_mode_(knot_mode), knots_(std::move(knots)) {
  validateControlPoints(control_points_, degree_);
  if (!isFinite(dt_) || dt_ <= 0.0 || degree_ < 0 || degree_ > kCubicDegree) {
    throw std::invalid_argument("invalid derived B-spline parameters");
  }
  if (controlPointCount() <= degree_ || knots_.size() !=
                                             static_cast<std::size_t>(controlPointCount() + degree_ + 1)) {
    throw std::invalid_argument("inconsistent derived B-spline knot vector");
  }
}

void UniformBspline::validateControlPoints(const ControlPointMatrix &control_points, int degree) {
  if (control_points.rows() != 3 || control_points.cols() <= degree) {
    throw std::invalid_argument("B-spline requires a 3 x N matrix with N > degree");
  }
  if (!control_points.allFinite()) {
    throw std::invalid_argument("B-spline control points must be finite");
  }
}

std::vector<double> UniformBspline::makeClampedUniformKnots(int control_point_count, int degree) {
  const int knot_count = control_point_count + degree + 1;
  const int span_count = control_point_count - degree;
  std::vector<double> knots(static_cast<std::size_t>(knot_count), 0.0);
  for (int i = 0; i < knot_count; ++i) {
    if (i <= degree) {
      knots[static_cast<std::size_t>(i)] = 0.0;
    } else if (i >= control_point_count) {
      knots[static_cast<std::size_t>(i)] = static_cast<double>(span_count);
    } else {
      knots[static_cast<std::size_t>(i)] = static_cast<double>(i - degree);
    }
  }
  return knots;
}

std::vector<double> UniformBspline::makeEgoUniformKnots(int control_point_count, int degree) {
  const int knot_count = control_point_count + degree + 1;
  std::vector<double> knots(static_cast<std::size_t>(knot_count), 0.0);
  for (int index = 0; index < knot_count; ++index) {
    knots[static_cast<std::size_t>(index)] =
        index <= degree ? static_cast<double>(index - degree)
                        : knots[static_cast<std::size_t>(index - 1)] + 1.0;
  }
  return knots;
}

double UniformBspline::duration() const noexcept {
  return static_cast<double>(controlPointCount() - degree_) * dt_;
}

double UniformBspline::clampNormalizedTime(const std::vector<double> &knots, int degree,
                                            int control_point_count, double normalized_time) {
  const double start = knots[static_cast<std::size_t>(degree)];
  const double end = knots[static_cast<std::size_t>(control_point_count)];
  if (!isFinite(normalized_time)) {
    throw std::invalid_argument("B-spline evaluation time must be finite");
  }
  return std::clamp(normalized_time, start, end);
}

int UniformBspline::findSpan(const std::vector<double> &knots, int degree, int control_point_count,
                             double normalized_time) {
  const int last_control_point = control_point_count - 1;
  const double end = knots[static_cast<std::size_t>(control_point_count)];
  if (normalized_time >= end - kNumericalEpsilon) {
    return last_control_point;
  }
  for (int span = degree; span < control_point_count; ++span) {
    if (normalized_time >= knots[static_cast<std::size_t>(span)] &&
        normalized_time < knots[static_cast<std::size_t>(span + 1)]) {
      return span;
    }
  }
  return degree;
}

Eigen::Vector3d UniformBspline::evaluateCurve(const ControlPointMatrix &control_points, int degree,
                                              const std::vector<double> &knots,
                                              double normalized_time) {
  const int control_point_count = static_cast<int>(control_points.cols());
  const double u = clampNormalizedTime(knots, degree, control_point_count, normalized_time);
  const int span = findSpan(knots, degree, control_point_count, u);

  Eigen::Matrix<double, 3, Eigen::Dynamic> work(3, degree + 1);
  for (int j = 0; j <= degree; ++j) {
    work.col(j) = control_points.col(span - degree + j);
  }

  for (int order = 1; order <= degree; ++order) {
    for (int j = degree; j >= order; --j) {
      const int knot_index = span - degree + j;
      const double denominator = knots[static_cast<std::size_t>(knot_index + degree - order + 1)] -
                                 knots[static_cast<std::size_t>(knot_index)];
      const double alpha = denominator > kNumericalEpsilon
                               ? (u - knots[static_cast<std::size_t>(knot_index)]) / denominator
                               : 0.0;
      work.col(j) = (1.0 - alpha) * work.col(j - 1) + alpha * work.col(j);
    }
  }
  return work.col(degree);
}

Eigen::VectorXd UniformBspline::evaluateBasis(const std::vector<double> &knots, int degree,
                                              int control_point_count, double normalized_time) {
  const double u = clampNormalizedTime(knots, degree, control_point_count, normalized_time);
  Eigen::VectorXd result = Eigen::VectorXd::Zero(control_point_count);
  const int span = findSpan(knots, degree, control_point_count, u);
  std::vector<double> left(static_cast<std::size_t>(degree + 1), 0.0);
  std::vector<double> right(static_cast<std::size_t>(degree + 1), 0.0);
  std::vector<double> local_basis(static_cast<std::size_t>(degree + 1), 0.0);
  local_basis[0] = 1.0;

  for (int order = 1; order <= degree; ++order) {
    left[static_cast<std::size_t>(order)] =
        u - knots[static_cast<std::size_t>(span + 1 - order)];
    right[static_cast<std::size_t>(order)] =
        knots[static_cast<std::size_t>(span + order)] - u;
    double saved = 0.0;
    for (int j = 0; j < order; ++j) {
      const double denominator = right[static_cast<std::size_t>(j + 1)] +
                                 left[static_cast<std::size_t>(order - j)];
      const double term = denominator > kNumericalEpsilon
                              ? local_basis[static_cast<std::size_t>(j)] / denominator
                              : 0.0;
      local_basis[static_cast<std::size_t>(j)] =
          saved + right[static_cast<std::size_t>(j + 1)] * term;
      saved = left[static_cast<std::size_t>(order - j)] * term;
    }
    local_basis[static_cast<std::size_t>(order)] = saved;
  }

  for (int j = 0; j <= degree; ++j) {
    result(span - degree + j) = local_basis[static_cast<std::size_t>(j)];
  }
  return result;
}

Eigen::Vector3d UniformBspline::evaluate(double time, int derivative_order) const {
  if (!isFinite(time)) {
    throw std::invalid_argument("B-spline evaluation time must be finite");
  }
  if (derivative_order < 0) {
    throw std::invalid_argument("B-spline derivative order must be non-negative");
  }
  if (derivative_order > degree_) {
    return Eigen::Vector3d::Zero();
  }
  const UniformBspline derivative_spline = derivative(derivative_order);
  return evaluateCurve(derivative_spline.control_points_, derivative_spline.degree_,
                       derivative_spline.knots_, time / dt_);
}

UniformBspline UniformBspline::derivative(int order) const {
  if (order < 0 || order > degree_) {
    throw std::invalid_argument("B-spline derivative order is outside the valid range");
  }
  if (order == 0) {
    return *this;
  }

  ControlPointMatrix derived_control_points = control_points_;
  std::vector<double> derived_knots = knots_;
  int derived_degree = degree_;
  for (int derivative_index = 0; derivative_index < order; ++derivative_index) {
    const int current_count = static_cast<int>(derived_control_points.cols());
    ControlPointMatrix next_control_points(3, current_count - 1);
    for (int i = 0; i < current_count - 1; ++i) {
      const double denominator =
          derived_knots[static_cast<std::size_t>(i + derived_degree + 1)] -
          derived_knots[static_cast<std::size_t>(i + 1)];
      if (denominator <= kNumericalEpsilon) {
        throw std::logic_error("degenerate B-spline derivative knot interval");
      }
      next_control_points.col(i) =
          (static_cast<double>(derived_degree) / (denominator * dt_)) *
          (derived_control_points.col(i + 1) - derived_control_points.col(i));
    }
    derived_control_points = std::move(next_control_points);
    derived_knots.erase(derived_knots.begin());
    derived_knots.pop_back();
    --derived_degree;
  }

  return UniformBspline(std::move(derived_control_points), dt_, derived_degree,
                        std::move(derived_knots), knot_mode_, DerivedSplineTag{});
}

Eigen::VectorXd UniformBspline::basisFunctions(double time) const {
  return evaluateBasis(knots_, degree_, controlPointCount(), time / dt_);
}

UniformBspline::ControlPointMatrix UniformBspline::parameterizeToControlPoints(
    const std::vector<Eigen::Vector3d> &points, double dt,
    const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &end_velocity,
    const Eigen::Vector3d &start_acceleration,
    const Eigen::Vector3d &end_acceleration) {
  if (!isFinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("B-spline parameterization dt must be finite and positive");
  }
  if (points.size() < 4U) {
    throw std::invalid_argument("B-spline parameterization requires at least four points");
  }
  if (!start_velocity.allFinite() || !end_velocity.allFinite() ||
      !start_acceleration.allFinite() || !end_acceleration.allFinite()) {
    throw std::invalid_argument("B-spline boundary derivatives must be finite");
  }
  for (const Eigen::Vector3d &point : points) {
    if (!point.allFinite()) {
      throw std::invalid_argument("B-spline parameterization points must be finite");
    }
  }

  const int point_count = static_cast<int>(points.size());
  const int control_point_count = point_count + 2;
  Eigen::MatrixXd position_system = Eigen::MatrixXd::Zero(point_count, control_point_count);
  constexpr int kBoundaryConstraintCount = 6;
  Eigen::MatrixXd boundary_system =
      Eigen::MatrixXd::Zero(kBoundaryConstraintCount, control_point_count);
  const Eigen::RowVector3d position_row(1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0);
  const Eigen::RowVector3d velocity_row(-1.0 / (2.0 * dt), 0.0, 1.0 / (2.0 * dt));
  const Eigen::RowVector3d acceleration_row(1.0 / (dt * dt), -2.0 / (dt * dt),
                                             1.0 / (dt * dt));
  for (int point = 0; point < point_count; ++point) {
    position_system.block(point, point, 1, 3) = position_row;
  }
  boundary_system.block(0, 0, 1, 3) = position_row;
  boundary_system.block(1, 0, 1, 3) = velocity_row;
  boundary_system.block(2, 0, 1, 3) = acceleration_row;
  boundary_system.block(3, point_count - 1, 1, 3) = position_row;
  boundary_system.block(4, point_count - 1, 1, 3) = velocity_row;
  boundary_system.block(5, point_count - 1, 1, 3) = acceleration_row;

  Eigen::MatrixXd position_values(point_count, 3);
  for (int point = 0; point < point_count; ++point) {
    position_values.row(point) = points[static_cast<std::size_t>(point)].transpose();
  }
  Eigen::MatrixXd boundary_values(kBoundaryConstraintCount, 3);
  boundary_values.row(0) = points.front().transpose();
  boundary_values.row(1) = start_velocity.transpose();
  boundary_values.row(2) = start_acceleration.transpose();
  boundary_values.row(3) = points.back().transpose();
  boundary_values.row(4) = end_velocity.transpose();
  boundary_values.row(5) = end_acceleration.transpose();

  // Keep the waypoint equations as a least-squares fit while enforcing the
  // six endpoint state equations exactly. A null-space least-squares
  // solve avoids the poor conditioning introduced by normal equations.
  Eigen::JacobiSVD<Eigen::MatrixXd> boundary_svd(
      boundary_system, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd singular_values = boundary_svd.singularValues();
  if (singular_values.size() < kBoundaryConstraintCount ||
      singular_values(kBoundaryConstraintCount - 1) <= kNumericalEpsilon) {
    throw std::runtime_error("B-spline parameterization matrix is rank deficient");
  }
  Eigen::VectorXd inverse_singular_values = Eigen::VectorXd::Zero(kBoundaryConstraintCount);
  for (int index = 0; index < kBoundaryConstraintCount; ++index) {
    inverse_singular_values(index) = 1.0 / singular_values(index);
  }
  const Eigen::MatrixXd boundary_pseudoinverse =
      boundary_svd.matrixV().leftCols(kBoundaryConstraintCount) *
      inverse_singular_values.asDiagonal() *
      boundary_svd.matrixU().leftCols(kBoundaryConstraintCount).transpose();
  const Eigen::MatrixXd particular = boundary_pseudoinverse * boundary_values;
  const Eigen::MatrixXd null_space =
      boundary_svd.matrixV().rightCols(control_point_count - kBoundaryConstraintCount);
  const Eigen::MatrixXd projected_system = position_system * null_space;
  const Eigen::MatrixXd projected_values = position_values - position_system * particular;
  Eigen::MatrixXd solution = particular;
  if (projected_system.cols() > 0) {
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(projected_system);
    if (qr.rank() < projected_system.cols()) {
      throw std::runtime_error("B-spline parameterization position matrix is rank deficient");
    }
    solution += null_space * qr.solve(projected_values);
  }
  const ControlPointMatrix control_points = solution.topRows(control_point_count).transpose();
  if (!control_points.allFinite()) {
    throw std::runtime_error("B-spline parameterization produced non-finite control points");
  }
  return control_points;
}

}  // namespace aurora::math
