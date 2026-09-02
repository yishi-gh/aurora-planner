#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace aurora::math {

enum class UniformBsplineKnotMode {
  CLAMPED,
  EGO_UNCLAMPED,
};

class UniformBspline {
public:
  using ControlPointMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;

  // The public constructor creates a cubic, clamped uniform B-spline.
  explicit UniformBspline(ControlPointMatrix control_points, double dt,
                          UniformBsplineKnotMode knot_mode = UniformBsplineKnotMode::CLAMPED);

  int degree() const noexcept { return degree_; }
  int controlPointCount() const noexcept { return static_cast<int>(control_points_.cols()); }
  double dt() const noexcept { return dt_; }
  double duration() const noexcept;
  UniformBsplineKnotMode knotMode() const noexcept { return knot_mode_; }
  const ControlPointMatrix &controlPoints() const noexcept { return control_points_; }

  // derivative_order is with respect to physical time, not normalized knot time.
  Eigen::Vector3d evaluate(double time, int derivative_order = 0) const;

  // Returns the derivative spline. Its degree is cubic - order and its duration is unchanged.
  UniformBspline derivative(int order) const;

  // Returns the non-zero cubic basis values in a vector indexed by the original control points.
  Eigen::VectorXd basisFunctions(double time) const;

  // EGO-compatible parameterization. Waypoint equations are fitted in least
  // squares while both endpoint positions and endpoint derivatives are exact.
  // The returned matrix has K + 2 columns for K input points and must be used
  // with EGO_UNCLAMPED mode.
  static ControlPointMatrix parameterizeToControlPoints(
      const std::vector<Eigen::Vector3d> &points, double dt,
      const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &end_velocity,
      const Eigen::Vector3d &start_acceleration,
      const Eigen::Vector3d &end_acceleration);

private:
  struct DerivedSplineTag {};

  UniformBspline(ControlPointMatrix control_points, double dt, int degree,
                 std::vector<double> knots, UniformBsplineKnotMode knot_mode,
                 DerivedSplineTag);

  static void validateControlPoints(const ControlPointMatrix &control_points, int degree);
  static std::vector<double> makeClampedUniformKnots(int control_point_count, int degree);
  static std::vector<double> makeEgoUniformKnots(int control_point_count, int degree);
  static Eigen::Vector3d evaluateCurve(const ControlPointMatrix &control_points, int degree,
                                       const std::vector<double> &knots, double normalized_time);
  static Eigen::VectorXd evaluateBasis(const std::vector<double> &knots, int degree,
                                       int control_point_count, double normalized_time);
  static int findSpan(const std::vector<double> &knots, int degree, int control_point_count,
                      double normalized_time);
  static double clampNormalizedTime(const std::vector<double> &knots, int degree,
                                    int control_point_count, double normalized_time);

  ControlPointMatrix control_points_;
  double dt_{0.0};
  int degree_{3};
  UniformBsplineKnotMode knot_mode_{UniformBsplineKnotMode::CLAMPED};
  std::vector<double> knots_;
};

}  // namespace aurora::math
