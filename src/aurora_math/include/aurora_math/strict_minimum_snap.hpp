#pragma once

#include <Eigen/Core>

#include <array>
#include <vector>

namespace aurora::math {

// Piecewise degree-seven polynomial that minimizes integrated squared snap
// for fixed segment times and waypoint/boundary constraints. Coefficients are
// expressed in physical seconds, so derivatives are physical quantities.
class StrictMinimumSnapTrajectory {
public:
  using WaypointMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using CoefficientMatrix = Eigen::Matrix<double, 3, 8>;

  static StrictMinimumSnapTrajectory fromWaypoints(
      const WaypointMatrix &waypoints, const Eigen::Vector3d &start_velocity,
      const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
      const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &start_jerk,
      const Eigen::Vector3d &end_jerk, const Eigen::VectorXd &segment_times);

  static StrictMinimumSnapTrajectory oneSegment(
      const Eigen::Vector3d &start_position, const Eigen::Vector3d &start_velocity,
      const Eigen::Vector3d &start_acceleration, const Eigen::Vector3d &start_jerk,
      const Eigen::Vector3d &end_position, const Eigen::Vector3d &end_velocity,
      const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &end_jerk,
      double duration);

  int segmentCount() const noexcept { return static_cast<int>(coefficients_.size()); }
  double duration() const noexcept { return total_duration_; }
  const Eigen::VectorXd &segmentTimes() const noexcept { return segment_times_; }
  const std::vector<CoefficientMatrix> &coefficients() const noexcept { return coefficients_; }

  // Time is clamped to [0, duration]. Derivative order is with respect to
  // physical seconds. Orders above seven are identically zero.
  Eigen::Vector3d evaluate(double time, int derivative_order = 0) const;

  // Integral of squared fourth derivative over all segments.
  double snapCost() const noexcept;

private:
  StrictMinimumSnapTrajectory(std::vector<CoefficientMatrix> coefficients,
                              std::vector<CoefficientMatrix> normalized_coefficients,
                              std::array<Eigen::Vector3d, 4> start_boundary,
                              std::array<Eigen::Vector3d, 4> end_boundary,
                              Eigen::VectorXd segment_times);

  static void validateInputs(
      const WaypointMatrix &waypoints, const Eigen::Vector3d &start_velocity,
      const Eigen::Vector3d &end_velocity, const Eigen::Vector3d &start_acceleration,
      const Eigen::Vector3d &end_acceleration, const Eigen::Vector3d &start_jerk,
      const Eigen::Vector3d &end_jerk, const Eigen::VectorXd &segment_times);

  std::vector<CoefficientMatrix> coefficients_;
  // q(s), where s is local time divided by the segment duration. Evaluation
  // uses this form to avoid cancellation for long physical-time segments.
  std::vector<CoefficientMatrix> normalized_coefficients_;
  std::array<Eigen::Vector3d, 4> start_boundary_;
  std::array<Eigen::Vector3d, 4> end_boundary_;
  Eigen::VectorXd segment_times_;
  Eigen::VectorXd cumulative_end_times_;
  double total_duration_{0.0};
};

}  // namespace aurora::math
