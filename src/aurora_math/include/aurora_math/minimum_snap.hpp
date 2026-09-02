#pragma once

#include <Eigen/Core>

#include <vector>

namespace aurora::math {

// EGO-compatible piecewise quintic waypoint trajectory. The public name keeps
// the upstream API concept; the quintic smoothness functional is the
// integrated squared jerk, which is the derivative order available in the
// upstream five-degree implementation.
class MinimumSnapTrajectory {
public:
  using WaypointMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;
  using CoefficientMatrix = Eigen::Matrix<double, 3, 6>;

  static MinimumSnapTrajectory fromWaypoints(const WaypointMatrix &waypoints,
                                             const Eigen::Vector3d &start_velocity,
                                             const Eigen::Vector3d &end_velocity,
                                             const Eigen::Vector3d &start_acceleration,
                                             const Eigen::Vector3d &end_acceleration,
                                             const Eigen::VectorXd &segment_times);

  static MinimumSnapTrajectory oneSegment(const Eigen::Vector3d &start_position,
                                          const Eigen::Vector3d &start_velocity,
                                          const Eigen::Vector3d &start_acceleration,
                                          const Eigen::Vector3d &end_position,
                                          const Eigen::Vector3d &end_velocity,
                                          const Eigen::Vector3d &end_acceleration,
                                          double duration);

  int segmentCount() const noexcept { return static_cast<int>(coefficients_.size()); }
  double duration() const noexcept { return total_duration_; }
  const Eigen::VectorXd &segmentTimes() const noexcept { return segment_times_; }
  const std::vector<CoefficientMatrix> &coefficients() const noexcept { return coefficients_; }

  // time is clamped to [0, duration]. Derivatives use physical seconds.
  // Orders above five are identically zero for a quintic trajectory.
  Eigen::Vector3d evaluate(double time, int derivative_order = 0) const;

  // Integral of squared third derivative over all segments.
  double jerkCost() const noexcept;

private:
  MinimumSnapTrajectory(std::vector<CoefficientMatrix> coefficients,
                        Eigen::VectorXd segment_times);

  static void validateInputs(const WaypointMatrix &waypoints,
                             const Eigen::Vector3d &start_velocity,
                             const Eigen::Vector3d &end_velocity,
                             const Eigen::Vector3d &start_acceleration,
                             const Eigen::Vector3d &end_acceleration,
                             const Eigen::VectorXd &segment_times);

  std::vector<CoefficientMatrix> coefficients_;
  Eigen::VectorXd segment_times_;
  Eigen::VectorXd cumulative_end_times_;
  double total_duration_{0.0};
};

}  // namespace aurora::math
