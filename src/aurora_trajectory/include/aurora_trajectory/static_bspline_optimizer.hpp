#pragma once

#include "aurora_map/voxel_map.hpp"
#include "aurora_math/uniform_bspline.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace aurora::trajectory {

using ControlPointMatrix = aurora::math::UniformBspline::ControlPointMatrix;

// A ROS-independent soft-risk hook. The callback time is an absolute world
// timestamp; the position gradient is with respect to the queried position.
// Hard safety decisions remain outside this hook and are always rechecked.
struct RiskCostEvaluation {
  bool valid{true};
  double value{0.0};
  Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
  std::string detail;
};

using RiskCostFunction = std::function<RiskCostEvaluation(
    double absolute_stamp, const Eigen::Vector3d &position)>;

struct StaticOptimizerOptions {
  double interval{0.25};
  double clearance{0.65};
  double max_velocity{3.0};
  double max_acceleration{6.0};

  double lambda_smooth{0.08};
  double lambda_obstacle{35.0};
  double lambda_feasibility{0.15};
  double lambda_fitness{0.01};
  double lambda_risk{0.0};
  double risk_time_origin{0.0};
  std::size_t max_risk_evaluations{10000};
  RiskCostFunction risk_cost;

  int max_iterations{180};
  // Zero disables the wall-clock budget. A non-zero budget is a best-effort
  // deadline checked between expensive objective evaluations.
  double max_compute_time_sec{0.0};
  int max_line_search_iterations{14};
  std::size_t samples_per_span{8};
  double initial_step{0.18};
  double gradient_clip{4.0};
  double convergence_gradient_norm{1e-5};
  double improvement_tolerance{1e-9};
  aurora::math::UniformBsplineKnotMode knot_mode{
      aurora::math::UniformBsplineKnotMode::EGO_UNCLAMPED};
};

enum class OptimizationStatus {
  CONVERGED,
  MAX_ITERATIONS,
  STALLED,
  TIMEOUT,
};

const char *toString(OptimizationStatus status) noexcept;

struct CostBreakdown {
  double total{0.0};
  double smoothness{0.0};
  double obstacle{0.0};
  double feasibility{0.0};
  double fitness{0.0};
  double risk{0.0};
  bool risk_available{true};
  bool risk_evaluation_failed{false};
  std::string risk_detail;
  ControlPointMatrix gradient;
};

struct OptimizationResult {
  OptimizationStatus status{OptimizationStatus::STALLED};
  ControlPointMatrix control_points;
  CostBreakdown cost;
  int iterations{0};
  bool risk_enabled{false};
  bool risk_fallback{false};
};

// Static EGO-style B-spline optimizer. The map is treated as an immutable
// snapshot for the lifetime of this optimizer.
class StaticBsplineOptimizer {
public:
  StaticBsplineOptimizer(const aurora::map::VoxelMap &map,
                         ControlPointMatrix initial_control_points,
                         ControlPointMatrix reference_control_points,
                         StaticOptimizerOptions options = {});

  const ControlPointMatrix &controlPoints() const noexcept { return control_points_; }
  const StaticOptimizerOptions &options() const noexcept { return options_; }

  CostBreakdown evaluate(const ControlPointMatrix &control_points,
                         bool with_gradient = true) const;
  OptimizationResult optimize();

private:
  struct ObstaclePotential {
    double value{0.0};
    Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
  };

  static void validateOptions(const StaticOptimizerOptions &options);
  static void validateControlPoints(const ControlPointMatrix &control_points,
                                    const char *name);
  static std::vector<Eigen::Vector3d> collectOccupiedCenters(
      const aurora::map::VoxelMap &map);
  ObstaclePotential obstaclePotential(const Eigen::Vector3d &position) const;
  CostBreakdown evaluateInternal(const ControlPointMatrix &control_points,
                                 bool with_gradient, bool include_risk) const;

  const aurora::map::VoxelMap &map_;
  StaticOptimizerOptions options_;
  ControlPointMatrix control_points_;
  ControlPointMatrix reference_control_points_;
  std::vector<Eigen::Vector3d> occupied_centers_;
};

enum class ValidationStatus {
  VALID,
  INVALID_OPTIONS,
  NONFINITE,
  OUT_OF_MAP,
  OCCUPIED,
  UNKNOWN,
  VELOCITY_LIMIT,
  ACCELERATION_LIMIT,
};

const char *toString(ValidationStatus status) noexcept;

struct StaticTrajectoryValidationOptions {
  std::size_t samples_per_span{16};
  double max_velocity{3.0};
  double max_acceleration{6.0};
  double tolerance{1e-6};
  bool reject_unknown{true};
};

struct StaticTrajectoryValidationResult {
  ValidationStatus status{ValidationStatus::VALID};
  bool valid{false};
  std::size_t checked_samples{0};
  std::size_t occupied_samples{0};
  std::size_t unknown_samples{0};
  double maximum_velocity{0.0};
  double maximum_acceleration{0.0};
  std::string detail;
};

StaticTrajectoryValidationResult validateStaticTrajectory(
    const aurora::map::VoxelMap &map,
    const aurora::math::UniformBspline &spline,
    const StaticTrajectoryValidationOptions &options = {});

// Validates only [start_time, start_time + duration] of a spline. This is
// used when a new plan retains a time-windowed prefix of an older trajectory.
StaticTrajectoryValidationResult validateStaticTrajectoryWindow(
    const aurora::map::VoxelMap &map,
    const aurora::math::UniformBspline &spline,
    double start_time,
    double duration,
    const StaticTrajectoryValidationOptions &options = {});

}  // namespace aurora::trajectory
