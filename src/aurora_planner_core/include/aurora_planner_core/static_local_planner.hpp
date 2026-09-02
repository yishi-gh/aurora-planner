#pragma once

#include "aurora_map/voxel_map.hpp"
#include "aurora_math/minimum_snap.hpp"
#include "aurora_math/path_resampler.hpp"
#include "aurora_math/strict_minimum_snap.hpp"
#include "aurora_math/time_allocation.hpp"
#include "aurora_math/uniform_bspline.hpp"
#include "aurora_search/astar_3d.hpp"
#include "aurora_trajectory/static_bspline_optimizer.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace aurora::planner {

struct VehicleState {
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool has_position_covariance{false};
  Eigen::Matrix3d position_covariance{Eigen::Matrix3d::Zero()};
};

struct ReferencePoint {
  double time_from_start{0.0};
  bool has_time{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

// Options for constructing a time-parameterized global reference with the
// strict seventh-order minimum-snap trajectory. The time allocator provides a
// deterministic initial schedule; it is not a dynamic feasibility proof.
struct StrictMinimumSnapReferenceOptions {
  aurora::math::TimeAllocationOptions time_allocation;
  double sample_interval{0.25};
  Eigen::Vector3d start_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_jerk{Eigen::Vector3d::Zero()};
};

struct GlobalReference {
  std::vector<ReferencePoint> points;

  static GlobalReference fromWaypoints(
      const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &start_velocity = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &end_velocity = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &start_acceleration = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &end_acceleration = Eigen::Vector3d::Zero());

  static GlobalReference fromTrajectory(const aurora::math::MinimumSnapTrajectory &trajectory,
                                        double sample_interval = 0.25);

  static GlobalReference fromTrajectory(
      const aurora::math::StrictMinimumSnapTrajectory &trajectory,
      double sample_interval = 0.25);

  static GlobalReference fromWaypointsWithTimeAllocation(
      const std::vector<Eigen::Vector3d> &waypoints,
      const StrictMinimumSnapReferenceOptions &options = {});
};

enum class LocalHorizonMode {
  DISTANCE,
  TIME,
};

struct LocalPlannerOptions {
  LocalHorizonMode horizon_mode{LocalHorizonMode::DISTANCE};
  double local_horizon{6.0};
  double goal_tolerance{0.25};
  double max_projection_distance{std::numeric_limits<double>::infinity()};

  aurora::math::PathResamplingOptions resampling;
  aurora::search::SearchOptions search;
  aurora::trajectory::StaticOptimizerOptions optimizer;
  aurora::trajectory::StaticTrajectoryValidationOptions validation;

  // A positive duration retains the active, already validated prefix before
  // the newly optimized local segment. Zero disables stitching.
  double stitch_prefix_duration{0.5};
  double stitch_position_tolerance{0.5};
  double stitch_velocity_tolerance{1.0};
  double stitch_acceleration_tolerance{2.0};
};

struct PlanningRequest {
  std::uint64_t request_id{0};
  double planning_stamp{0.0};
  VehicleState vehicle_state;
  GlobalReference global_reference;
};

struct LocalGoal {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double reference_progress{0.0};
  double reference_time{0.0};
  std::size_t source_segment{0};
  bool terminal{false};
};

struct TrajectoryState {
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

// A segment can either be a complete newly generated spline or a time window
// into an older spline. source_start_time is in the segment's spline clock.
struct TrajectorySegment {
  double start_stamp{0.0};
  double source_start_time{0.0};
  double duration{0.0};
  aurora::math::UniformBspline spline;

  TrajectoryState evaluate(double stamp) const;
  double endStamp() const noexcept { return start_stamp + duration; }
};

struct PlannedTrajectory {
  std::uint64_t trajectory_id{0};
  std::uint64_t map_version{0};
  bool validated{false};
  std::vector<TrajectorySegment> segments;

  bool empty() const noexcept { return segments.empty(); }
  double startStamp() const noexcept;
  double endStamp() const noexcept;
  bool contains(double stamp) const noexcept;
  TrajectoryState evaluate(double stamp) const;
  std::vector<TrajectorySegment> slice(double start_stamp, double end_stamp) const;
};

enum class PlanningStatus {
  SUCCESS,
  GOAL_REACHED,
  INVALID_REQUEST,
  NO_GLOBAL_REFERENCE,
  INVALID_HORIZON,
  LOCAL_GOAL_UNAVAILABLE,
  SEARCH_FAILED,
  OPTIMIZATION_FAILED,
  PREFIX_UNSAFE,
  VALIDATION_FAILED,
};

const char *toString(PlanningStatus status) noexcept;

struct PlanningResult {
  PlanningStatus status{PlanningStatus::INVALID_REQUEST};
  std::string detail;
  LocalGoal local_goal;
  aurora::search::SearchResult search;
  aurora::trajectory::OptimizationResult optimization;
  aurora::trajectory::StaticTrajectoryValidationResult validation;
  double initial_cost{0.0};
  double final_cost{0.0};
  double stitch_prefix_duration{0.0};
  bool used_stitch_prefix{false};
  std::optional<PlannedTrajectory> trajectory;
};

enum class PlannerState {
  INIT,
  WAIT_TARGET,
  GENERATE,
  EXECUTE,
  REPLAN,
  DEGRADED,
  EMERGENCY_STOP,
};

enum class ReplanTrigger {
  NONE,
  CURRENT_TRAJECTORY_COLLISION,
  SAFETY_INFORMATION_STALE,
  PLANNING_FAILURE,
  MAP_UPDATED,
  TRAJECTORY_NEAR_END,
  LOCAL_GOAL_EXPIRED,
  DYNAMIC_OBSTACLE_UPDATED,
};

enum class PlannerAction {
  WAIT,
  REQUEST_REPLAN,
  START_PLANNING,
  WAIT_FOR_RESULT,
  ACCEPT_NEW_TRAJECTORY,
  KEEP_CURRENT_TRAJECTORY,
  HOLD_POSITION,
  EMERGENCY_STOP,
  INVALID_INPUT,
};

const char *toString(PlannerState state) noexcept;
const char *toString(ReplanTrigger trigger) noexcept;
const char *toString(PlannerAction action) noexcept;

struct ReplanFsmOptions {
  // A failed plan is allowed to fall back to the current trajectory this many
  // consecutive times before entering DEGRADED.
  std::size_t max_consecutive_failures{3};
  double trajectory_near_end_margin{1.0};
  // Below this remaining trajectory lifetime, the old trajectory is not a
  // sufficient fallback and the FSM requests an emergency stop.
  double emergency_time_remaining{0.5};
};

struct ReplanObservation {
  double now{std::numeric_limits<double>::quiet_NaN()};
  bool has_global_reference{false};
  bool active_trajectory_available{false};
  bool active_trajectory_safe{false};
  double active_trajectory_end_stamp{std::numeric_limits<double>::quiet_NaN()};

  // These flags are intentionally transport-neutral. The static phase sets
  // collision/map flags directly; future risk and prediction adapters map
  // their freshness result to safety_information_stale.
  bool current_trajectory_collision{false};
  bool safety_information_stale{false};
  bool planning_failed{false};
  // One-shot event: a newer dynamic-obstacle snapshot requires the active
  // trajectory to be reconsidered even when it is not currently unsafe.
  bool dynamic_obstacle_updated{false};
  bool map_updated{false};
  bool local_goal_expired{false};
};

struct FsmDecision {
  bool valid{true};
  PlannerState previous_state{PlannerState::INIT};
  PlannerState state{PlannerState::INIT};
  ReplanTrigger trigger{ReplanTrigger::NONE};
  PlannerAction action{PlannerAction::WAIT};
  std::size_t consecutive_failures{0};
  std::string detail;
};

// Deterministic policy state machine for the static planning phase. It does
// not execute a planning request itself; the caller starts planning when the
// decision says START_PLANNING and reports the result back to this object.
class StaticReplanFsm {
public:
  explicit StaticReplanFsm(ReplanFsmOptions options = {});

  const ReplanFsmOptions &options() const noexcept { return options_; }
  PlannerState state() const noexcept { return state_; }
  std::size_t consecutiveFailures() const noexcept { return consecutive_failures_; }

  void reset() noexcept;
  FsmDecision step(const ReplanObservation &observation);
  FsmDecision onPlanningResult(const PlanningResult &result, double now);

private:
  static void validateOptions(const ReplanFsmOptions &options);
  static bool validateObservation(const ReplanObservation &observation,
                                  std::string *detail);
  static ReplanTrigger selectTrigger(const ReplanObservation &observation,
                                     const ReplanFsmOptions &options);
  static double remainingTrajectoryTime(const ReplanObservation &observation);
  FsmDecision makeDecision(PlannerState previous_state, PlannerAction action,
                           ReplanTrigger trigger, std::string detail) const;
  FsmDecision failedPlanningDecision(const PlanningResult &result, double now);

  ReplanFsmOptions options_;
  PlannerState state_{PlannerState::INIT};
  std::size_t consecutive_failures_{0};
  std::optional<ReplanObservation> last_observation_;
};

class StaticLocalPlanner {
public:
  explicit StaticLocalPlanner(LocalPlannerOptions options = {});

  const LocalPlannerOptions &options() const noexcept { return options_; }

  PlanningResult plan(const aurora::map::VoxelMap &map,
                      const PlanningRequest &request,
                      const std::optional<PlannedTrajectory> &previous = std::nullopt,
                      const std::optional<aurora::trajectory::RiskCostFunction> &risk_cost =
                          std::nullopt) const;

  static LocalGoal extractLocalGoal(const PlanningRequest &request,
                                    const LocalPlannerOptions &options);

private:
  struct Projection {
    std::size_t segment{0};
    double fraction{0.0};
    double distance_along_path{0.0};
    double distance_to_path{0.0};
  };

  static void validateOptions(const LocalPlannerOptions &options);
  static void validateRequest(const PlanningRequest &request,
                              const LocalPlannerOptions &options);
  static Projection projectOntoReference(const PlanningRequest &request);
  static ReferencePoint interpolate(const ReferencePoint &from, const ReferencePoint &to,
                                    double fraction);
  static ReferencePoint pointAtDistance(const GlobalReference &reference,
                                        const std::vector<double> &cumulative_lengths,
                                        double distance, std::size_t *segment);
  static ReferencePoint pointAtTime(const GlobalReference &reference, double time,
                                    std::size_t *segment);
  static std::vector<double> cumulativeLengths(const GlobalReference &reference);
  static bool statesClose(const VehicleState &lhs, const TrajectoryState &rhs,
                          const LocalPlannerOptions &options);
  static bool validatePrefix(const aurora::map::VoxelMap &map,
                             const PlannedTrajectory &previous,
                             double start_stamp, double end_stamp,
                             const LocalPlannerOptions &options);

  LocalPlannerOptions options_;
};

}  // namespace aurora::planner
