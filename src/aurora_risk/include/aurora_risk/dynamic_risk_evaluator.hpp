#pragma once

#include "aurora_prediction/kinematic_predictor.hpp"
#include "aurora_risk/risk_context.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace aurora::risk {

enum class RiskStatus {
  ACCEPTED,
  INVALID_OPTIONS,
  INVALID_INPUT,
  NO_DYNAMIC_INFORMATION,
  INFORMATION_STALE,
  PREDICTION_INVALID,
  NO_MAP_INFORMATION,
  MAP_UNKNOWN,
  MAP_OUT_OF_MAP,
  MAP_COLLISION,
  MAP_RISK,
  DYNAMIC_COLLISION,
  RISK_LIMIT,
};

enum class RiskLevel {
  LOW,
  MEDIUM,
  HIGH,
  UNKNOWN,
};

const char *toString(RiskStatus status) noexcept;
const char *toString(RiskLevel level) noexcept;

struct TrajectorySample {
  double stamp{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Covariance3 position_covariance{Covariance3::Zero()};
};

// A batch is valid only when its source message was present. invalid_track_count
// is intentionally retained: silently dropping one malformed target would
// otherwise turn an incomplete observation into a false safety conclusion.
struct DynamicRiskInput {
  bool has_snapshot{false};
  double snapshot_stamp{std::numeric_limits<double>::quiet_NaN()};
  // Optional time at which freshness is evaluated. When absent, trajectory
  // start time remains the alignment reference for backwards compatibility.
  double evaluation_stamp{std::numeric_limits<double>::quiet_NaN()};
  std::vector<aurora::prediction::PredictionResult> predictions;
  std::size_t invalid_track_count{0U};
  bool occlusion_active{false};
  std::size_t occluded_track_count{0U};
  // The context is carried through the same immutable input snapshot. The
  // current evaluator keeps the established dynamic baseline behavior; later
  // risk-policy stages consume these fields explicitly.
  RiskContext context;
};

struct DynamicRiskEvaluation {
  RiskStatus status{RiskStatus::INVALID_INPUT};
  RiskLevel level{RiskLevel::UNKNOWN};
  bool accepted{false};
  double total_risk{std::numeric_limits<double>::quiet_NaN()};
  double dynamic_risk{std::numeric_limits<double>::quiet_NaN()};
  double map_risk{std::numeric_limits<double>::quiet_NaN()};
  double information_risk{std::numeric_limits<double>::quiet_NaN()};
  double minimum_clearance{std::numeric_limits<double>::quiet_NaN()};
  double information_age{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t worst_obstacle_id{-1};
  double worst_time{std::numeric_limits<double>::quiet_NaN()};
  std::size_t checked_samples{0U};
  std::size_t checked_obstacles{0U};
  std::string detail;
};

struct DynamicRiskEvaluatorOptions {
  // The vehicle and obstacle are conservatively reduced to spheres.
  double vehicle_radius{0.65};
  double sigma_multiplier{3.0};

  double time_tolerance{1e-6};
  double max_prediction_age{0.5};
  double warning_clearance{0.5};
  bool require_map_quality{false};
  bool allow_unknown_space{false};
  double map_free_probability{0.2};
  double map_occupancy_threshold{0.8};
  double map_max_observation_age{1.0};
  double map_occupancy_weight{0.5};
  double map_age_weight{0.25};
  double map_confidence_weight{0.25};
  double map_risk_limit{1.0};
  std::size_t max_samples{100000};
  std::size_t max_obstacles{1000};
  bool require_dynamic_information{true};
};

class DynamicRiskEvaluator {
public:
  explicit DynamicRiskEvaluator(DynamicRiskEvaluatorOptions options = {});

  const DynamicRiskEvaluatorOptions &options() const noexcept { return options_; }

  DynamicRiskEvaluation evaluate(const std::vector<TrajectorySample> &trajectory,
                                 const DynamicRiskInput &input) const;

private:
  static void validateOptions(const DynamicRiskEvaluatorOptions &options);

  DynamicRiskEvaluatorOptions options_;
};

}  // namespace aurora::risk
