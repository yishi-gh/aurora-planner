#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace aurora::risk {

using Covariance3 = Eigen::Matrix3d;

// This is deliberately independent from ROS and from the map implementation.
// Adapters translate their native state into this small risk-facing contract.
enum class MapRiskState {
  FREE,
  OCCUPIED,
  UNKNOWN,
  OUT_OF_MAP,
};

struct MapQualitySample {
  double stamp{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  MapRiskState state{MapRiskState::UNKNOWN};
  double occupancy_probability{std::numeric_limits<double>::quiet_NaN()};
  double observation_age{std::numeric_limits<double>::infinity()};
  double confidence{std::numeric_limits<double>::quiet_NaN()};
  bool inflated{false};
  std::uint64_t map_version{0U};
};

struct MapQualityContext {
  bool available{false};
  double snapshot_stamp{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t map_version{0U};
  std::vector<MapQualitySample> samples;
};

// Localization and execution uncertainty remain separate at the boundary so
// diagnostics can identify their source. Risk evaluation may combine them as
// independent position covariance terms after validation.
struct VehicleUncertaintyInput {
  bool has_localization_position_covariance{false};
  Covariance3 localization_position_covariance{Covariance3::Zero()};
  bool has_execution_position_covariance{false};
  Covariance3 execution_position_covariance{Covariance3::Zero()};
};

struct DelayBudget {
  // All values are seconds and must be finite and non-negative when supplied.
  double sensing_delay{0.0};
  double tracking_delay{0.0};
  double planning_delay{0.0};
  double execution_delay{0.0};
  double safety_margin{0.0};

  double total() const noexcept {
    return sensing_delay + tracking_delay + planning_delay + execution_delay +
           safety_margin;
  }
};

struct RiskContext {
  MapQualityContext map;
  VehicleUncertaintyInput vehicle;
  DelayBudget delay;
};

enum class RiskContextStatus {
  VALID,
  INVALID_OPTIONS,
  INVALID_MAP,
  INVALID_VEHICLE_UNCERTAINTY,
  INVALID_DELAY,
};

struct RiskContextValidation {
  RiskContextStatus status{RiskContextStatus::VALID};
  bool valid{true};
  std::string detail;
};

const char *toString(RiskContextStatus status) noexcept;

// Validate an optional context without changing or regularizing its values.
// A missing map or covariance is valid and is handled by the risk policy as a
// separate degradation case. Supplied values must be physically well-formed.
RiskContextValidation validateRiskContext(const RiskContext &context,
                                          double covariance_tolerance = 1e-9);

}  // namespace aurora::risk
