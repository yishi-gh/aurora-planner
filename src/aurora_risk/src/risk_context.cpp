#include "aurora_risk/risk_context.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>

namespace aurora::risk {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool validCovariance(const Covariance3 &covariance, double tolerance) {
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > tolerance) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Covariance3> solver(
      0.5 * (covariance + covariance.transpose()));
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -tolerance;
}

bool validMapState(MapRiskState state) {
  switch (state) {
    case MapRiskState::FREE:
    case MapRiskState::OCCUPIED:
    case MapRiskState::UNKNOWN:
    case MapRiskState::OUT_OF_MAP:
      return true;
  }
  return false;
}

bool validObservationAge(double age, MapRiskState state) {
  if (finite(age) && age >= 0.0) {
    return true;
  }
  // The map core represents unobserved/invalid spatial cells with positive
  // infinity. Keep that representation valid so policy code can reject it as
  // UNKNOWN or OUT_OF_MAP instead of misclassifying it as malformed input.
  return std::isinf(age) && age > 0.0 &&
         (state == MapRiskState::UNKNOWN || state == MapRiskState::OUT_OF_MAP);
}

RiskContextValidation failure(RiskContextStatus status, const char *detail) {
  RiskContextValidation result;
  result.status = status;
  result.valid = false;
  result.detail = detail;
  return result;
}

}  // namespace

const char *toString(RiskContextStatus status) noexcept {
  switch (status) {
    case RiskContextStatus::VALID:
      return "VALID";
    case RiskContextStatus::INVALID_OPTIONS:
      return "INVALID_OPTIONS";
    case RiskContextStatus::INVALID_MAP:
      return "INVALID_MAP";
    case RiskContextStatus::INVALID_VEHICLE_UNCERTAINTY:
      return "INVALID_VEHICLE_UNCERTAINTY";
    case RiskContextStatus::INVALID_DELAY:
      return "INVALID_DELAY";
  }
  return "UNKNOWN_STATUS";
}

RiskContextValidation validateRiskContext(const RiskContext &context,
                                          double covariance_tolerance) {
  if (!finite(covariance_tolerance) || covariance_tolerance < 0.0) {
    return failure(RiskContextStatus::INVALID_OPTIONS,
                   "covariance tolerance must be finite and non-negative");
  }

  if (context.vehicle.has_localization_position_covariance &&
      !validCovariance(context.vehicle.localization_position_covariance,
                       covariance_tolerance)) {
    return failure(RiskContextStatus::INVALID_VEHICLE_UNCERTAINTY,
                   "localization position covariance is not finite, symmetric and positive semidefinite");
  }
  if (context.vehicle.has_execution_position_covariance &&
      !validCovariance(context.vehicle.execution_position_covariance,
                       covariance_tolerance)) {
    return failure(RiskContextStatus::INVALID_VEHICLE_UNCERTAINTY,
                   "execution position covariance is not finite, symmetric and positive semidefinite");
  }

  const auto &map = context.map;
  if (!map.available) {
    if (finite(map.snapshot_stamp) || !map.samples.empty()) {
      return failure(RiskContextStatus::INVALID_MAP,
                     "map context is unavailable but contains snapshot data");
    }
  } else {
    if (!finite(map.snapshot_stamp) || map.samples.empty()) {
      return failure(RiskContextStatus::INVALID_MAP,
                     "available map context must contain a finite stamp and samples");
    }
    for (const auto &sample : map.samples) {
      if (!finite(sample.stamp) || !sample.position.allFinite() || !validMapState(sample.state) ||
          !finite(sample.occupancy_probability) || sample.occupancy_probability < 0.0 ||
          sample.occupancy_probability > 1.0 ||
          !validObservationAge(sample.observation_age, sample.state) ||
          !finite(sample.confidence) || sample.confidence < 0.0 || sample.confidence > 1.0 ||
          sample.map_version != map.map_version) {
        return failure(RiskContextStatus::INVALID_MAP,
                       "map quality sample contains an invalid value or version");
      }
    }
  }

  const auto &delay = context.delay;
  if (!finite(delay.sensing_delay) || delay.sensing_delay < 0.0 ||
      !finite(delay.tracking_delay) || delay.tracking_delay < 0.0 ||
      !finite(delay.planning_delay) || delay.planning_delay < 0.0 ||
      !finite(delay.execution_delay) || delay.execution_delay < 0.0 ||
      !finite(delay.safety_margin) || delay.safety_margin < 0.0 ||
      !finite(delay.total())) {
    return failure(RiskContextStatus::INVALID_DELAY,
                   "delay components and total delay must be finite and non-negative");
  }

  return {};
}

}  // namespace aurora::risk
