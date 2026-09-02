// Copyright 2026 PathAlgo26
#include "aurora_risk/risk_context.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aurora::risk::MapQualitySample;
using aurora::risk::MapRiskState;
using aurora::risk::RiskContext;
using aurora::risk::RiskContextStatus;
using aurora::risk::validateRiskContext;

TEST(RiskContext, DefaultContextIsValidAndMissingInputsRemainExplicit) {
  const RiskContext context;
  const auto result = validateRiskContext(context);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.status, RiskContextStatus::VALID);
  EXPECT_FALSE(context.map.available);
  EXPECT_FALSE(context.vehicle.has_localization_position_covariance);
  EXPECT_FALSE(context.vehicle.has_execution_position_covariance);
}

TEST(RiskContext, RejectsInvalidCovarianceTolerance) {
  RiskContext context;

  EXPECT_EQ(validateRiskContext(context, -1.0).status,
            RiskContextStatus::INVALID_OPTIONS);
  EXPECT_EQ(validateRiskContext(context, std::numeric_limits<double>::quiet_NaN()).status,
            RiskContextStatus::INVALID_OPTIONS);
}

TEST(RiskContext, AcceptsSymmetricPositiveSemidefiniteVehicleCovariances) {
  RiskContext context;
  context.vehicle.has_localization_position_covariance = true;
  context.vehicle.localization_position_covariance =
      (Eigen::Vector3d(1.0, 2.0, 3.0)).asDiagonal();
  context.vehicle.has_execution_position_covariance = true;
  context.vehicle.execution_position_covariance = Eigen::Matrix3d::Zero();

  const auto result = validateRiskContext(context);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.status, RiskContextStatus::VALID);
}

TEST(RiskContext, RejectsNonFiniteAsymmetricAndNegativeCovariances) {
  RiskContext context;
  context.vehicle.has_localization_position_covariance = true;
  context.vehicle.localization_position_covariance(0, 0) =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validateRiskContext(context).status,
            RiskContextStatus::INVALID_VEHICLE_UNCERTAINTY);

  context.vehicle.localization_position_covariance.setZero();
  context.vehicle.localization_position_covariance(0, 1) = 0.1;
  EXPECT_EQ(validateRiskContext(context).status,
            RiskContextStatus::INVALID_VEHICLE_UNCERTAINTY);

  context.vehicle.localization_position_covariance.setZero();
  context.vehicle.localization_position_covariance(0, 0) = -0.1;
  EXPECT_EQ(validateRiskContext(context).status,
            RiskContextStatus::INVALID_VEHICLE_UNCERTAINTY);
}

TEST(RiskContext, AcceptsUnobservedAgeOnlyForUnknownOrOutOfMap) {
  RiskContext context;
  context.map.available = true;
  context.map.snapshot_stamp = 10.0;
  context.map.map_version = 4U;
  context.map.samples.push_back(MapQualitySample{
      10.0, Eigen::Vector3d::Zero(), MapRiskState::UNKNOWN, 0.5,
      std::numeric_limits<double>::infinity(), 0.0, false, 4U});

  EXPECT_TRUE(validateRiskContext(context).valid);

  context.map.samples.front().state = MapRiskState::FREE;
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_MAP);
}

TEST(RiskContext, RejectsMissingMapDataAndVersionMismatch) {
  RiskContext context;
  context.map.available = true;
  context.map.snapshot_stamp = 10.0;
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_MAP);

  context.map.samples.push_back(
      MapQualitySample{10.0, Eigen::Vector3d::Zero(), MapRiskState::FREE, 0.1, 0.0, 1.0,
                       false, 3U});
  context.map.map_version = 4U;
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_MAP);
}

TEST(RiskContext, RejectsUnavailableMapWithAttachedData) {
  RiskContext context;
  context.map.snapshot_stamp = 10.0;
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_MAP);

  context = RiskContext{};
  context.map.samples.push_back(
      MapQualitySample{10.0, Eigen::Vector3d::Zero(), MapRiskState::FREE, 0.1, 0.0, 1.0,
                       false, 0U});
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_MAP);
}

TEST(RiskContext, RejectsNegativeOrNonFiniteDelay) {
  RiskContext context;
  context.delay.tracking_delay = -0.01;
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_DELAY);

  context.delay = {};
  context.delay.execution_delay = std::numeric_limits<double>::infinity();
  EXPECT_EQ(validateRiskContext(context).status, RiskContextStatus::INVALID_DELAY);
}

}  // namespace
