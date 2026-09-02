// Copyright 2026 PathAlgo26
#include "aurora_risk/risk_cost_field.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

using aurora::prediction::ObstacleShape;
using aurora::prediction::PredictedState;
using aurora::prediction::PredictionModel;
using aurora::prediction::PredictionResult;
using aurora::prediction::PredictionStatus;
using aurora::risk::DynamicRiskCostField;
using aurora::risk::DynamicRiskCostFieldOptions;
using aurora::risk::RiskContext;
using aurora::trajectory::RiskCostEvaluation;

PredictedState makeState(double stamp, const Eigen::Vector3d &position,
                         double existence_probability = 1.0) {
  PredictedState state;
  state.stamp = stamp;
  state.position = position;
  state.velocity.setZero();
  state.acceleration.setZero();
  state.covariance.setZero();
  state.existence_probability = existence_probability;
  state.mode_probability = 1.0;
  state.shape.type = aurora::prediction::ShapeType::SPHERE;
  state.shape.radius = 0.0;
  return state;
}

PredictionResult makePrediction(std::uint64_t track_id,
                                 double first_stamp = 10.0,
                                 double last_stamp = 11.0,
                                 const Eigen::Vector3d &first_position =
                                     Eigen::Vector3d::Zero(),
                                 const Eigen::Vector3d &last_position =
                                     Eigen::Vector3d::Zero(),
                                 double existence_probability = 1.0) {
  PredictionResult prediction;
  prediction.status = PredictionStatus::SUCCESS;
  prediction.track_id = track_id;
  prediction.reference_stamp = first_stamp;
  prediction.model = PredictionModel::CV;
  prediction.states.push_back(
      makeState(first_stamp, first_position, existence_probability));
  prediction.states.push_back(
      makeState(last_stamp, last_position, existence_probability));
  return prediction;
}

DynamicRiskCostFieldOptions testOptions() {
  DynamicRiskCostFieldOptions options;
  options.vehicle_radius = 0.0;
  options.sigma_multiplier = 3.0;
  options.warning_clearance = 1.0;
  return options;
}

TEST(DynamicRiskCostField, UsesAbsoluteTimeWithDelayAndInterpolatesPrediction) {
  RiskContext context;
  context.delay.safety_margin = 0.25;
  DynamicRiskCostField field(
      {makePrediction(7U, 10.0, 11.0, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d(2.0, 0.0, 0.0))},
      context, testOptions());

  const RiskCostEvaluation result = field.evaluate(10.0, Eigen::Vector3d::Zero());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.value, 0.5, 1e-12);
  EXPECT_NEAR(result.gradient.x(), 1.0, 1e-12);
  EXPECT_NEAR(result.gradient.y(), 0.0, 1e-12);
  EXPECT_NEAR(result.gradient.z(), 0.0, 1e-12);
}

TEST(DynamicRiskCostField, CovarianceAndExistenceProbabilityAreMonotonic) {
  const auto prediction = makePrediction(3U, 10.0, 10.0,
                                          Eigen::Vector3d(1.2, 0.0, 0.0),
                                          Eigen::Vector3d(1.2, 0.0, 0.0));
  RiskContext context;
  DynamicRiskCostField field({prediction}, context, testOptions());

  const auto no_uncertainty = field.evaluate(10.0, Eigen::Vector3d::Zero());
  ASSERT_TRUE(no_uncertainty.valid);
  EXPECT_DOUBLE_EQ(no_uncertainty.value, 0.0);

  auto uncertain_prediction = prediction;
  uncertain_prediction.states.front().covariance.block<3, 3>(0, 0).setIdentity();
  uncertain_prediction.states.back().covariance.block<3, 3>(0, 0).setIdentity();
  DynamicRiskCostField uncertain_field({uncertain_prediction}, context, testOptions());
  const auto high_uncertainty = uncertain_field.evaluate(10.0, Eigen::Vector3d::Zero());
  ASSERT_TRUE(high_uncertainty.valid);
  EXPECT_GT(high_uncertainty.value, no_uncertainty.value);

  uncertain_prediction.states.front().existence_probability = 0.0;
  uncertain_prediction.states.back().existence_probability = 0.0;
  DynamicRiskCostField low_probability_field({uncertain_prediction}, context,
                                             testOptions());
  const auto low_probability =
      low_probability_field.evaluate(10.0, Eigen::Vector3d::Zero());
  ASSERT_TRUE(low_probability.valid);
  EXPECT_NEAR(low_probability.value, 0.05 * high_uncertainty.value, 1e-12);
}

TEST(DynamicRiskCostField, AggregatesWorstObstacleAndHandlesEmptyPredictionSet) {
  RiskContext context;
  const auto far = makePrediction(20U, 10.0, 10.0,
                                  Eigen::Vector3d(3.0, 0.0, 0.0),
                                  Eigen::Vector3d(3.0, 0.0, 0.0));
  const auto near = makePrediction(5U, 10.0, 10.0,
                                   Eigen::Vector3d(0.75, 0.0, 0.0),
                                   Eigen::Vector3d(0.75, 0.0, 0.0));
  DynamicRiskCostField field({far, near}, context, testOptions());
  const auto result = field.evaluate(10.0, Eigen::Vector3d::Zero());

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.value, 0.25, 1e-12);
  // The obstacle is on +x. A positive d(cost)/dx makes gradient descent move
  // toward -x, away from the obstacle.
  EXPECT_GT(result.gradient.x(), 0.0);

  DynamicRiskCostField empty_field({}, context, testOptions());
  const auto empty = empty_field.evaluate(10.0, Eigen::Vector3d::Zero());
  EXPECT_TRUE(empty.valid);
  EXPECT_DOUBLE_EQ(empty.value, 0.0);
  EXPECT_TRUE(empty.gradient.isZero(1e-12));
}

TEST(DynamicRiskCostField, RejectsQueriesOutsidePredictionCoverage) {
  RiskContext context;
  DynamicRiskCostField field({makePrediction(9U)}, context, testOptions());

  const auto before = field.evaluate(9.0, Eigen::Vector3d::Zero());
  EXPECT_FALSE(before.valid);
  EXPECT_FALSE(before.detail.empty());

  const auto after = field.evaluate(11.1, Eigen::Vector3d::Zero());
  EXPECT_FALSE(after.valid);
  EXPECT_FALSE(after.detail.empty());
}

TEST(DynamicRiskCostField, RejectsInvalidOptionsAndPredictionStates) {
  RiskContext context;
  auto options = testOptions();
  options.warning_clearance = 0.0;
  EXPECT_THROW(DynamicRiskCostField({}, context, options), std::invalid_argument);

  auto invalid = makePrediction(1U);
  invalid.states.front().covariance(0, 1) = 0.1;
  DynamicRiskCostField field({invalid}, context, testOptions());
  const auto result = field.evaluate(10.0, Eigen::Vector3d::Zero());
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.detail.empty());
}

}  // namespace
