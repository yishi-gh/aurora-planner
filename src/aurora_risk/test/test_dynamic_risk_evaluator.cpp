// Copyright 2026 PathAlgo26
#include "aurora_risk/dynamic_risk_evaluator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

using aurora::prediction::ObstacleShape;
using aurora::prediction::PredictedState;
using aurora::prediction::PredictionResult;
using aurora::prediction::PredictionStatus;
using aurora::prediction::ShapeType;
using aurora::risk::DynamicRiskEvaluation;
using aurora::risk::DynamicRiskEvaluator;
using aurora::risk::DynamicRiskEvaluatorOptions;
using aurora::risk::DynamicRiskInput;
using aurora::risk::MapQualitySample;
using aurora::risk::MapRiskState;
using aurora::risk::RiskStatus;
using aurora::risk::TrajectorySample;

TrajectorySample makeSample(double stamp, const Eigen::Vector3d &position) {
  TrajectorySample sample;
  sample.stamp = stamp;
  sample.position = position;
  sample.position_covariance.setZero();
  return sample;
}

std::vector<TrajectorySample> makeTrajectory(double start = 10.0, double end = 11.0) {
  return {makeSample(start, Eigen::Vector3d::Zero()),
          makeSample(0.5 * (start + end), Eigen::Vector3d::Zero()),
          makeSample(end, Eigen::Vector3d::Zero())};
}

PredictedState makeState(double stamp, const Eigen::Vector3d &position,
                         const ObstacleShape &shape = ObstacleShape{}) {
  PredictedState state;
  state.stamp = stamp;
  state.position = position;
  state.velocity.setZero();
  state.acceleration.setZero();
  state.covariance.setZero();
  state.existence_probability = 1.0;
  state.mode_probability = 1.0;
  state.shape = shape;
  return state;
}

PredictionResult makePrediction(std::uint64_t track_id = 7U,
                                 double reference_stamp = 10.0,
                                 double end_stamp = 11.0,
                                 const Eigen::Vector3d &start_position =
                                     Eigen::Vector3d(0.0, 0.0, 0.0),
                                 const Eigen::Vector3d &end_position =
                                     Eigen::Vector3d(0.0, 0.0, 0.0),
                                 const ObstacleShape &shape = ObstacleShape{}) {
  PredictionResult prediction;
  prediction.status = PredictionStatus::SUCCESS;
  prediction.track_id = track_id;
  prediction.reference_stamp = reference_stamp;
  prediction.states.push_back(makeState(reference_stamp, start_position, shape));
  if (end_stamp > reference_stamp + 1e-12) {
    prediction.states.push_back(makeState(end_stamp, end_position, shape));
  }
  return prediction;
}

DynamicRiskInput makeInput(std::vector<PredictionResult> predictions = {}) {
  DynamicRiskInput input;
  input.has_snapshot = true;
  input.snapshot_stamp = 10.0;
  input.predictions = std::move(predictions);
  return input;
}

DynamicRiskEvaluatorOptions permissiveTestOptions() {
  DynamicRiskEvaluatorOptions options;
  options.vehicle_radius = 0.0;
  options.warning_clearance = 1.0;
  options.max_prediction_age = 2.0;
  return options;
}

void attachMapQuality(DynamicRiskInput *input, const std::vector<TrajectorySample> &trajectory,
                      MapRiskState state, double occupancy_probability,
                      double observation_age, double confidence) {
  input->context.map.available = true;
  input->context.map.snapshot_stamp = trajectory.front().stamp;
  input->context.map.map_version = 23U;
  input->context.map.samples.clear();
  input->context.map.samples.reserve(trajectory.size());
  for (const auto &sample : trajectory) {
    input->context.map.samples.push_back(MapQualitySample{
        sample.stamp, sample.position, state, occupancy_probability, observation_age,
        confidence, false, 23U});
  }
}

TEST(DynamicRiskEvaluator, AcceptsExplicitEmptySnapshotHeartbeat) {
  DynamicRiskEvaluator evaluator;
  const DynamicRiskEvaluation result = evaluator.evaluate(makeTrajectory(), makeInput());

  EXPECT_EQ(result.status, RiskStatus::ACCEPTED);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.level, aurora::risk::RiskLevel::LOW);
  EXPECT_DOUBLE_EQ(result.dynamic_risk, 0.0);
  EXPECT_EQ(result.checked_samples, 3U);
  EXPECT_EQ(result.checked_obstacles, 0U);
}

TEST(DynamicRiskEvaluator, CarriesUnifiedRiskContextWithoutChangingBaselineHeartbeat) {
  DynamicRiskEvaluator evaluator;
  DynamicRiskInput input = makeInput();
  const auto trajectory = makeTrajectory();
  attachMapQuality(&input, trajectory, MapRiskState::FREE, 0.1, 0.0, 0.9);
  input.context.vehicle.has_localization_position_covariance = true;
  input.context.vehicle.localization_position_covariance =
      0.04 * Eigen::Matrix3d::Identity();
  input.context.vehicle.has_execution_position_covariance = true;
  input.context.vehicle.execution_position_covariance =
      0.01 * Eigen::Matrix3d::Identity();
  input.context.delay = {0.01, 0.02, 0.03, 0.04, 0.05};

  EXPECT_DOUBLE_EQ(input.context.delay.total(), 0.15);
  const auto result = evaluator.evaluate(trajectory, input);

  EXPECT_EQ(result.status, RiskStatus::ACCEPTED);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.checked_obstacles, 0U);
}

TEST(DynamicRiskEvaluator, RejectsConservativeSphereEnvelopeIntersection) {
  DynamicRiskEvaluatorOptions options;
  options.vehicle_radius = 0.65;
  DynamicRiskEvaluator evaluator(options);

  ObstacleShape shape;
  shape.type = ShapeType::SPHERE;
  shape.radius = 0.4;
  const auto prediction = makePrediction(42U, 10.0, 11.0,
                                         Eigen::Vector3d(0.8, 0.0, 0.0),
                                         Eigen::Vector3d(0.8, 0.0, 0.0), shape);
  const auto result = evaluator.evaluate(makeTrajectory(), makeInput({prediction}));

  EXPECT_EQ(result.status, RiskStatus::DYNAMIC_COLLISION);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.level, aurora::risk::RiskLevel::HIGH);
  EXPECT_EQ(result.worst_obstacle_id, 42);
  EXPECT_NEAR(result.minimum_clearance, -0.25, 1e-12);
  EXPECT_DOUBLE_EQ(result.dynamic_risk, 1.0);
  EXPECT_DOUBLE_EQ(result.total_risk, 1.0);
}

TEST(DynamicRiskEvaluator, ThreeSigmaEnvelopeExpandsWithCovariance) {
  auto low_covariance_prediction = makePrediction(
      3U, 10.0, 11.0, Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0));
  auto high_covariance_prediction = low_covariance_prediction;
  high_covariance_prediction.states[0].covariance.setIdentity();
  high_covariance_prediction.states[1].covariance.setIdentity();

  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  const auto safe = evaluator.evaluate(makeTrajectory(),
                                        makeInput({low_covariance_prediction}));
  const auto risky = evaluator.evaluate(makeTrajectory(),
                                        makeInput({high_covariance_prediction}));

  EXPECT_EQ(safe.status, RiskStatus::ACCEPTED);
  EXPECT_TRUE(safe.minimum_clearance > 0.0);
  EXPECT_EQ(risky.status, RiskStatus::DYNAMIC_COLLISION);
  EXPECT_LT(risky.minimum_clearance, safe.minimum_clearance);
}

TEST(DynamicRiskEvaluator, AddsVehicleAndTrajectoryPositionCovariances) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  const auto prediction = makePrediction(13U, 10.0, 10.0,
                                         Eigen::Vector3d(2.0, 0.0, 0.0),
                                         Eigen::Vector3d(2.0, 0.0, 0.0));
  const auto trajectory = std::vector<TrajectorySample>{makeSample(
      10.0, Eigen::Vector3d::Zero())};

  const auto baseline = evaluator.evaluate(trajectory, makeInput({prediction}));
  DynamicRiskInput input = makeInput({prediction});
  input.context.vehicle.has_localization_position_covariance = true;
  input.context.vehicle.localization_position_covariance =
      0.04 * Eigen::Matrix3d::Identity();
  input.context.vehicle.has_execution_position_covariance = true;
  input.context.vehicle.execution_position_covariance =
      0.36 * Eigen::Matrix3d::Identity();
  auto uncertain_trajectory = trajectory;
  uncertain_trajectory.front().position_covariance =
      0.04 * Eigen::Matrix3d::Identity();
  const auto uncertain = evaluator.evaluate(uncertain_trajectory, input);

  ASSERT_EQ(baseline.status, RiskStatus::ACCEPTED);
  ASSERT_EQ(uncertain.status, RiskStatus::ACCEPTED);
  EXPECT_NEAR(baseline.minimum_clearance, 2.0, 1e-12);
  EXPECT_NEAR(uncertain.minimum_clearance, 2.0 - 3.0 * std::sqrt(0.44), 1e-12);
  EXPECT_LT(uncertain.minimum_clearance, baseline.minimum_clearance);
}

TEST(DynamicRiskEvaluator, AppliesDelayAsPredictionTimeLeadWithoutMovingTrajectoryTime) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  const auto prediction = makePrediction(14U, 10.0, 11.0,
                                         Eigen::Vector3d(2.0, 0.0, 0.0),
                                         Eigen::Vector3d(-2.0, 0.0, 0.0));
  DynamicRiskInput input = makeInput({prediction});
  input.context.delay.safety_margin = 0.5;
  const auto result = evaluator.evaluate(
      std::vector<TrajectorySample>{makeSample(10.0, Eigen::Vector3d::Zero())}, input);

  EXPECT_EQ(result.status, RiskStatus::DYNAMIC_COLLISION);
  EXPECT_FALSE(result.accepted);
  EXPECT_DOUBLE_EQ(result.worst_time, 10.0);
}

TEST(DynamicRiskEvaluator, ScalesSoftRiskByExistenceProbabilityButKeepsHardGate) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  auto low_probability_prediction = makePrediction(
      15U, 10.0, 10.0, Eigen::Vector3d(0.75, 0.0, 0.0),
      Eigen::Vector3d(0.75, 0.0, 0.0));
  low_probability_prediction.states.front().existence_probability = 0.0;
  const auto soft_result = evaluator.evaluate(
      std::vector<TrajectorySample>{makeSample(10.0, Eigen::Vector3d::Zero())},
      makeInput({low_probability_prediction}));

  ASSERT_EQ(soft_result.status, RiskStatus::ACCEPTED);
  EXPECT_NEAR(soft_result.dynamic_risk, 0.05 * 0.25, 1e-12);

  auto colliding_prediction = low_probability_prediction;
  colliding_prediction.states.front().position = Eigen::Vector3d::Zero();
  const auto hard_result = evaluator.evaluate(
      std::vector<TrajectorySample>{makeSample(10.0, Eigen::Vector3d::Zero())},
      makeInput({colliding_prediction}));
  EXPECT_EQ(hard_result.status, RiskStatus::DYNAMIC_COLLISION);
  EXPECT_DOUBLE_EQ(hard_result.dynamic_risk, 1.0);
}

TEST(DynamicRiskEvaluator, RequiresMapQualityWhenConfigured) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.require_map_quality = true;
  DynamicRiskEvaluator evaluator(options);

  const auto result = evaluator.evaluate(makeTrajectory(), makeInput());

  EXPECT_EQ(result.status, RiskStatus::NO_MAP_INFORMATION);
  EXPECT_FALSE(result.accepted);
}

TEST(DynamicRiskEvaluator, RejectsOccupiedMapSamplesBeforeDynamicEvaluation) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.require_map_quality = true;
  DynamicRiskEvaluator evaluator(options);
  const auto trajectory = makeTrajectory();
  auto input = makeInput();
  attachMapQuality(&input, trajectory, MapRiskState::FREE, 0.1, 0.0, 1.0);
  input.context.map.samples[1].state = MapRiskState::OCCUPIED;

  const auto result = evaluator.evaluate(trajectory, input);

  EXPECT_EQ(result.status, RiskStatus::MAP_COLLISION);
  EXPECT_FALSE(result.accepted);
  EXPECT_DOUBLE_EQ(result.map_risk, 1.0);
}

TEST(DynamicRiskEvaluator, MapQualitySoftRiskIsMonotonicAndAggregated) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.require_map_quality = true;
  DynamicRiskEvaluator evaluator(options);
  const auto trajectory = makeTrajectory();

  auto good_input = makeInput();
  attachMapQuality(&good_input, trajectory, MapRiskState::FREE, 0.1, 0.0, 1.0);
  const auto good = evaluator.evaluate(trajectory, good_input);

  auto degraded_input = makeInput();
  attachMapQuality(&degraded_input, trajectory, MapRiskState::FREE, 0.6, 0.5, 0.2);
  const auto degraded = evaluator.evaluate(trajectory, degraded_input);

  ASSERT_EQ(good.status, RiskStatus::ACCEPTED);
  ASSERT_EQ(degraded.status, RiskStatus::ACCEPTED);
  EXPECT_DOUBLE_EQ(good.map_risk, 0.0);
  EXPECT_NEAR(degraded.map_risk, 0.6583333333333333, 1e-12);
  EXPECT_DOUBLE_EQ(degraded.total_risk, degraded.map_risk);
  EXPECT_GT(degraded.map_risk, good.map_risk);
  EXPECT_EQ(degraded.level, aurora::risk::RiskLevel::MEDIUM);
}

TEST(DynamicRiskEvaluator, UnknownMapCanOnlyEnterExplicitSoftRiskMode) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.require_map_quality = true;
  DynamicRiskEvaluator evaluator(options);
  const auto trajectory = makeTrajectory();
  auto input = makeInput();
  attachMapQuality(&input, trajectory, MapRiskState::UNKNOWN, 0.5,
                  std::numeric_limits<double>::infinity(), 0.0);

  const auto rejected = evaluator.evaluate(trajectory, input);
  EXPECT_EQ(rejected.status, RiskStatus::MAP_UNKNOWN);

  options.allow_unknown_space = true;
  DynamicRiskEvaluator soft_evaluator(options);
  const auto accepted = soft_evaluator.evaluate(trajectory, input);
  EXPECT_EQ(accepted.status, RiskStatus::ACCEPTED);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_DOUBLE_EQ(accepted.map_risk, 1.0);
}

TEST(DynamicRiskEvaluator, AppliesConfiguredMapRiskLimit) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.require_map_quality = true;
  options.map_risk_limit = 0.5;
  DynamicRiskEvaluator evaluator(options);
  const auto trajectory = makeTrajectory();
  auto input = makeInput();
  attachMapQuality(&input, trajectory, MapRiskState::FREE, 0.6, 0.5, 0.2);

  const auto result = evaluator.evaluate(trajectory, input);

  EXPECT_EQ(result.status, RiskStatus::MAP_RISK);
  EXPECT_FALSE(result.accepted);
  EXPECT_NEAR(result.map_risk, 0.6583333333333333, 1e-12);
}

TEST(DynamicRiskEvaluator, ConvertsBoxCapsuleAndMultiSphereToConservativeRadius) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  const auto trajectory = std::vector<TrajectorySample>{makeSample(10.0, Eigen::Vector3d::Zero())};

  ObstacleShape box;
  box.type = ShapeType::BOX;
  box.dimensions = Eigen::Vector3d(2.0, 2.0, 2.0);
  auto box_prediction = makePrediction(1U, 10.0, 10.0, Eigen::Vector3d(1.5, 0.0, 0.0),
                                       Eigen::Vector3d(1.5, 0.0, 0.0), box);
  const auto box_result = evaluator.evaluate(trajectory, makeInput({box_prediction}));
  EXPECT_EQ(box_result.status, RiskStatus::DYNAMIC_COLLISION);

  ObstacleShape capsule;
  capsule.type = ShapeType::CAPSULE;
  capsule.radius = 0.2;
  capsule.dimensions = Eigen::Vector3d(2.0, 0.0, 0.0);
  auto capsule_prediction = makePrediction(2U, 10.0, 10.0, Eigen::Vector3d(1.1, 0.0, 0.0),
                                            Eigen::Vector3d(1.1, 0.0, 0.0), capsule);
  const auto capsule_result = evaluator.evaluate(trajectory, makeInput({capsule_prediction}));
  EXPECT_EQ(capsule_result.status, RiskStatus::DYNAMIC_COLLISION);

  ObstacleShape multi_sphere;
  multi_sphere.type = ShapeType::MULTI_SPHERE;
  multi_sphere.radius = 0.1;
  multi_sphere.dimensions = Eigen::Vector3d(3.0, 0.0, 0.0);
  auto multi_prediction = makePrediction(3U, 10.0, 10.0, Eigen::Vector3d(1.4, 0.0, 0.0),
                                          Eigen::Vector3d(1.4, 0.0, 0.0), multi_sphere);
  const auto multi_result = evaluator.evaluate(trajectory, makeInput({multi_prediction}));
  EXPECT_EQ(multi_result.status, RiskStatus::DYNAMIC_COLLISION);
}

TEST(DynamicRiskEvaluator, InterpolatesPredictionOnAbsoluteTimeline) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  const auto prediction = makePrediction(5U, 10.0, 11.0, Eigen::Vector3d(2.0, 0.0, 0.0),
                                         Eigen::Vector3d(4.0, 0.0, 0.0));
  const auto trajectory = std::vector<TrajectorySample>{
      makeSample(10.0, Eigen::Vector3d::Zero()),
      makeSample(10.5, Eigen::Vector3d::Zero()),
      makeSample(11.0, Eigen::Vector3d::Zero())};

  const auto result = evaluator.evaluate(trajectory, makeInput({prediction}));

  ASSERT_EQ(result.status, RiskStatus::ACCEPTED);
  EXPECT_NEAR(result.minimum_clearance, 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.worst_time, 10.0);
  EXPECT_EQ(result.worst_obstacle_id, 5);
}

TEST(DynamicRiskEvaluator, UsesTheLargerShapeWhenPredictionShapeChanges) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  auto prediction = makePrediction(8U, 10.0, 11.0, Eigen::Vector3d(2.0, 0.0, 0.0),
                                   Eigen::Vector3d(2.0, 0.0, 0.0));
  prediction.states[0].shape.radius = 0.0;
  prediction.states[1].shape.type = ShapeType::BOX;
  prediction.states[1].shape.dimensions = Eigen::Vector3d(4.0, 4.0, 4.0);
  const auto result = evaluator.evaluate(
      std::vector<TrajectorySample>{makeSample(10.75, Eigen::Vector3d::Zero())},
      makeInput({prediction}));

  EXPECT_EQ(result.status, RiskStatus::DYNAMIC_COLLISION);
}

TEST(DynamicRiskEvaluator, RejectsPredictionThatDoesNotCoverTrajectoryEnd) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  const auto prediction = makePrediction(5U, 10.0, 10.5, Eigen::Vector3d(4.0, 0.0, 0.0),
                                         Eigen::Vector3d(4.0, 0.0, 0.0));

  const auto result = evaluator.evaluate(makeTrajectory(), makeInput({prediction}));

  EXPECT_EQ(result.status, RiskStatus::PREDICTION_INVALID);
  EXPECT_FALSE(result.accepted);
}

TEST(DynamicRiskEvaluator, RejectsMissingOrMalformedDynamicInformation) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  DynamicRiskInput missing_snapshot;
  const auto no_snapshot = evaluator.evaluate(makeTrajectory(), missing_snapshot);
  EXPECT_EQ(no_snapshot.status, RiskStatus::NO_DYNAMIC_INFORMATION);

  auto invalid_count = makeInput();
  invalid_count.invalid_track_count = 1U;
  const auto malformed_batch = evaluator.evaluate(makeTrajectory(), invalid_count);
  EXPECT_EQ(malformed_batch.status, RiskStatus::INFORMATION_STALE);

  auto invalid_snapshot_time = makeInput();
  invalid_snapshot_time.snapshot_stamp = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_snapshot = evaluator.evaluate(makeTrajectory(), invalid_snapshot_time);
  EXPECT_EQ(invalid_snapshot.status, RiskStatus::INVALID_INPUT);
}

TEST(DynamicRiskEvaluator, RejectsExpiredEmptySnapshot) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.max_prediction_age = 0.25;
  DynamicRiskEvaluator evaluator(options);
  auto input = makeInput();
  input.snapshot_stamp = 9.0;

  const auto result = evaluator.evaluate(makeTrajectory(), input);

  EXPECT_EQ(result.status, RiskStatus::INFORMATION_STALE);
  EXPECT_FALSE(result.accepted);
}

TEST(DynamicRiskEvaluator, RejectsExplicitOcclusionInsteadOfTreatingItAsEmptyHeartbeat) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  auto input = makeInput();
  input.occlusion_active = true;

  const auto result = evaluator.evaluate(makeTrajectory(), input);

  EXPECT_EQ(result.status, RiskStatus::INFORMATION_STALE);
  EXPECT_FALSE(result.accepted);
}

TEST(DynamicRiskEvaluator, EvaluatesInformationAgeAtAnExplicitTime) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.max_prediction_age = 0.25;
  DynamicRiskEvaluator evaluator(options);

  auto fresh_input = makeInput();
  fresh_input.evaluation_stamp = 10.2;
  const auto fresh = evaluator.evaluate(makeTrajectory(), fresh_input);
  EXPECT_EQ(fresh.status, RiskStatus::ACCEPTED);
  EXPECT_NEAR(fresh.information_age, 0.2, 1e-12);

  auto stale_input = makeInput();
  stale_input.evaluation_stamp = 10.3;
  const auto stale = evaluator.evaluate(makeTrajectory(), stale_input);
  EXPECT_EQ(stale.status, RiskStatus::INFORMATION_STALE);
  EXPECT_FALSE(stale.accepted);
  EXPECT_NEAR(stale.information_age, 0.3, 1e-12);
}

TEST(DynamicRiskEvaluator, CanExplicitlyDisableRequiredDynamicInformation) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.require_dynamic_information = false;
  DynamicRiskEvaluator evaluator(options);
  DynamicRiskInput input;

  const auto result = evaluator.evaluate(makeTrajectory(), input);

  EXPECT_EQ(result.status, RiskStatus::ACCEPTED);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.level, aurora::risk::RiskLevel::UNKNOWN);
}

TEST(DynamicRiskEvaluator, RejectsStalePredictionReference) {
  DynamicRiskEvaluatorOptions options = permissiveTestOptions();
  options.max_prediction_age = 0.25;
  DynamicRiskEvaluator evaluator(options);
  const auto prediction = makePrediction(5U, 9.0, 11.0, Eigen::Vector3d(4.0, 0.0, 0.0),
                                         Eigen::Vector3d(4.0, 0.0, 0.0));

  const auto result = evaluator.evaluate(makeTrajectory(), makeInput({prediction}));

  EXPECT_EQ(result.status, RiskStatus::INFORMATION_STALE);
  EXPECT_FALSE(result.accepted);
}

TEST(DynamicRiskEvaluator, RejectsInvalidPredictionCovarianceAndTrajectorySample) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  auto invalid_covariance_prediction = makePrediction(5U, 10.0, 11.0,
                                                       Eigen::Vector3d(4.0, 0.0, 0.0),
                                                       Eigen::Vector3d(4.0, 0.0, 0.0));
  invalid_covariance_prediction.states[0].covariance(0, 1) = 0.2;
  const auto invalid_covariance = evaluator.evaluate(
      makeTrajectory(), makeInput({invalid_covariance_prediction}));
  EXPECT_EQ(invalid_covariance.status, RiskStatus::PREDICTION_INVALID);

  auto invalid_trajectory = makeTrajectory();
  invalid_trajectory[1].stamp = invalid_trajectory[0].stamp;
  const auto invalid_sample = evaluator.evaluate(invalid_trajectory, makeInput());
  EXPECT_EQ(invalid_sample.status, RiskStatus::INVALID_INPUT);
}

TEST(DynamicRiskEvaluator, RejectsInvalidPredictionOrdering) {
  DynamicRiskEvaluator evaluator(permissiveTestOptions());
  auto prediction = makePrediction(5U, 10.0, 11.0, Eigen::Vector3d(4.0, 0.0, 0.0),
                                   Eigen::Vector3d(4.0, 0.0, 0.0));
  prediction.states[1].stamp = prediction.states[0].stamp;
  const auto result = evaluator.evaluate(makeTrajectory(), makeInput({prediction}));
  EXPECT_EQ(result.status, RiskStatus::PREDICTION_INVALID);
}

TEST(DynamicRiskEvaluator, RejectsInvalidOptions) {
  DynamicRiskEvaluatorOptions options;
  options.vehicle_radius = -0.1;
  EXPECT_THROW(DynamicRiskEvaluator evaluator(options), std::invalid_argument);

  options = {};
  options.warning_clearance = 0.0;
  EXPECT_THROW(DynamicRiskEvaluator evaluator(options), std::invalid_argument);
}

}  // namespace
