#include "aurora_prediction/kinematic_predictor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aurora::prediction::Covariance6;
using aurora::prediction::KinematicPredictor;
using aurora::prediction::KinematicPredictorOptions;
using aurora::prediction::PredictionModel;
using aurora::prediction::PredictionStatus;
using aurora::prediction::TrackState;

TrackState makeTrack() {
  TrackState track;
  track.track_id = 7U;
  track.stamp = 10.0;
  track.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  track.velocity = Eigen::Vector3d(2.0, 1.0, -0.5);
  track.acceleration = Eigen::Vector3d(0.5, -1.0, 2.0);
  track.existence_probability = 0.8;
  track.shape.radius = 0.4;
  return track;
}

TEST(KinematicPredictor, PropagatesConstantVelocityAndCovariance) {
  KinematicPredictorOptions options;
  options.sample_interval = 0.1;
  options.max_horizon = 1.0;
  options.process_noise_acceleration = 0.5;
  KinematicPredictor predictor(options);

  TrackState track = makeTrack();
  track.model = PredictionModel::CV;
  track.has_covariance = true;
  track.covariance = Covariance6::Identity();

  const auto result = predictor.predict(track, 0.2);
  ASSERT_EQ(result.status, PredictionStatus::SUCCESS);
  ASSERT_EQ(result.states.size(), 3U);
  EXPECT_FALSE(result.covariance_defaulted);
  EXPECT_FALSE(result.covariance_regularized);
  EXPECT_FALSE(result.acceleration_covariance_defaulted);

  const auto &initial = result.states.front();
  const auto &final = result.states.back();
  EXPECT_DOUBLE_EQ(initial.stamp, 10.0);
  EXPECT_NEAR(final.stamp, 10.2, 1e-12);
  EXPECT_NEAR(final.position.x(), 1.4, 1e-12);
  EXPECT_NEAR(final.position.y(), -1.8, 1e-12);
  EXPECT_NEAR(final.position.z(), 0.4, 1e-12);
  EXPECT_TRUE(final.velocity.isApprox(track.velocity, 1e-12));
  EXPECT_TRUE(final.acceleration.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_GT(final.covariance(0, 0), initial.covariance(0, 0));
  EXPECT_GT(final.covariance(3, 3), initial.covariance(3, 3));
  EXPECT_DOUBLE_EQ(final.existence_probability, track.existence_probability);
  EXPECT_DOUBLE_EQ(final.shape.radius, track.shape.radius);
}

TEST(KinematicPredictor, PropagatesConstantAccelerationAndUsesExactHorizon) {
  KinematicPredictorOptions options;
  options.sample_interval = 0.3;
  options.max_horizon = 1.0;
  options.process_noise_jerk = 0.0;
  options.default_acceleration_variance = 0.0;
  KinematicPredictor predictor(options);

  TrackState track = makeTrack();
  track.model = PredictionModel::CA;
  track.has_covariance = true;
  track.covariance = Covariance6::Zero();

  const auto result = predictor.predict(track, 0.8);
  ASSERT_EQ(result.status, PredictionStatus::SUCCESS);
  ASSERT_EQ(result.states.size(), 4U);
  EXPECT_TRUE(result.acceleration_covariance_defaulted);
  EXPECT_NEAR(result.states.back().stamp, 10.8, 1e-12);
  EXPECT_NEAR(result.states.back().position.x(), 2.76, 1e-12);
  EXPECT_NEAR(result.states.back().position.y(), -1.52, 1e-12);
  EXPECT_NEAR(result.states.back().position.z(), 0.74, 1e-12);
  EXPECT_NEAR(result.states.back().velocity.x(), 2.4, 1e-12);
  EXPECT_NEAR(result.states.back().velocity.y(), 0.2, 1e-12);
  EXPECT_NEAR(result.states.back().velocity.z(), 1.1, 1e-12);
  EXPECT_TRUE(result.states.back().acceleration.isApprox(track.acceleration, 1e-12));
}

TEST(KinematicPredictor, DefaultsAndRegularizesSmallCovarianceArtifacts) {
  KinematicPredictorOptions options;
  options.sample_interval = 0.2;
  options.max_horizon = 1.0;
  options.process_noise_acceleration = 0.0;
  KinematicPredictor predictor(options);

  TrackState default_track = makeTrack();
  const auto default_result = predictor.predict(default_track, 0.0);
  ASSERT_EQ(default_result.status, PredictionStatus::SUCCESS);
  ASSERT_EQ(default_result.states.size(), 1U);
  EXPECT_TRUE(default_result.covariance_defaulted);
  EXPECT_NEAR(default_result.states.front().covariance(0, 0),
              options.default_position_variance, 1e-12);
  EXPECT_NEAR(default_result.states.front().covariance(3, 3),
              options.default_velocity_variance, 1e-12);

  TrackState regularized_track = makeTrack();
  regularized_track.has_covariance = true;
  regularized_track.covariance = Covariance6::Identity();
  regularized_track.covariance(0, 0) = -0.5 * options.covariance_tolerance;
  const auto regularized_result = predictor.predict(regularized_track, 0.0);
  ASSERT_EQ(regularized_result.status, PredictionStatus::SUCCESS);
  EXPECT_TRUE(regularized_result.covariance_regularized);
  EXPECT_GE(regularized_result.states.front().covariance.minCoeff(), -1e-12);
}

TEST(KinematicPredictor, RejectsInvalidTrackCovarianceAndHorizon) {
  KinematicPredictor predictor;
  TrackState track = makeTrack();

  auto result = predictor.predict(track, -0.1);
  EXPECT_EQ(result.status, PredictionStatus::INVALID_INPUT);

  result = predictor.predict(track, 6.0);
  EXPECT_EQ(result.status, PredictionStatus::HORIZON_EXCEEDED);

  track.has_covariance = true;
  track.covariance = Covariance6::Identity();
  track.covariance(0, 1) = 0.1;
  result = predictor.predict(track, 0.1);
  EXPECT_EQ(result.status, PredictionStatus::INVALID_INPUT);

  track.covariance = Covariance6::Identity();
  track.covariance(0, 0) = -1.0;
  result = predictor.predict(track, 0.1);
  EXPECT_EQ(result.status, PredictionStatus::INVALID_INPUT);

  track.covariance = Covariance6::Identity();
  track.position.x() = std::numeric_limits<double>::quiet_NaN();
  result = predictor.predict(track, 0.1);
  EXPECT_EQ(result.status, PredictionStatus::INVALID_INPUT);
}

TEST(KinematicPredictor, ReportsSampleLimitWithoutPartialPrediction) {
  KinematicPredictorOptions options;
  options.sample_interval = 0.1;
  options.max_horizon = 1.0;
  options.max_samples = 2U;
  KinematicPredictor predictor(options);

  const auto result = predictor.predict(makeTrack(), 0.2);
  EXPECT_EQ(result.status, PredictionStatus::SAMPLE_LIMIT);
  EXPECT_TRUE(result.states.empty());
}

}  // namespace
