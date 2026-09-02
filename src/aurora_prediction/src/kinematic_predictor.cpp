#include "aurora_prediction/kinematic_predictor.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::prediction {
namespace {

constexpr double kEpsilon = 1e-12;

bool isFinite(double value) { return std::isfinite(value); }

struct CovariancePreparation {
  bool valid{false};
  Covariance6 covariance{Covariance6::Zero()};
  bool regularized{false};
  std::string detail;
};

CovariancePreparation prepareCovariance(const TrackState &track,
                                        const KinematicPredictorOptions &options) {
  CovariancePreparation result;
  if (!track.has_covariance) {
    result.covariance.setZero();
    result.covariance.diagonal().segment<3>(0).setConstant(options.default_position_variance);
    result.covariance.diagonal().segment<3>(3).setConstant(options.default_velocity_variance);
    result.valid = true;
    result.detail = "state covariance was not supplied; conservative defaults were used";
    return result;
  }

  if (!track.covariance.allFinite()) {
    result.detail = "state covariance contains non-finite values";
    return result;
  }
  const Covariance6 symmetric = 0.5 * (track.covariance + track.covariance.transpose());
  if ((track.covariance - track.covariance.transpose()).cwiseAbs().maxCoeff() >
      options.covariance_tolerance) {
    result.detail = "state covariance is not symmetric";
    return result;
  }

  Eigen::SelfAdjointEigenSolver<Covariance6> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    result.detail = "state covariance eigendecomposition failed";
    return result;
  }
  const auto eigenvalues = solver.eigenvalues();
  if (eigenvalues.minCoeff() < -options.covariance_tolerance) {
    result.detail = "state covariance is not positive semidefinite";
    return result;
  }
  if (eigenvalues.minCoeff() < 0.0) {
    result.covariance = solver.eigenvectors() * eigenvalues.cwiseMax(0.0).asDiagonal() *
                        solver.eigenvectors().transpose();
    result.regularized = true;
  } else {
    result.covariance = symmetric;
  }
  result.valid = true;
  result.detail = "state covariance accepted";
  return result;
}

Eigen::Matrix<double, 6, 6> cvTransition(double dt) {
  Eigen::Matrix<double, 6, 6> transition = Eigen::Matrix<double, 6, 6>::Identity();
  transition.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  return transition;
}

Eigen::Matrix<double, 6, 6> cvProcessNoise(double dt, double spectral_density) {
  Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  noise.block<3, 3>(0, 0) = spectral_density * (dt * dt * dt / 3.0) * identity;
  noise.block<3, 3>(0, 3) = spectral_density * (dt * dt / 2.0) * identity;
  noise.block<3, 3>(3, 0) = noise.block<3, 3>(0, 3);
  noise.block<3, 3>(3, 3) = spectral_density * dt * identity;
  return noise;
}

Eigen::Matrix<double, 9, 9> caTransition(double dt) {
  Eigen::Matrix<double, 9, 9> transition = Eigen::Matrix<double, 9, 9>::Identity();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  transition.block<3, 3>(0, 3) = dt * identity;
  transition.block<3, 3>(0, 6) = 0.5 * dt * dt * identity;
  transition.block<3, 3>(3, 6) = dt * identity;
  return transition;
}

Eigen::Matrix<double, 9, 9> caProcessNoise(double dt, double spectral_density) {
  Eigen::Matrix<double, 9, 9> noise = Eigen::Matrix<double, 9, 9>::Zero();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;
  noise.block<3, 3>(0, 0) = spectral_density * (dt5 / 20.0) * identity;
  noise.block<3, 3>(0, 3) = spectral_density * (dt4 / 8.0) * identity;
  noise.block<3, 3>(0, 6) = spectral_density * (dt3 / 6.0) * identity;
  noise.block<3, 3>(3, 0) = noise.block<3, 3>(0, 3);
  noise.block<3, 3>(3, 3) = spectral_density * (dt3 / 3.0) * identity;
  noise.block<3, 3>(3, 6) = spectral_density * (dt2 / 2.0) * identity;
  noise.block<3, 3>(6, 0) = noise.block<3, 3>(0, 6);
  noise.block<3, 3>(6, 3) = noise.block<3, 3>(3, 6);
  noise.block<3, 3>(6, 6) = spectral_density * dt * identity;
  return noise;
}

Covariance6 marginalCovariance(const Eigen::Matrix<double, 9, 9> &covariance) {
  Covariance6 result = Covariance6::Zero();
  result.block<3, 3>(0, 0) = covariance.block<3, 3>(0, 0);
  result.block<3, 3>(0, 3) = covariance.block<3, 3>(0, 3);
  result.block<3, 3>(3, 0) = covariance.block<3, 3>(3, 0);
  result.block<3, 3>(3, 3) = covariance.block<3, 3>(3, 3);
  return 0.5 * (result + result.transpose());
}

PredictedState makeInitialState(const TrackState &track, const Covariance6 &covariance) {
  PredictedState state;
  state.stamp = track.stamp;
  state.position = track.position;
  state.velocity = track.velocity;
  state.acceleration = track.acceleration;
  state.covariance = covariance;
  state.existence_probability = track.existence_probability;
  state.shape = track.shape;
  return state;
}

bool validModel(PredictionModel model) {
  return model == PredictionModel::CV || model == PredictionModel::CA;
}

}  // namespace

const char *toString(PredictionModel model) noexcept {
  switch (model) {
    case PredictionModel::CV:
      return "CV";
    case PredictionModel::CA:
      return "CA";
  }
  return "UNKNOWN_MODEL";
}

const char *toString(PredictionStatus status) noexcept {
  switch (status) {
    case PredictionStatus::SUCCESS:
      return "SUCCESS";
    case PredictionStatus::INVALID_OPTIONS:
      return "INVALID_OPTIONS";
    case PredictionStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case PredictionStatus::HORIZON_EXCEEDED:
      return "HORIZON_EXCEEDED";
    case PredictionStatus::SAMPLE_LIMIT:
      return "SAMPLE_LIMIT";
  }
  return "UNKNOWN_STATUS";
}

void KinematicPredictor::validateOptions(const KinematicPredictorOptions &options) {
  if (!isFinite(options.sample_interval) || options.sample_interval <= 0.0 ||
      !isFinite(options.max_horizon) || options.max_horizon <= 0.0 || options.max_samples == 0U ||
      !isFinite(options.process_noise_acceleration) || options.process_noise_acceleration < 0.0 ||
      !isFinite(options.process_noise_jerk) || options.process_noise_jerk < 0.0 ||
      !isFinite(options.default_position_variance) || options.default_position_variance < 0.0 ||
      !isFinite(options.default_velocity_variance) || options.default_velocity_variance < 0.0 ||
      !isFinite(options.default_acceleration_variance) ||
      options.default_acceleration_variance < 0.0 ||
      !isFinite(options.covariance_tolerance) || options.covariance_tolerance < 0.0) {
    throw std::invalid_argument("invalid kinematic predictor options");
  }
}

KinematicPredictor::KinematicPredictor(KinematicPredictorOptions options)
    : options_(std::move(options)) {
  validateOptions(options_);
}

PredictionResult KinematicPredictor::predict(const TrackState &track, double horizon) const {
  PredictionResult result;
  result.track_id = track.track_id;
  result.reference_stamp = track.stamp;
  result.model = track.model;

  if (!isFinite(horizon) || horizon < 0.0) {
    result.status = PredictionStatus::INVALID_INPUT;
    result.detail = "prediction horizon must be finite and non-negative";
    return result;
  }
  if (horizon > options_.max_horizon + kEpsilon) {
    result.status = PredictionStatus::HORIZON_EXCEEDED;
    result.detail = "prediction horizon exceeds configured maximum";
    return result;
  }
  if (!isFinite(track.stamp) || track.stamp < 0.0 || !track.position.allFinite() ||
      !track.velocity.allFinite() || !track.acceleration.allFinite() ||
      !isFinite(track.existence_probability) || track.existence_probability < 0.0 ||
      track.existence_probability > 1.0 || !validModel(track.model)) {
    result.status = PredictionStatus::INVALID_INPUT;
    result.detail = "dynamic track contains invalid state, probability, time or model";
    return result;
  }

  const CovariancePreparation covariance = prepareCovariance(track, options_);
  if (!covariance.valid) {
    result.status = PredictionStatus::INVALID_INPUT;
    result.detail = covariance.detail;
    return result;
  }
  result.covariance_defaulted = !track.has_covariance;
  result.covariance_regularized = covariance.regularized;
  result.acceleration_covariance_defaulted = track.model == PredictionModel::CA;

  const long double estimated_steps = std::ceil(
      static_cast<long double>(horizon) / static_cast<long double>(options_.sample_interval));
  if (estimated_steps > static_cast<long double>(options_.max_samples - 1U)) {
    result.status = PredictionStatus::SAMPLE_LIMIT;
    result.detail = "prediction sample count exceeds configured maximum";
    return result;
  }
  result.states.reserve(1U + static_cast<std::size_t>(estimated_steps));
  result.states.push_back(makeInitialState(track, covariance.covariance));
  if (horizon <= kEpsilon) {
    result.status = PredictionStatus::SUCCESS;
    result.detail = "zero-horizon prediction contains the current state";
    return result;
  }

  if (track.model == PredictionModel::CV) {
    Eigen::Matrix<double, 6, 1> state;
    state << track.position, track.velocity;
    Covariance6 state_covariance = covariance.covariance;
    double elapsed = 0.0;
    while (elapsed < horizon - kEpsilon) {
      if (result.states.size() >= options_.max_samples) {
        result.status = PredictionStatus::SAMPLE_LIMIT;
        result.detail = "prediction sample count exceeds configured maximum";
        result.states.clear();
        return result;
      }
      const double dt = std::min(options_.sample_interval, horizon - elapsed);
      const auto transition = cvTransition(dt);
      state = transition * state;
      state_covariance = transition * state_covariance * transition.transpose() +
                         cvProcessNoise(dt, options_.process_noise_acceleration);
      state_covariance = 0.5 * (state_covariance + state_covariance.transpose());

      PredictedState predicted;
      elapsed += dt;
      predicted.stamp = track.stamp + elapsed;
      predicted.position = state.head<3>();
      predicted.velocity = state.tail<3>();
      predicted.acceleration = Eigen::Vector3d::Zero();
      predicted.covariance = state_covariance;
      predicted.existence_probability = track.existence_probability;
      predicted.shape = track.shape;
      result.states.push_back(std::move(predicted));
    }
  } else {
    Eigen::Matrix<double, 9, 1> state;
    state << track.position, track.velocity, track.acceleration;
    Eigen::Matrix<double, 9, 9> state_covariance =
        Eigen::Matrix<double, 9, 9>::Zero();
    state_covariance.block<6, 6>(0, 0) = covariance.covariance;
    state_covariance.block<3, 3>(6, 6) =
        options_.default_acceleration_variance * Eigen::Matrix3d::Identity();
    double elapsed = 0.0;
    while (elapsed < horizon - kEpsilon) {
      if (result.states.size() >= options_.max_samples) {
        result.status = PredictionStatus::SAMPLE_LIMIT;
        result.detail = "prediction sample count exceeds configured maximum";
        result.states.clear();
        return result;
      }
      const double dt = std::min(options_.sample_interval, horizon - elapsed);
      const auto transition = caTransition(dt);
      state = transition * state;
      state_covariance = transition * state_covariance * transition.transpose() +
                         caProcessNoise(dt, options_.process_noise_jerk);
      state_covariance = 0.5 * (state_covariance + state_covariance.transpose());

      PredictedState predicted;
      elapsed += dt;
      predicted.stamp = track.stamp + elapsed;
      predicted.position = state.segment<3>(0);
      predicted.velocity = state.segment<3>(3);
      predicted.acceleration = state.segment<3>(6);
      predicted.covariance = marginalCovariance(state_covariance);
      predicted.existence_probability = track.existence_probability;
      predicted.shape = track.shape;
      result.states.push_back(std::move(predicted));
    }
  }

  result.status = PredictionStatus::SUCCESS;
  result.detail = covariance.detail;
  return result;
}

}  // namespace aurora::prediction
