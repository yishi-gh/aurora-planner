#include "aurora_tracking/obstacle_tracker.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace aurora::tracking {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kForbiddenCost = 1e9;
constexpr double kUnmatchedCost = 1.000001;

bool finite(double value) { return std::isfinite(value); }

Eigen::Matrix<double, 6, 6> makeInitialCovariance(const Covariance3 &position_covariance,
                                                   const Covariance3 &velocity_covariance) {
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  covariance.block<3, 3>(0, 0) = position_covariance;
  covariance.block<3, 3>(3, 3) = velocity_covariance;
  return covariance;
}

bool validPrediction(const prediction::PredictionResult &result) {
  return result.status == prediction::PredictionStatus::SUCCESS &&
         !result.states.empty();
}

}  // namespace

const char *toString(LifecycleState state) noexcept {
  switch (state) {
    case LifecycleState::TENTATIVE:
      return "TENTATIVE";
    case LifecycleState::CONFIRMED:
      return "CONFIRMED";
    case LifecycleState::OCCLUDED:
      return "OCCLUDED";
    case LifecycleState::LOST:
      return "LOST";
  }
  return "UNKNOWN_LIFECYCLE";
}

const char *toString(TrackingStatus status) noexcept {
  switch (status) {
    case TrackingStatus::SUCCESS:
      return "SUCCESS";
    case TrackingStatus::PARTIAL_INPUT:
      return "PARTIAL_INPUT";
    case TrackingStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case TrackingStatus::STALE_INPUT:
      return "STALE_INPUT";
  }
  return "UNKNOWN_STATUS";
}

void ObstacleTracker::validateOptions(const ObstacleTrackerOptions &options) {
  if (!finite(options.mahalanobis_gate) || options.mahalanobis_gate <= 0.0 ||
      !finite(options.euclidean_gate) || options.euclidean_gate <= 0.0 ||
      !finite(options.covariance_tolerance) || options.covariance_tolerance < 0.0 ||
      !finite(options.minimum_measurement_variance) ||
      options.minimum_measurement_variance <= 0.0 ||
      !finite(options.default_position_variance) || options.default_position_variance <= 0.0 ||
      !finite(options.default_velocity_variance) || options.default_velocity_variance <= 0.0 ||
      !finite(options.process_noise_acceleration) ||
      options.process_noise_acceleration < 0.0 || options.confirmation_match_count < 2U ||
      !finite(options.lost_after) || options.lost_after <= 0.0 ||
      !finite(options.deleted_after) || options.deleted_after <= options.lost_after ||
      options.first_track_id == 0U) {
    throw std::invalid_argument("invalid obstacle tracker options");
  }
  if (options.default_shape.type != prediction::ShapeType::SPHERE ||
      !finite(options.default_shape.radius) || options.default_shape.radius < 0.0 ||
      !options.default_shape.dimensions.allFinite()) {
    throw std::invalid_argument("default tracker shape must be a finite sphere");
  }
}

ObstacleTracker::ObstacleTracker(ObstacleTrackerOptions options)
    : options_(std::move(options)),
      predictor_([this]() {
        prediction::KinematicPredictorOptions prediction_options;
        prediction_options.sample_interval = 0.05;
        prediction_options.max_horizon = std::max(1.0, options_.deleted_after);
        prediction_options.max_samples = 1000U;
        prediction_options.process_noise_acceleration = options_.process_noise_acceleration;
        prediction_options.default_position_variance = options_.default_position_variance;
        prediction_options.default_velocity_variance = options_.default_velocity_variance;
        prediction_options.covariance_tolerance = options_.covariance_tolerance;
        return prediction_options;
      }()),
      next_track_id_(options_.first_track_id) {
  validateOptions(options_);
}

bool ObstacleTracker::prepareCovariance(const Covariance3 &input,
                                        Covariance3 *output) const {
  if (!input.allFinite() ||
      (input - input.transpose()).cwiseAbs().maxCoeff() > options_.covariance_tolerance) {
    return false;
  }
  const Covariance3 symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Covariance3> solver(symmetric);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().minCoeff() < -options_.covariance_tolerance) {
    return false;
  }
  const Eigen::Vector3d eigenvalues =
      solver.eigenvalues().cwiseMax(options_.minimum_measurement_variance);
  *output = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
  *output = 0.5 * (*output + output->transpose());
  return output->allFinite();
}

bool ObstacleTracker::validateShape(const prediction::ObstacleShape &shape) const {
  if (!shape.dimensions.allFinite() || !finite(shape.radius)) {
    return false;
  }
  switch (shape.type) {
    case prediction::ShapeType::SPHERE:
      return shape.radius >= 0.0;
    case prediction::ShapeType::BOX:
      return (shape.dimensions.array() > 0.0).all();
    case prediction::ShapeType::CAPSULE:
      return shape.radius >= 0.0 && (shape.dimensions.array() >= 0.0).all() &&
             shape.dimensions.norm() > kEpsilon;
    case prediction::ShapeType::MULTI_SPHERE:
      return shape.radius >= 0.0 && (shape.dimensions.array() >= 0.0).all() &&
             (shape.dimensions.norm() > kEpsilon || shape.radius > kEpsilon);
  }
  return false;
}

bool ObstacleTracker::validateDetection(const Detection &detection) const {
  if (!finite(detection.stamp) || detection.stamp < 0.0 || !detection.position.allFinite()) {
    return false;
  }
  if (detection.has_position_covariance) {
    Covariance3 prepared;
    if (!prepareCovariance(detection.position_covariance, &prepared)) {
      return false;
    }
  }
  if (detection.has_velocity && !detection.velocity.allFinite()) {
    return false;
  }
  if (detection.has_velocity_covariance) {
    Covariance3 prepared;
    if (!prepareCovariance(detection.velocity_covariance, &prepared)) {
      return false;
    }
  }
  if (detection.has_velocity_covariance && !detection.has_velocity) {
    return false;
  }
  return !detection.has_shape || validateShape(detection.shape);
}

Covariance3 ObstacleTracker::detectionPositionCovariance(const Detection &detection) const {
  if (!detection.has_position_covariance) {
    return options_.default_position_variance * Covariance3::Identity();
  }
  Covariance3 result;
  if (!prepareCovariance(detection.position_covariance, &result)) {
    return options_.default_position_variance * Covariance3::Identity();
  }
  return result;
}

Covariance3 ObstacleTracker::detectionVelocityCovariance(const Detection &detection) const {
  if (!detection.has_velocity_covariance) {
    return options_.default_velocity_variance * Covariance3::Identity();
  }
  Covariance3 result;
  if (!prepareCovariance(detection.velocity_covariance, &result)) {
    return options_.default_velocity_variance * Covariance3::Identity();
  }
  return result;
}

prediction::ObstacleShape ObstacleTracker::detectionShape(const Detection &detection) const {
  return detection.has_shape ? detection.shape : options_.default_shape;
}

bool ObstacleTracker::predictRecordTo(TrackRecord *record, double stamp) const {
  if (!record || !finite(stamp) || stamp < record->estimate.state.stamp) {
    return false;
  }
  const double horizon = stamp - record->estimate.state.stamp;
  if (horizon <= kEpsilon) {
    record->estimate.missed_duration =
        std::max(0.0, stamp - record->estimate.last_detection_stamp);
    return true;
  }
  const prediction::PredictionResult prediction_result =
      predictor_.predict(record->estimate.state, horizon);
  if (!validPrediction(prediction_result)) {
    return false;
  }
  const prediction::PredictedState &predicted = prediction_result.states.back();
  record->estimate.state.stamp = predicted.stamp;
  record->estimate.state.position = predicted.position;
  record->estimate.state.velocity = predicted.velocity;
  record->estimate.state.acceleration = predicted.acceleration;
  record->estimate.state.covariance = predicted.covariance;
  record->estimate.state.has_covariance = true;
  record->estimate.missed_duration =
      std::max(0.0, stamp - record->estimate.last_detection_stamp);
  return true;
}

ObstacleTracker::CandidateMetric ObstacleTracker::associationMetric(
    const TrackRecord &record, const Detection &detection) const {
  CandidateMetric result;
  const Eigen::Vector3d residual = detection.position - record.estimate.state.position;
  if (detection.has_position_covariance) {
    const Covariance3 innovation =
        record.estimate.state.covariance.block<3, 3>(0, 0) +
        detectionPositionCovariance(detection);
    Eigen::LDLT<Covariance3> solver(0.5 * (innovation + innovation.transpose()));
    if (solver.info() != Eigen::Success) {
      return result;
    }
    const Eigen::Vector3d solved = solver.solve(residual);
    if (!solved.allFinite()) {
      return result;
    }
    const double squared_distance = residual.dot(solved);
    if (!finite(squared_distance) || squared_distance < -options_.covariance_tolerance) {
      return result;
    }
    result.distance = std::sqrt(std::max(0.0, squared_distance));
    result.normalized_cost = squared_distance / (options_.mahalanobis_gate *
                                                  options_.mahalanobis_gate);
    result.mahalanobis = true;
    result.valid = result.distance <= options_.mahalanobis_gate + kEpsilon;
    return result;
  }

  result.distance = residual.norm();
  result.normalized_cost = (result.distance * result.distance) /
                           (options_.euclidean_gate * options_.euclidean_gate);
  result.valid = finite(result.distance) &&
                 result.distance <= options_.euclidean_gate + kEpsilon;
  return result;
}

std::vector<int> ObstacleTracker::solveAssignment(
    const std::vector<std::vector<CandidateMetric>> &metrics) const {
  const std::size_t track_count = metrics.size();
  std::size_t detection_count = 0U;
  if (!metrics.empty()) {
    detection_count = metrics.front().size();
  }
  const std::size_t size = std::max(track_count, detection_count);
  if (size == 0U) {
    return {};
  }

  std::vector<std::vector<double>> cost(size, std::vector<double>(size, 0.0));
  for (std::size_t row = 0; row < size; ++row) {
    for (std::size_t column = 0; column < size; ++column) {
      double value = 0.0;
      if (row < track_count && column < detection_count) {
        value = metrics[row][column].valid ? metrics[row][column].normalized_cost
                                            : kForbiddenCost;
      } else if (row < track_count && column >= detection_count) {
        value = kUnmatchedCost;
      }
      if (row < track_count && column < detection_count && metrics[row][column].valid) {
        value += 1e-10 * static_cast<double>(row) +
                 1e-12 * static_cast<double>(column);
      }
      cost[row][column] = value;
    }
  }

  // Kuhn-Munkres for a square minimization problem. Dummy rows and columns
  // represent unmatched tracks and detections.
  std::vector<double> u(size + 1U, 0.0);
  std::vector<double> v(size + 1U, 0.0);
  std::vector<std::size_t> p(size + 1U, 0U);
  std::vector<std::size_t> way(size + 1U, 0U);
  for (std::size_t row = 1U; row <= size; ++row) {
    p[0] = row;
    std::size_t column0 = 0U;
    std::vector<double> minimum(size + 1U, kForbiddenCost);
    std::vector<bool> used(size + 1U, false);
    do {
      used[column0] = true;
      const std::size_t row0 = p[column0];
      double delta = kForbiddenCost;
      std::size_t column1 = 0U;
      for (std::size_t column = 1U; column <= size; ++column) {
        if (used[column]) {
          continue;
        }
        const double current = cost[row0 - 1U][column - 1U] - u[row0] - v[column];
        const double previous_minimum = minimum.at(column);
        const std::size_t previous_way = way.at(column);
        if (current < previous_minimum - kEpsilon ||
            (std::abs(current - previous_minimum) <= kEpsilon && column < previous_way)) {
          minimum.at(column) = current;
          way.at(column) = column0;
        }
        const double candidate_minimum = minimum.at(column);
        const bool improves_delta = candidate_minimum < delta - kEpsilon;
        const bool breaks_delta_tie =
            std::abs(candidate_minimum - delta) <= kEpsilon && column < column1;
        if (improves_delta || breaks_delta_tie) {
          delta = candidate_minimum;
          column1 = column;
        }
      }
      for (std::size_t column = 0U; column <= size; ++column) {
        if (used[column]) {
          u[p[column]] += delta;
          v[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      column0 = column1;
    } while (p[column0] != 0U);
    do {
      const std::size_t previous = way[column0];
      p[column0] = p[previous];
      column0 = previous;
    } while (column0 != 0U);
  }

  std::vector<int> assignment(track_count, -1);
  for (std::size_t column = 1U; column <= size; ++column) {
    if (p[column] != 0U && p[column] - 1U < track_count && column - 1U < detection_count &&
        metrics[p[column] - 1U][column - 1U].valid) {
      assignment[p[column] - 1U] = static_cast<int>(column - 1U);
    }
  }
  return assignment;
}

void ObstacleTracker::updateMatchedRecord(TrackRecord *record, const Detection &detection,
                                          double stamp) const {
  auto measurementUpdate = [&](const Eigen::Vector3d &measurement,
                               const Covariance3 &measurement_covariance, int state_offset) {
    Eigen::Matrix<double, 6, 3> cross_covariance =
        record->estimate.state.covariance.block<6, 3>(0, state_offset);
    const Covariance3 state_covariance =
        record->estimate.state.covariance.block<3, 3>(state_offset, state_offset);
    const Covariance3 innovation =
        state_covariance + measurement_covariance;
    Eigen::LDLT<Covariance3> solver(0.5 * (innovation + innovation.transpose()));
    if (solver.info() != Eigen::Success) {
      return;
    }
    const Eigen::Matrix<double, 6, 3> gain =
        cross_covariance * solver.solve(Covariance3::Identity());
    Eigen::Matrix<double, 6, 1> state_vector;
    state_vector << record->estimate.state.position, record->estimate.state.velocity;
    const Eigen::Vector3d predicted = state_vector.segment<3>(state_offset);
    state_vector += gain * (measurement - predicted);
    const Eigen::Matrix<double, 6, 6> identity =
        Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 3, 6> observation = Eigen::Matrix<double, 3, 6>::Zero();
    observation.block<3, 3>(0, state_offset) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix<double, 6, 6> joseph_left = identity - gain * observation;
    record->estimate.state.covariance =
        joseph_left * record->estimate.state.covariance * joseph_left.transpose() +
        gain * measurement_covariance * gain.transpose();
    record->estimate.state.covariance =
        0.5 * (record->estimate.state.covariance +
                record->estimate.state.covariance.transpose());
    record->estimate.state.position = state_vector.head<3>();
    record->estimate.state.velocity = state_vector.tail<3>();
  };

  measurementUpdate(detection.position, detectionPositionCovariance(detection), 0);
  if (detection.has_velocity) {
    measurementUpdate(detection.velocity, detectionVelocityCovariance(detection), 3);
  }
  record->estimate.state.stamp = stamp;
  record->estimate.state.acceleration = Eigen::Vector3d::Zero();
  record->estimate.state.model = prediction::PredictionModel::CV;
  record->estimate.state.shape = detectionShape(detection);
  record->estimate.last_detection_stamp = stamp;
  record->estimate.missed_duration = 0.0;
  record->estimate.consecutive_hits += 1U;
  record->estimate.total_hits += 1U;
  if (record->estimate.lifecycle == LifecycleState::LOST ||
      record->estimate.lifecycle == LifecycleState::OCCLUDED) {
    record->estimate.lifecycle = record->estimate.total_hits >= options_.confirmation_match_count
                                    ? LifecycleState::CONFIRMED
                                    : LifecycleState::TENTATIVE;
  } else if (record->estimate.consecutive_hits >= options_.confirmation_match_count) {
    record->estimate.lifecycle = LifecycleState::CONFIRMED;
  }
}

ObstacleTracker::TrackRecord ObstacleTracker::makeNewRecord(const Detection &detection,
                                                             double stamp,
                                                             std::uint64_t track_id) const {
  TrackRecord record;
  record.estimate.state.track_id = track_id;
  record.estimate.state.stamp = stamp;
  record.estimate.state.position = detection.position;
  record.estimate.state.velocity = detection.has_velocity ? detection.velocity
                                                          : Eigen::Vector3d::Zero();
  record.estimate.state.acceleration = Eigen::Vector3d::Zero();
  record.estimate.state.has_covariance = true;
  record.estimate.state.covariance = makeInitialCovariance(
      detectionPositionCovariance(detection), detectionVelocityCovariance(detection));
  record.estimate.state.existence_probability = 1.0;
  record.estimate.state.shape = detectionShape(detection);
  record.estimate.state.model = prediction::PredictionModel::CV;
  record.estimate.lifecycle = LifecycleState::TENTATIVE;
  record.estimate.last_detection_stamp = stamp;
  record.estimate.missed_duration = 0.0;
  record.estimate.total_hits = 1U;
  record.estimate.consecutive_hits = 1U;
  return record;
}

void ObstacleTracker::sortEstimates(std::vector<TrackEstimate> *estimates) {
  std::sort(estimates->begin(), estimates->end(), [](const TrackEstimate &left,
                                                     const TrackEstimate &right) {
    return left.state.track_id < right.state.track_id;
  });
}

TrackingResult ObstacleTracker::update(double stamp, const std::vector<Detection> &detections) {
  TrackingResult result;
  result.stamp = stamp;
  if (!finite(stamp) || stamp < 0.0) {
    result.status = TrackingStatus::INVALID_INPUT;
    result.detail = "tracking timestamp must be finite and non-negative";
    return result;
  }
  if (has_update_ && stamp <= last_update_stamp_ + kEpsilon) {
    result.status = TrackingStatus::STALE_INPUT;
    result.detail = "tracking timestamp must be strictly newer than the previous batch";
    result.tracks = snapshot();
    return result;
  }

  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(detections.size());
  for (std::size_t index = 0U; index < detections.size(); ++index) {
    if (std::abs(detections[index].stamp - stamp) > 1e-6 ||
        !validateDetection(detections[index])) {
      ++result.invalid_detection_count;
    } else {
      valid_indices.push_back(index);
    }
  }
  if (result.invalid_detection_count > 0U && valid_indices.empty() && !detections.empty()) {
    result.status = TrackingStatus::PARTIAL_INPUT;
    result.detail = "all detections in the batch are invalid; tracker state was not advanced";
    result.tracks = snapshot();
    return result;
  }

  std::vector<TrackRecord> working_records = records_;
  std::vector<TrackRecord> candidate_records;
  candidate_records.reserve(working_records.size());
  for (auto &record : working_records) {
    const double missed_duration = stamp - record.estimate.last_detection_stamp;
    if (missed_duration < options_.deleted_after - kEpsilon) {
      if (!predictRecordTo(&record, stamp)) {
        result.status = TrackingStatus::INVALID_INPUT;
        result.detail = "track prediction failed while advancing to detection time";
        result.tracks = snapshot();
        return result;
      }
      candidate_records.push_back(std::move(record));
    } else {
      ++result.deleted_count;
    }
  }
  working_records = std::move(candidate_records);

  std::vector<std::vector<CandidateMetric>> metrics(
      working_records.size(), std::vector<CandidateMetric>(valid_indices.size()));
  for (std::size_t track_index = 0U; track_index < working_records.size(); ++track_index) {
    for (std::size_t detection_index = 0U; detection_index < valid_indices.size();
         ++detection_index) {
      metrics[track_index][detection_index] =
          associationMetric(working_records[track_index], detections[valid_indices[detection_index]]);
    }
  }
  const std::vector<int> assignment = solveAssignment(metrics);
  std::vector<bool> matched_detections(valid_indices.size(), false);
  for (std::size_t track_index = 0U; track_index < working_records.size(); ++track_index) {
    const int assignment_index = track_index < assignment.size() ? assignment[track_index] : -1;
    if (assignment_index >= 0 &&
        static_cast<std::size_t>(assignment_index) < valid_indices.size()) {
      const std::size_t compact_detection_index = static_cast<std::size_t>(assignment_index);
      const Detection &detection = detections[valid_indices[compact_detection_index]];
      updateMatchedRecord(&working_records[track_index], detection, stamp);
      matched_detections[compact_detection_index] = true;
      const CandidateMetric &metric = metrics[track_index][compact_detection_index];
      result.associations.push_back({working_records[track_index].estimate.state.track_id,
                                     valid_indices[compact_detection_index], metric.distance,
                                     metric.mahalanobis});
      ++result.associated_count;
    } else {
      TrackEstimate &estimate = working_records[track_index].estimate;
      estimate.consecutive_hits = 0U;
      if (estimate.missed_duration >= options_.lost_after - kEpsilon) {
        estimate.lifecycle = LifecycleState::LOST;
      } else {
        estimate.lifecycle = LifecycleState::OCCLUDED;
      }
    }
  }

  for (std::size_t detection_index = 0U; detection_index < valid_indices.size();
       ++detection_index) {
    if (!matched_detections[detection_index]) {
      std::uint64_t track_id = next_track_id_++;
      if (track_id == 0U) {
        throw std::overflow_error("obstacle tracker ID space exhausted");
      }
      working_records.push_back(makeNewRecord(
          detections[valid_indices[detection_index]], stamp, track_id));
      ++result.created_count;
    }
  }

  records_ = std::move(working_records);
  last_update_stamp_ = stamp;
  has_update_ = true;
  result.status = result.invalid_detection_count == 0U ? TrackingStatus::SUCCESS
                                                        : TrackingStatus::PARTIAL_INPUT;
  result.detail = result.invalid_detection_count == 0U
                      ? "detection batch associated and lifecycle state updated"
                      : "valid detections processed, but batch was incomplete";
  result.tracks = snapshot();
  return result;
}

std::vector<TrackEstimate> ObstacleTracker::snapshot() const {
  std::vector<TrackEstimate> result;
  result.reserve(records_.size());
  for (const auto &record : records_) {
    result.push_back(record.estimate);
  }
  sortEstimates(&result);
  return result;
}

void ObstacleTracker::reset() {
  records_.clear();
  // reset clears active state for a new tracking epoch, but IDs remain
  // monotonic for the lifetime of this tracker instance. This prevents a
  // delayed observation from being mistaken for a recycled obstacle.
  last_update_stamp_ = -1.0;
  has_update_ = false;
}

}  // namespace aurora::tracking
