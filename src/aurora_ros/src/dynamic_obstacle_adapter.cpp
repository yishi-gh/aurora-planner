#include "aurora_ros/dynamic_obstacle_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace aurora::ros {
namespace {

constexpr double kShapeEpsilon = 1e-12;

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 &value) {
  return {value.x, value.y, value.z};
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &value) {
  return {value.x, value.y, value.z};
}

}  // namespace

DynamicObstacleAdapter::DynamicObstacleAdapter(std::string expected_frame,
                                               double time_tolerance)
    : expected_frame_(std::move(expected_frame)), time_tolerance_(time_tolerance) {
  if (expected_frame_.empty() || !std::isfinite(time_tolerance_) || time_tolerance_ < 0.0) {
    throw std::invalid_argument("invalid dynamic obstacle adapter options");
  }
}

bool DynamicObstacleAdapter::validRosTime(const builtin_interfaces::msg::Time &time) {
  return time.sec >= 0 && time.nanosec < 1000000000U;
}

double DynamicObstacleAdapter::timeToSeconds(const builtin_interfaces::msg::Time &time) {
  return static_cast<double>(time.sec) + 1e-9 * static_cast<double>(time.nanosec);
}

bool DynamicObstacleAdapter::toShape(
    const aurora_msgs::msg::DynamicObstacleTrack &message,
    aurora::prediction::ObstacleShape *shape) {
  shape->dimensions = toEigen(message.dimensions);
  shape->radius = message.radius;
  if (!shape->dimensions.allFinite()) {
    return false;
  }
  switch (message.shape_type) {
    case aurora_msgs::msg::DynamicObstacleTrack::SPHERE:
      shape->type = aurora::prediction::ShapeType::SPHERE;
      if (!std::isfinite(message.radius) || message.radius < 0.0) {
        return false;
      }
      break;
    case aurora_msgs::msg::DynamicObstacleTrack::BOX:
      shape->type = aurora::prediction::ShapeType::BOX;
      if ((shape->dimensions.array() <= 0.0).any()) {
        return false;
      }
      break;
    case aurora_msgs::msg::DynamicObstacleTrack::CAPSULE:
      shape->type = aurora::prediction::ShapeType::CAPSULE;
      if (!std::isfinite(message.radius) || message.radius < 0.0 ||
          (shape->dimensions.array() < 0.0).any() ||
          shape->dimensions.norm() <= kShapeEpsilon) {
        return false;
      }
      break;
    case aurora_msgs::msg::DynamicObstacleTrack::MULTI_SPHERE:
      shape->type = aurora::prediction::ShapeType::MULTI_SPHERE;
      if (!std::isfinite(message.radius) || message.radius < 0.0 ||
          (shape->dimensions.array() < 0.0).any() ||
          (shape->dimensions.norm() <= kShapeEpsilon && message.radius <= kShapeEpsilon)) {
        return false;
      }
      break;
    default:
      return false;
  }
  return true;
}

bool DynamicObstacleAdapter::toTrackState(
    const aurora_msgs::msg::DynamicObstacleTrack &message, double batch_stamp,
    const std::string &batch_frame, aurora::prediction::TrackState *track) const {
  if (message.header.frame_id != batch_frame || !validRosTime(message.header.stamp) ||
      std::abs(timeToSeconds(message.header.stamp) - batch_stamp) > time_tolerance_ ||
      !toShape(message, &track->shape)) {
    return false;
  }

  track->track_id = message.track_id;
  track->stamp = batch_stamp;
  track->position = toEigen(message.pose.position);
  track->velocity = toEigen(message.twist.linear);
  track->acceleration = toEigen(message.acceleration.linear);
  track->has_covariance = message.has_state_covariance;
  track->covariance.setZero();
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      track->covariance(row, column) =
          message.state_covariance[static_cast<std::size_t>(row * 6 + column)];
    }
  }
  track->existence_probability = message.existence_probability;
  switch (message.prediction_model) {
    case aurora_msgs::msg::DynamicObstacleTrack::CV:
      track->model = aurora::prediction::PredictionModel::CV;
      break;
    case aurora_msgs::msg::DynamicObstacleTrack::CA:
      track->model = aurora::prediction::PredictionModel::CA;
      break;
    default:
      return false;
  }
  return track->position.allFinite() && track->velocity.allFinite() &&
         track->acceleration.allFinite() && std::isfinite(track->existence_probability) &&
         track->existence_probability >= 0.0 && track->existence_probability <= 1.0 &&
         (!track->has_covariance || track->covariance.allFinite());
}

DynamicTrackSnapshot DynamicObstacleAdapter::convert(
    const aurora_msgs::msg::DynamicObstacleTrackArray &message) const {
  DynamicTrackSnapshot snapshot;
  snapshot.has_snapshot = true;
  snapshot.stamp = timeToSeconds(message.header.stamp);
  snapshot.occlusion_active = message.occlusion_active || !message.occluded_track_ids.empty();
  snapshot.occluded_track_ids = message.occluded_track_ids;
  snapshot.valid_header = validRosTime(message.header.stamp) &&
                          std::isfinite(snapshot.stamp) &&
                          message.header.frame_id == expected_frame_;
  if (!snapshot.valid_header) {
    snapshot.invalid_track_count = 1U;
    return snapshot;
  }

  std::unordered_set<std::uint64_t> track_ids;
  track_ids.reserve(message.tracks.size());
  snapshot.tracks.reserve(message.tracks.size());
  for (const auto &track_message : message.tracks) {
    aurora::prediction::TrackState track;
    const bool unique_id = track_ids.insert(track_message.track_id).second;
    if (!unique_id || !toTrackState(track_message, snapshot.stamp, message.header.frame_id,
                                    &track)) {
      ++snapshot.invalid_track_count;
      continue;
    }
    snapshot.tracks.push_back(std::move(track));
  }
  return snapshot;
}

}  // namespace aurora::ros
