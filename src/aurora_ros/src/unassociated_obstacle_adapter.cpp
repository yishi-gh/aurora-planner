#include "aurora_ros/unassociated_obstacle_adapter.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aurora::ros {
namespace {

constexpr double kShapeEpsilon = 1e-12;
constexpr double kCovarianceTolerance = 1e-9;
constexpr double kMinimumVariance = 1e-9;

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point &value) {
  return {value.x, value.y, value.z};
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 &value) {
  return {value.x, value.y, value.z};
}

bool finite(double value) { return std::isfinite(value); }

}  // namespace

UnassociatedObstacleAdapter::UnassociatedObstacleAdapter(std::string expected_frame,
                                                         double time_tolerance)
    : expected_frame_(std::move(expected_frame)), time_tolerance_(time_tolerance) {
  if (expected_frame_.empty() || !finite(time_tolerance_) || time_tolerance_ < 0.0) {
    throw std::invalid_argument("invalid unassociated obstacle adapter options");
  }
}

bool UnassociatedObstacleAdapter::validRosTime(const builtin_interfaces::msg::Time &time) {
  return time.sec >= 0 && time.nanosec < 1000000000U;
}

double UnassociatedObstacleAdapter::timeToSeconds(
    const builtin_interfaces::msg::Time &time) {
  return static_cast<double>(time.sec) + 1e-9 * static_cast<double>(time.nanosec);
}

bool UnassociatedObstacleAdapter::toCovariance(const std::array<double, 9> &values,
                                               Eigen::Matrix3d *covariance) {
  Eigen::Matrix3d input;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      input(row, column) = values[static_cast<std::size_t>(row * 3 + column)];
    }
  }
  if (!input.allFinite() ||
      (input - input.transpose()).cwiseAbs().maxCoeff() > kCovarianceTolerance) {
    return false;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().minCoeff() < -kCovarianceTolerance) {
    return false;
  }
  const Eigen::Vector3d eigenvalues =
      solver.eigenvalues().cwiseMax(kMinimumVariance);
  *covariance = solver.eigenvectors() * eigenvalues.asDiagonal() *
                solver.eigenvectors().transpose();
  *covariance = 0.5 * (*covariance + covariance->transpose());
  return covariance->allFinite();
}

bool UnassociatedObstacleAdapter::toShape(
    const aurora_msgs::msg::UnassociatedObstacleDetection &message,
    aurora::prediction::ObstacleShape *shape) {
  shape->dimensions = toEigen(message.dimensions);
  shape->radius = message.radius;
  if (!shape->dimensions.allFinite()) {
    return false;
  }
  switch (message.shape_type) {
    case aurora_msgs::msg::UnassociatedObstacleDetection::SPHERE:
      shape->type = aurora::prediction::ShapeType::SPHERE;
      return finite(message.radius) && message.radius >= 0.0;
    case aurora_msgs::msg::UnassociatedObstacleDetection::BOX:
      shape->type = aurora::prediction::ShapeType::BOX;
      return (shape->dimensions.array() > 0.0).all();
    case aurora_msgs::msg::UnassociatedObstacleDetection::CAPSULE:
      shape->type = aurora::prediction::ShapeType::CAPSULE;
      return finite(message.radius) && message.radius >= 0.0 &&
             (shape->dimensions.array() >= 0.0).all() &&
             shape->dimensions.norm() > kShapeEpsilon;
    case aurora_msgs::msg::UnassociatedObstacleDetection::MULTI_SPHERE:
      shape->type = aurora::prediction::ShapeType::MULTI_SPHERE;
      return finite(message.radius) && message.radius >= 0.0 &&
             (shape->dimensions.array() >= 0.0).all() &&
             (shape->dimensions.norm() > kShapeEpsilon ||
              message.radius > kShapeEpsilon);
    default:
      return false;
  }
}

bool UnassociatedObstacleAdapter::toDetection(
    const aurora_msgs::msg::UnassociatedObstacleDetection &message,
    double batch_stamp, const std::string &batch_frame,
    aurora::tracking::Detection *detection) const {
  if (!validRosTime(message.header.stamp) || message.header.frame_id != batch_frame ||
      std::abs(timeToSeconds(message.header.stamp) - batch_stamp) > time_tolerance_) {
    return false;
  }
  detection->stamp = batch_stamp;
  detection->position = toEigen(message.position);
  if (!detection->position.allFinite()) {
    return false;
  }
  detection->has_position_covariance = message.has_position_covariance;
  if (detection->has_position_covariance &&
      !toCovariance(message.position_covariance, &detection->position_covariance)) {
    return false;
  }
  detection->has_velocity = message.has_velocity;
  detection->velocity = toEigen(message.velocity);
  if (detection->has_velocity && !detection->velocity.allFinite()) {
    return false;
  }
  detection->has_velocity_covariance = message.has_velocity_covariance;
  if (detection->has_velocity_covariance && !detection->has_velocity) {
    return false;
  }
  if (detection->has_velocity_covariance &&
      !toCovariance(message.velocity_covariance, &detection->velocity_covariance)) {
    return false;
  }
  detection->has_shape = message.has_shape;
  if (detection->has_shape && !toShape(message, &detection->shape)) {
    return false;
  }
  return true;
}

UnassociatedDetectionBatch UnassociatedObstacleAdapter::convert(
    const aurora_msgs::msg::UnassociatedObstacleDetectionArray &message) const {
  UnassociatedDetectionBatch batch;
  batch.has_batch = true;
  batch.occlusion_active = message.occlusion_active;
  batch.stamp = timeToSeconds(message.header.stamp);
  batch.valid_header = validRosTime(message.header.stamp) && finite(batch.stamp) &&
                       message.header.frame_id == expected_frame_;
  if (!batch.valid_header) {
    batch.invalid_detection_count = 1U;
    return batch;
  }

  batch.detections.reserve(message.detections.size());
  for (const auto &message_detection : message.detections) {
    aurora::tracking::Detection detection;
    if (!toDetection(message_detection, batch.stamp, message.header.frame_id,
                     &detection)) {
      ++batch.invalid_detection_count;
      continue;
    }
    batch.detections.push_back(std::move(detection));
  }
  return batch;
}

}  // namespace aurora::ros
