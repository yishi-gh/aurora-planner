#include "aurora_ros/unassociated_obstacle_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using aurora::ros::UnassociatedObstacleAdapter;
using aurora_msgs::msg::UnassociatedObstacleDetection;
using aurora_msgs::msg::UnassociatedObstacleDetectionArray;

builtin_interfaces::msg::Time makeTime(double seconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(std::floor(seconds));
  time.nanosec = static_cast<std::uint32_t>(
      std::llround((seconds - std::floor(seconds)) * 1e9));
  return time;
}

UnassociatedObstacleDetection makeDetection(double stamp = 10.0) {
  UnassociatedObstacleDetection detection;
  detection.header.stamp = makeTime(stamp);
  detection.header.frame_id = "map";
  detection.position.x = 1.0;
  detection.position.y = -2.0;
  detection.position.z = 0.5;
  return detection;
}

UnassociatedObstacleDetectionArray makeBatch() {
  UnassociatedObstacleDetectionArray batch;
  batch.header.stamp = makeTime(10.0);
  batch.header.frame_id = "map";
  return batch;
}

TEST(UnassociatedObstacleAdapter, ConvertsPositionAndOptionalMeasurements) {
  UnassociatedObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  auto detection = makeDetection();
  detection.has_position_covariance = true;
  detection.position_covariance.fill(0.0);
  detection.position_covariance[0] = 0.04;
  detection.position_covariance[4] = 0.09;
  detection.position_covariance[8] = 0.16;
  detection.has_velocity = true;
  detection.velocity.x = 2.0;
  detection.velocity.z = -0.25;
  detection.has_velocity_covariance = true;
  detection.velocity_covariance.fill(0.0);
  detection.velocity_covariance[0] = 0.01;
  detection.velocity_covariance[4] = 0.02;
  detection.velocity_covariance[8] = 0.03;
  batch.detections.push_back(detection);

  const auto result = adapter.convert(batch);

  EXPECT_TRUE(result.has_batch);
  EXPECT_DOUBLE_EQ(result.stamp, 10.0);
  EXPECT_EQ(result.invalid_detection_count, 0U);
  ASSERT_EQ(result.detections.size(), 1U);
  const auto &converted = result.detections.front();
  EXPECT_DOUBLE_EQ(converted.position.x(), 1.0);
  EXPECT_TRUE(converted.has_position_covariance);
  EXPECT_DOUBLE_EQ(converted.position_covariance(1, 1), 0.09);
  EXPECT_TRUE(converted.has_velocity);
  EXPECT_DOUBLE_EQ(converted.velocity.z(), -0.25);
  EXPECT_TRUE(converted.has_velocity_covariance);
  EXPECT_DOUBLE_EQ(converted.velocity_covariance(2, 2), 0.03);
}

TEST(UnassociatedObstacleAdapter, ConvertsOptionalShapesAndUsesDetectionStamp) {
  UnassociatedObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  auto sphere = makeDetection();
  sphere.has_shape = true;
  sphere.shape_type = UnassociatedObstacleDetection::SPHERE;
  sphere.radius = 0.4;
  auto box = makeDetection();
  box.has_shape = true;
  box.shape_type = UnassociatedObstacleDetection::BOX;
  box.dimensions.x = 1.0;
  box.dimensions.y = 2.0;
  box.dimensions.z = 3.0;
  batch.detections = {sphere, box};

  const auto result = adapter.convert(batch);

  ASSERT_EQ(result.detections.size(), 2U);
  EXPECT_TRUE(result.detections[0].has_shape);
  EXPECT_EQ(result.detections[0].shape.type, aurora::prediction::ShapeType::SPHERE);
  EXPECT_DOUBLE_EQ(result.detections[0].shape.radius, 0.4);
  EXPECT_EQ(result.detections[1].shape.type, aurora::prediction::ShapeType::BOX);
  EXPECT_DOUBLE_EQ(result.detections[1].shape.dimensions.y(), 2.0);
}

TEST(UnassociatedObstacleAdapter, AcceptsValidEmptyHeartbeatAndOcclusionFlag) {
  UnassociatedObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  batch.occlusion_active = true;

  const auto result = adapter.convert(batch);

  EXPECT_TRUE(result.has_batch);
  EXPECT_TRUE(result.detections.empty());
  EXPECT_EQ(result.invalid_detection_count, 0U);
  EXPECT_TRUE(result.occlusion_active);
}

TEST(UnassociatedObstacleAdapter, KeepsValidDetectionsAndCountsInvalidOnes) {
  UnassociatedObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  auto valid = makeDetection();
  auto wrong_frame = makeDetection();
  wrong_frame.header.frame_id = "camera";
  auto wrong_time = makeDetection(10.1);
  auto wrong_covariance = makeDetection();
  wrong_covariance.has_position_covariance = true;
  wrong_covariance.position_covariance.fill(0.0);
  wrong_covariance.position_covariance[0] = 1.0;
  wrong_covariance.position_covariance[1] = 2.0;
  batch.detections = {valid, wrong_frame, wrong_time, wrong_covariance};

  const auto result = adapter.convert(batch);

  EXPECT_EQ(result.invalid_detection_count, 3U);
  ASSERT_EQ(result.detections.size(), 1U);
  EXPECT_DOUBLE_EQ(result.detections.front().position.x(), 1.0);
}

TEST(UnassociatedObstacleAdapter, RejectsInvalidOptionalFieldCombinations) {
  UnassociatedObstacleAdapter adapter("map", 1e-6);

  auto no_velocity = makeBatch();
  auto velocity_covariance_without_velocity = makeDetection();
  velocity_covariance_without_velocity.has_velocity_covariance = true;
  velocity_covariance_without_velocity.velocity_covariance.fill(0.0);
  velocity_covariance_without_velocity.velocity_covariance[0] = 1.0;
  velocity_covariance_without_velocity.velocity_covariance[4] = 1.0;
  velocity_covariance_without_velocity.velocity_covariance[8] = 1.0;
  no_velocity.detections.push_back(velocity_covariance_without_velocity);
  EXPECT_EQ(adapter.convert(no_velocity).invalid_detection_count, 1U);

  auto nonfinite = makeBatch();
  auto nonfinite_detection = makeDetection();
  nonfinite_detection.position.x = std::numeric_limits<double>::quiet_NaN();
  nonfinite.detections.push_back(nonfinite_detection);
  EXPECT_EQ(adapter.convert(nonfinite).invalid_detection_count, 1U);

  auto invalid_shape = makeBatch();
  auto invalid_shape_detection = makeDetection();
  invalid_shape_detection.has_shape = true;
  invalid_shape_detection.shape_type = UnassociatedObstacleDetection::BOX;
  invalid_shape_detection.dimensions.x = 0.0;
  invalid_shape_detection.dimensions.y = 1.0;
  invalid_shape_detection.dimensions.z = 1.0;
  invalid_shape.detections.push_back(invalid_shape_detection);
  EXPECT_EQ(adapter.convert(invalid_shape).invalid_detection_count, 1U);
}

TEST(UnassociatedObstacleAdapter, RejectsInvalidBatchHeaderAndOptions) {
  UnassociatedObstacleAdapter adapter("map", 1e-6);
  auto wrong_frame = makeBatch();
  wrong_frame.header.frame_id = "camera";
  wrong_frame.detections.push_back(makeDetection());
  const auto result = adapter.convert(wrong_frame);
  EXPECT_TRUE(result.has_batch);
  EXPECT_EQ(result.invalid_detection_count, 1U);
  EXPECT_TRUE(result.detections.empty());

  EXPECT_THROW(UnassociatedObstacleAdapter("", 1e-6), std::invalid_argument);
  EXPECT_THROW(UnassociatedObstacleAdapter("map", -1.0), std::invalid_argument);
  EXPECT_THROW(UnassociatedObstacleAdapter(
                   "map", std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

}  // namespace
