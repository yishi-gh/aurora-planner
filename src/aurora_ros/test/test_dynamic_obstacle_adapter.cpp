#include "aurora_ros/dynamic_obstacle_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using aurora::ros::DynamicObstacleAdapter;
using aurora::ros::DynamicTrackSnapshot;
using aurora_msgs::msg::DynamicObstacleTrack;
using aurora_msgs::msg::DynamicObstacleTrackArray;

builtin_interfaces::msg::Time makeTime(double seconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(std::floor(seconds));
  time.nanosec = static_cast<std::uint32_t>(
      std::llround((seconds - std::floor(seconds)) * 1e9));
  return time;
}

DynamicObstacleTrack makeTrack(std::uint64_t id, double stamp = 10.0) {
  DynamicObstacleTrack track;
  track.header.stamp = makeTime(stamp);
  track.header.frame_id = "map";
  track.track_id = id;
  track.pose.position.x = 1.0;
  track.pose.position.y = -2.0;
  track.pose.position.z = 0.5;
  track.twist.linear.x = 2.0;
  track.twist.linear.y = 0.5;
  track.acceleration.linear.z = 0.25;
  track.shape_type = DynamicObstacleTrack::SPHERE;
  track.radius = 0.4;
  track.existence_probability = 0.9;
  track.prediction_model = DynamicObstacleTrack::CV;
  track.state_covariance.fill(0.0);
  return track;
}

DynamicObstacleTrackArray makeBatch() {
  DynamicObstacleTrackArray batch;
  batch.header.stamp = makeTime(10.0);
  batch.header.frame_id = "map";
  return batch;
}

TEST(DynamicObstacleAdapter, ConvertsValidCvAndCaTracks) {
  DynamicObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  auto cv_track = makeTrack(7U);
  auto ca_track = makeTrack(8U);
  ca_track.shape_type = DynamicObstacleTrack::BOX;
  ca_track.dimensions.x = 1.0;
  ca_track.dimensions.y = 2.0;
  ca_track.dimensions.z = 3.0;
  ca_track.radius = 0.0;
  ca_track.prediction_model = DynamicObstacleTrack::CA;
  ca_track.has_state_covariance = true;
  ca_track.state_covariance[0] = 0.04;
  ca_track.state_covariance[7] = 0.09;
  ca_track.state_covariance[14] = 0.16;
  batch.tracks = {cv_track, ca_track};

  const DynamicTrackSnapshot snapshot = adapter.convert(batch);

  EXPECT_TRUE(snapshot.has_snapshot);
  EXPECT_DOUBLE_EQ(snapshot.stamp, 10.0);
  EXPECT_EQ(snapshot.invalid_track_count, 0U);
  ASSERT_EQ(snapshot.tracks.size(), 2U);
  EXPECT_EQ(snapshot.tracks[0].track_id, 7U);
  EXPECT_EQ(snapshot.tracks[0].model, aurora::prediction::PredictionModel::CV);
  EXPECT_DOUBLE_EQ(snapshot.tracks[0].shape.radius, 0.4);
  EXPECT_EQ(snapshot.tracks[1].track_id, 8U);
  EXPECT_EQ(snapshot.tracks[1].model, aurora::prediction::PredictionModel::CA);
  EXPECT_EQ(snapshot.tracks[1].shape.type, aurora::prediction::ShapeType::BOX);
  EXPECT_TRUE(snapshot.tracks[1].has_covariance);
  EXPECT_DOUBLE_EQ(snapshot.tracks[1].covariance(1, 1), 0.09);
}

TEST(DynamicObstacleAdapter, PreservesExplicitEmptyBatchHeartbeat) {
  DynamicObstacleAdapter adapter("map", 1e-6);
  const DynamicTrackSnapshot snapshot = adapter.convert(makeBatch());

  EXPECT_TRUE(snapshot.has_snapshot);
  EXPECT_TRUE(snapshot.tracks.empty());
  EXPECT_EQ(snapshot.invalid_track_count, 0U);
}

TEST(DynamicObstacleAdapter, PreservesExplicitOcclusionMetadata) {
  DynamicObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  batch.occlusion_active = true;
  batch.occluded_track_ids = {7U, 9U};

  const auto snapshot = adapter.convert(batch);

  EXPECT_TRUE(snapshot.has_snapshot);
  EXPECT_TRUE(snapshot.occlusion_active);
  ASSERT_EQ(snapshot.occluded_track_ids.size(), 2U);
  EXPECT_EQ(snapshot.occluded_track_ids[0], 7U);
  EXPECT_EQ(snapshot.occluded_track_ids[1], 9U);
}

TEST(DynamicObstacleAdapter, NonEmptyOcclusionIdsAreExplicitlyUnusable) {
  DynamicObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  batch.occluded_track_ids = {12U};

  const auto snapshot = adapter.convert(batch);

  EXPECT_TRUE(snapshot.occlusion_active);
  EXPECT_EQ(snapshot.occluded_track_ids.size(), 1U);
}

TEST(DynamicObstacleAdapter, CountsInvalidTargetsWithoutDroppingValidOnes) {
  DynamicObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  auto valid = makeTrack(1U);
  auto duplicate = makeTrack(1U);
  auto wrong_frame = makeTrack(2U);
  wrong_frame.header.frame_id = "camera";
  auto wrong_model = makeTrack(3U);
  wrong_model.prediction_model = 99U;
  auto wrong_shape = makeTrack(4U);
  wrong_shape.shape_type = DynamicObstacleTrack::BOX;
  wrong_shape.dimensions.x = 0.0;
  wrong_shape.dimensions.y = 1.0;
  wrong_shape.dimensions.z = 1.0;
  batch.tracks = {valid, duplicate, wrong_frame, wrong_model, wrong_shape};

  const DynamicTrackSnapshot snapshot = adapter.convert(batch);

  EXPECT_TRUE(snapshot.has_snapshot);
  EXPECT_EQ(snapshot.invalid_track_count, 4U);
  ASSERT_EQ(snapshot.tracks.size(), 1U);
  EXPECT_EQ(snapshot.tracks.front().track_id, 1U);
}

TEST(DynamicObstacleAdapter, RejectsHeaderShapeAndNonFiniteStateErrors) {
  DynamicObstacleAdapter adapter("map", 1e-6);

  auto wrong_time_batch = makeBatch();
  wrong_time_batch.tracks.push_back(makeTrack(1U, 10.01));
  const auto wrong_time = adapter.convert(wrong_time_batch);
  EXPECT_EQ(wrong_time.invalid_track_count, 1U);

  auto nonfinite_batch = makeBatch();
  auto nonfinite = makeTrack(2U);
  nonfinite.has_state_covariance = true;
  nonfinite.state_covariance[0] = std::numeric_limits<double>::quiet_NaN();
  nonfinite_batch.tracks.push_back(nonfinite);
  const auto nonfinite_result = adapter.convert(nonfinite_batch);
  EXPECT_EQ(nonfinite_result.invalid_track_count, 1U);

  auto invalid_batch = makeBatch();
  invalid_batch.header.frame_id = "camera";
  invalid_batch.tracks.push_back(makeTrack(3U));
  const auto invalid_header = adapter.convert(invalid_batch);
  EXPECT_TRUE(invalid_header.has_snapshot);
  EXPECT_EQ(invalid_header.invalid_track_count, 1U);
  EXPECT_TRUE(invalid_header.tracks.empty());
}

TEST(DynamicObstacleAdapter, AcceptsCapsuleAndMultiSphereShapes) {
  DynamicObstacleAdapter adapter("map", 1e-6);
  auto batch = makeBatch();
  auto capsule = makeTrack(5U);
  capsule.shape_type = DynamicObstacleTrack::CAPSULE;
  capsule.radius = 0.2;
  capsule.dimensions.x = 1.5;
  auto multi_sphere = makeTrack(6U);
  multi_sphere.shape_type = DynamicObstacleTrack::MULTI_SPHERE;
  multi_sphere.radius = 0.1;
  multi_sphere.dimensions.x = 2.0;
  batch.tracks = {capsule, multi_sphere};

  const auto snapshot = adapter.convert(batch);

  EXPECT_EQ(snapshot.invalid_track_count, 0U);
  ASSERT_EQ(snapshot.tracks.size(), 2U);
  EXPECT_EQ(snapshot.tracks[0].shape.type, aurora::prediction::ShapeType::CAPSULE);
  EXPECT_EQ(snapshot.tracks[1].shape.type, aurora::prediction::ShapeType::MULTI_SPHERE);
}

TEST(DynamicObstacleAdapter, RejectsInvalidOptions) {
  EXPECT_THROW(DynamicObstacleAdapter("", 1e-6), std::invalid_argument);
  EXPECT_THROW(DynamicObstacleAdapter("map", -1.0), std::invalid_argument);
  EXPECT_THROW(DynamicObstacleAdapter("map", std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

}  // namespace
