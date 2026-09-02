#include "aurora_tracking/obstacle_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aurora::prediction::ObstacleShape;
using aurora::prediction::ShapeType;
using aurora::tracking::Detection;
using aurora::tracking::LifecycleState;
using aurora::tracking::ObstacleTracker;
using aurora::tracking::ObstacleTrackerOptions;
using aurora::tracking::TrackingStatus;

Detection makeDetection(double stamp, double x, double y = 0.0, double z = 1.0) {
  Detection detection;
  detection.stamp = stamp;
  detection.position = {x, y, z};
  detection.has_shape = true;
  detection.shape.type = ShapeType::SPHERE;
  detection.shape.radius = 0.2;
  return detection;
}

ObstacleTrackerOptions makeOptions() {
  ObstacleTrackerOptions options;
  options.process_noise_acceleration = 0.5;
  options.euclidean_gate = 0.8;
  options.mahalanobis_gate = 3.0;
  options.default_shape.type = ShapeType::SPHERE;
  options.default_shape.radius = 0.5;
  return options;
}

TEST(ObstacleTracker, StartsTentativeAndConfirmsAfterTwoConsecutiveMatches) {
  ObstacleTracker tracker(makeOptions());
  auto first = tracker.update(1.0, {makeDetection(1.0, 0.0)});
  ASSERT_EQ(first.status, TrackingStatus::SUCCESS);
  ASSERT_EQ(first.tracks.size(), 1U);
  EXPECT_EQ(first.tracks.front().lifecycle, LifecycleState::TENTATIVE);
  EXPECT_EQ(first.tracks.front().total_hits, 1U);

  auto second_detection = makeDetection(1.1, 0.05);
  auto second = tracker.update(1.1, {second_detection});
  ASSERT_EQ(second.status, TrackingStatus::SUCCESS);
  ASSERT_EQ(second.associated_count, 1U);
  ASSERT_EQ(second.created_count, 0U);
  ASSERT_EQ(second.tracks.size(), 1U);
  EXPECT_EQ(second.tracks.front().state.track_id, first.tracks.front().state.track_id);
  EXPECT_EQ(second.tracks.front().lifecycle, LifecycleState::CONFIRMED);
}

TEST(ObstacleTracker, UsesMahalanobisWhenCovarianceIsAvailable) {
  ObstacleTracker tracker(makeOptions());
  auto first = makeDetection(1.0, 0.0);
  first.has_position_covariance = true;
  first.position_covariance = 0.01 * Eigen::Matrix3d::Identity();
  ASSERT_EQ(tracker.update(1.0, {first}).status, TrackingStatus::SUCCESS);

  auto second = makeDetection(1.1, 0.15);
  second.has_position_covariance = true;
  second.position_covariance = Eigen::Matrix3d::Identity();
  auto result = tracker.update(1.1, {second});
  ASSERT_EQ(result.associations.size(), 1U);
  EXPECT_TRUE(result.associations.front().mahalanobis);
}

TEST(ObstacleTracker, FallsBackToEuclideanDistanceWithoutDetectionCovariance) {
  ObstacleTracker tracker(makeOptions());
  ASSERT_EQ(tracker.update(1.0, {makeDetection(1.0, 0.0)}).status,
            TrackingStatus::SUCCESS);
  auto result = tracker.update(1.1, {makeDetection(1.1, 0.7)});
  ASSERT_EQ(result.associations.size(), 1U);
  EXPECT_FALSE(result.associations.front().mahalanobis);
}

TEST(ObstacleTracker, UsesGlobalOneToOneAssignmentDeterministically) {
  ObstacleTrackerOptions options = makeOptions();
  options.euclidean_gate = 2.0;
  ObstacleTracker tracker(options);
  auto first = tracker.update(1.0, {makeDetection(1.0, 0.0), makeDetection(1.0, 1.0)});
  ASSERT_EQ(first.created_count, 2U);

  // Both tracks can see both detections. The globally lower total cost pairs
  // the left track with 0.8 and the right track with 0.1.
  auto result = tracker.update(1.1, {makeDetection(1.1, 0.8), makeDetection(1.1, 0.1)});
  ASSERT_EQ(result.associated_count, 2U);
  ASSERT_EQ(result.associations.size(), 2U);
  EXPECT_EQ(result.associations[0].track_id, first.tracks[0].state.track_id);
  EXPECT_EQ(result.associations[0].detection_index, 1U);
  EXPECT_EQ(result.associations[1].track_id, first.tracks[1].state.track_id);
  EXPECT_EQ(result.associations[1].detection_index, 0U);
}

TEST(ObstacleTracker, AppliesOptionalVelocityMeasurement) {
  ObstacleTracker tracker(makeOptions());
  ASSERT_EQ(tracker.update(1.0, {makeDetection(1.0, 0.0)}).status,
            TrackingStatus::SUCCESS);
  auto detection = makeDetection(1.1, 0.1);
  detection.has_velocity = true;
  detection.velocity = {2.0, 0.0, 0.0};
  auto result = tracker.update(1.1, {detection});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_GT(result.tracks.front().state.velocity.x(), 0.1);
}

TEST(ObstacleTracker, AdvancesOccludedTrackThenMarksLostAndDeletesIt) {
  ObstacleTrackerOptions options = makeOptions();
  options.lost_after = 0.5;
  options.deleted_after = 2.0;
  ObstacleTracker tracker(options);
  auto first = tracker.update(1.0, {makeDetection(1.0, 0.0)});
  const auto id = first.tracks.front().state.track_id;
  ASSERT_EQ(tracker.update(1.1, {makeDetection(1.1, 0.0)}).status,
            TrackingStatus::SUCCESS);

  auto occluded = tracker.update(1.4, {});
  ASSERT_EQ(occluded.tracks.size(), 1U);
  EXPECT_EQ(occluded.tracks.front().lifecycle, LifecycleState::OCCLUDED);
  const double covariance_before = occluded.tracks.front().state.covariance(0, 0);

  auto lost = tracker.update(1.7, {});
  ASSERT_EQ(lost.tracks.size(), 1U);
  EXPECT_EQ(lost.tracks.front().state.track_id, id);
  EXPECT_EQ(lost.tracks.front().lifecycle, LifecycleState::LOST);
  EXPECT_GT(lost.tracks.front().state.covariance(0, 0), covariance_before);

  auto deleted = tracker.update(3.2, {});
  EXPECT_TRUE(deleted.tracks.empty());
  EXPECT_EQ(deleted.deleted_count, 1U);
}

TEST(ObstacleTracker, ReacquisitionKeepsIdentityAndNewTracksNeverReuseIds) {
  ObstacleTrackerOptions options = makeOptions();
  options.lost_after = 0.5;
  options.deleted_after = 2.0;
  ObstacleTracker tracker(options);
  const auto first = tracker.update(1.0, {makeDetection(1.0, 0.0)});
  const auto id = first.tracks.front().state.track_id;
  ASSERT_EQ(tracker.update(1.1, {makeDetection(1.1, 0.0)}).status,
            TrackingStatus::SUCCESS);
  ASSERT_EQ(tracker.update(1.7, {}).tracks.front().lifecycle, LifecycleState::LOST);

  auto reacquired = tracker.update(1.8, {makeDetection(1.8, 0.1)});
  ASSERT_EQ(reacquired.associated_count, 1U);
  ASSERT_EQ(reacquired.tracks.size(), 1U);
  EXPECT_EQ(reacquired.tracks.front().state.track_id, id);
  EXPECT_EQ(reacquired.tracks.front().lifecycle, LifecycleState::CONFIRMED);

  ASSERT_EQ(tracker.update(4.0, {}).tracks.size(), 0U);
  auto new_result = tracker.update(4.1, {makeDetection(4.1, 0.1)});
  ASSERT_EQ(new_result.tracks.size(), 1U);
  EXPECT_GT(new_result.tracks.front().state.track_id, id);
}

TEST(ObstacleTracker, RejectsInvalidOrStaleBatchesWithoutAdvancingState) {
  ObstacleTracker tracker(makeOptions());
  ASSERT_EQ(tracker.update(1.0, {makeDetection(1.0, 0.0)}).status,
            TrackingStatus::SUCCESS);
  auto invalid = makeDetection(1.1, 0.1);
  invalid.has_position_covariance = true;
  invalid.position_covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
  auto invalid_result = tracker.update(1.1, {invalid});
  EXPECT_EQ(invalid_result.status, TrackingStatus::PARTIAL_INPUT);
  EXPECT_DOUBLE_EQ(invalid_result.tracks.front().state.stamp, 1.0);

  auto stale_result = tracker.update(1.0, {makeDetection(1.0, 0.0)});
  EXPECT_EQ(stale_result.status, TrackingStatus::STALE_INPUT);
  EXPECT_DOUBLE_EQ(stale_result.tracks.front().state.stamp, 1.0);
}

TEST(ObstacleTracker, ResetRestartsStateWithoutReusingIds) {
  ObstacleTracker tracker(makeOptions());
  auto first = tracker.update(1.0, {makeDetection(1.0, 0.0)});
  ASSERT_EQ(first.tracks.front().state.track_id, 1U);
  tracker.reset();
  EXPECT_TRUE(tracker.snapshot().empty());
  auto after_reset = tracker.update(2.0, {makeDetection(2.0, 0.0)});
  ASSERT_EQ(after_reset.tracks.size(), 1U);
  EXPECT_EQ(after_reset.tracks.front().state.track_id, 2U);
}

}  // namespace
