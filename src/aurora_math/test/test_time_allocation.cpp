#include "aurora_math/time_allocation.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using aurora::math::TimeAllocationOptions;
using aurora::math::WaypointMatrix;
using aurora::math::allocateSegmentTimes;

WaypointMatrix makeLine() {
  WaypointMatrix waypoints(3, 3);
  waypoints << 0.0, 3.0, 6.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0;
  return waypoints;
}
}  // namespace

TEST(TimeAllocation, AllocatesFiniteTimesForStraightLine) {
  TimeAllocationOptions options;
  options.max_velocity = 10.0;
  options.max_acceleration = 10.0;
  options.max_jerk = 10.0;
  options.minimum_segment_time = 0.1;

  const Eigen::VectorXd times = allocateSegmentTimes(makeLine(), options);

  ASSERT_EQ(times.size(), 2);
  EXPECT_NEAR(times(0), std::cbrt(32.0 * 3.0 / 10.0), 1e-12);
  EXPECT_DOUBLE_EQ(times(0), times(1));
  EXPECT_TRUE(times.allFinite());
  EXPECT_GT(times.minCoeff(), 0.0);
}

TEST(TimeAllocation, GivesZeroLengthSegmentsTheMinimumTime) {
  WaypointMatrix waypoints(3, 4);
  waypoints << 1.0, 1.0, 1.0, 2.0,
               2.0, 2.0, 2.0, 2.0,
               3.0, 3.0, 3.0, 3.0;

  TimeAllocationOptions options;
  options.minimum_segment_time = 0.25;
  options.numerical_tolerance = 1e-6;

  const Eigen::VectorXd times = allocateSegmentTimes(waypoints, options);

  ASSERT_EQ(times.size(), 3);
  EXPECT_DOUBLE_EQ(times(0), 0.25);
  EXPECT_DOUBLE_EQ(times(1), 0.25);
  EXPECT_GT(times(2), 0.25);
}

TEST(TimeAllocation, LowerLimitsDoNotReduceAllocatedTime) {
  const WaypointMatrix waypoints = makeLine();
  TimeAllocationOptions baseline;
  baseline.max_velocity = 4.0;
  baseline.max_acceleration = 4.0;
  baseline.max_jerk = 4.0;
  baseline.minimum_segment_time = 0.1;
  baseline.time_scale = 1.0;
  const Eigen::VectorXd reference = allocateSegmentTimes(waypoints, baseline);

  TimeAllocationOptions stricter = baseline;
  stricter.max_velocity = 2.0;
  const Eigen::VectorXd velocity_times = allocateSegmentTimes(waypoints, stricter);
  stricter = baseline;
  stricter.max_acceleration = 2.0;
  const Eigen::VectorXd acceleration_times = allocateSegmentTimes(waypoints, stricter);
  stricter = baseline;
  stricter.max_jerk = 2.0;
  const Eigen::VectorXd jerk_times = allocateSegmentTimes(waypoints, stricter);
  stricter = baseline;
  stricter.minimum_segment_time = 2.0;
  const Eigen::VectorXd minimum_times = allocateSegmentTimes(waypoints, stricter);
  stricter = baseline;
  stricter.time_scale = 2.0;
  const Eigen::VectorXd scaled_times = allocateSegmentTimes(waypoints, stricter);

  for (Eigen::Index segment = 0; segment < reference.size(); ++segment) {
    EXPECT_GE(velocity_times(segment), reference(segment));
    EXPECT_GE(acceleration_times(segment), reference(segment));
    EXPECT_GE(jerk_times(segment), reference(segment));
    EXPECT_GE(minimum_times(segment), reference(segment));
    EXPECT_GE(scaled_times(segment), reference(segment));
  }
}

TEST(TimeAllocation, RejectsInvalidInputs) {
  EXPECT_THROW(allocateSegmentTimes(WaypointMatrix(3, 0)), std::invalid_argument);

  WaypointMatrix one_point(3, 1);
  one_point.setZero();
  EXPECT_THROW(allocateSegmentTimes(one_point), std::invalid_argument);

  WaypointMatrix waypoints = makeLine();
  waypoints(1, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(allocateSegmentTimes(waypoints), std::invalid_argument);

  waypoints = makeLine();
  TimeAllocationOptions options;
  options.max_velocity = 0.0;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
  options = {};
  options.max_acceleration = -1.0;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
  options = {};
  options.max_jerk = std::numeric_limits<double>::infinity();
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
  options = {};
  options.minimum_segment_time = 0.0;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
  options = {};
  options.time_scale = -0.5;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
  options = {};
  options.minimum_points = 1U;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
  options = {};
  options.numerical_tolerance = -1.0;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);

  options = {};
  options.minimum_points = 4U;
  EXPECT_THROW(allocateSegmentTimes(waypoints, options), std::invalid_argument);
}
