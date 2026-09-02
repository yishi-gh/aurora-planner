#include "aurora_flight_adapter/trajectory_admission.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

aurora_msgs::msg::Trajectory makeTrajectory() {
  aurora_msgs::msg::Trajectory message;
  message.header.stamp.sec = 10;
  message.header.frame_id = "map";
  message.trajectory_id = 17U;
  message.validation_state = aurora_msgs::msg::Trajectory::VALIDATED;
  message.safety_report.accepted = true;

  aurora_msgs::msg::TrajectorySegment segment;
  segment.start_stamp.sec = 10;
  segment.duration = 1.0;
  segment.dt = 0.25;
  segment.degree = 3;
  segment.knot_mode = aurora_msgs::msg::TrajectorySegment::CLAMPED;
  for (int index = 0; index < 7; ++index) {
    geometry_msgs::msg::Point point;
    point.x = -4.0 + 0.5 * static_cast<double>(index);
    point.y = 0.0;
    point.z = 1.0 + 0.2 * static_cast<double>(index);
    segment.control_points.push_back(point);
  }
  message.segments.push_back(segment);
  return message;
}

}  // namespace

TEST(TrajectoryAdmission, AcceptsValidatedThreeDimensionalTrajectory) {
  aurora::flight::TrajectoryAdmission admission;
  const auto result = admission.admit(makeTrajectory(), 10.0);
  ASSERT_TRUE(result.accepted);
  ASSERT_TRUE(result.trajectory.has_value());
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::ACCEPTED);
  ASSERT_FALSE(result.trajectory->setpoints.empty());
  EXPECT_GT(result.trajectory->setpoints.back().position.z(), 1.0);
  EXPECT_TRUE(result.trajectory->setpoints.back().position.allFinite());
  EXPECT_LE(result.trajectory->setpoints.size(), admission.options().max_setpoints);
}

TEST(TrajectoryAdmission, RejectsUnvalidatedUnsafeExpiredAndWrongFrame) {
  aurora::flight::TrajectoryAdmission admission;
  auto message = makeTrajectory();
  message.validation_state = aurora_msgs::msg::Trajectory::DEGRADED;
  auto result = admission.admit(message, 10.0);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::REJECTED_UNVALIDATED);

  message = makeTrajectory();
  message.safety_report.accepted = false;
  result = admission.admit(message, 10.0);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::REJECTED_UNSAFE);

  result = admission.admit(makeTrajectory(), 12.0);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::EXPIRED);

  message = makeTrajectory();
  message.header.frame_id = "odom";
  result = admission.admit(message, 10.0);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::FRAME_MISMATCH);
}

TEST(TrajectoryAdmission, RejectsContradictorySafetyReport) {
  aurora::flight::TrajectoryAdmission admission;
  auto trajectory = makeTrajectory();
  trajectory.safety_report.accepted = true;
  trajectory.safety_report.status = aurora_msgs::msg::SafetyReport::STATIC_COLLISION;

  const auto result = admission.admit(trajectory, 10.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::REJECTED_UNSAFE);
}

TEST(TrajectoryAdmission, RejectsMalformedSegmentAndSetpointOverflow) {
  auto message = makeTrajectory();
  message.segments.front().knot_mode = 42U;
  aurora::flight::TrajectoryAdmission admission;
  auto result = admission.admit(message, 10.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::REJECTED_INVALID);

  aurora::flight::AdapterOptions options;
  options.max_setpoints = 2U;
  aurora::flight::TrajectoryAdmission limited(options);
  result = limited.admit(makeTrajectory(), 10.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::SETPOINT_LIMIT);
}

TEST(TrajectoryAdmission, MapsFeedbackToReplanOrEmergencyStop) {
  aurora::flight::TrajectoryAdmission admission;
  auto feedback = aurora_msgs::msg::TrajectoryExecutionStatus();
  feedback.header.stamp.sec = 10;
  feedback.trajectory_id = 17U;
  feedback.status = aurora_msgs::msg::TrajectoryExecutionStatus::ACTIVE;
  feedback.accepted = true;
  auto result = admission.observeFeedback(feedback, 17U, 10.1);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.action, aurora::flight::FeedbackAction::KEEP_EXECUTING);

  feedback.accepted = false;
  feedback.status = aurora_msgs::msg::TrajectoryExecutionStatus::REJECTED_UNSAFE;
  result = admission.observeFeedback(feedback, 17U, 10.1);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.action, aurora::flight::FeedbackAction::REQUEST_REPLAN);

  feedback.status = aurora_msgs::msg::TrajectoryExecutionStatus::TIME_GAP;
  result = admission.observeFeedback(feedback, 17U, 10.1);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.action, aurora::flight::FeedbackAction::EMERGENCY_STOP);

  result = admission.observeFeedback(feedback, 18U, 10.1);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.action, aurora::flight::FeedbackAction::REQUEST_REPLAN);
}

TEST(TrajectoryAdmission, RequiresResetForEmergencyStopAdmission) {
  aurora::flight::TrajectoryAdmission admission;
  const auto result = admission.admit(makeTrajectory(), 10.0, true);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, aurora::flight::AdmissionStatus::EMERGENCY_STOP);
}
