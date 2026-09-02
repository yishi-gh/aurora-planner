#pragma once

#include "aurora_msgs/msg/unassociated_obstacle_detection_array.hpp"
#include "aurora_tracking/obstacle_tracker.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace aurora::ros {

struct UnassociatedDetectionBatch {
  bool has_batch{false};
  bool valid_header{false};
  double stamp{0.0};
  std::vector<aurora::tracking::Detection> detections;
  std::size_t invalid_detection_count{0U};
  bool occlusion_active{false};
};

class UnassociatedObstacleAdapter {
public:
  UnassociatedObstacleAdapter(std::string expected_frame, double time_tolerance);

  const std::string &expectedFrame() const noexcept { return expected_frame_; }
  double timeTolerance() const noexcept { return time_tolerance_; }

  // Receipt of a malformed batch is preserved in has_batch/stamp and its
  // invalid count. Callers must fail closed instead of treating it as empty.
  UnassociatedDetectionBatch convert(
      const aurora_msgs::msg::UnassociatedObstacleDetectionArray &message) const;

private:
  static bool validRosTime(const builtin_interfaces::msg::Time &time);
  static double timeToSeconds(const builtin_interfaces::msg::Time &time);
  static bool toCovariance(const std::array<double, 9> &values,
                           Eigen::Matrix3d *covariance);
  static bool toShape(const aurora_msgs::msg::UnassociatedObstacleDetection &message,
                      aurora::prediction::ObstacleShape *shape);
  bool toDetection(
      const aurora_msgs::msg::UnassociatedObstacleDetection &message,
      double batch_stamp, const std::string &batch_frame,
      aurora::tracking::Detection *detection) const;

  std::string expected_frame_;
  double time_tolerance_{0.0};
};

}  // namespace aurora::ros
