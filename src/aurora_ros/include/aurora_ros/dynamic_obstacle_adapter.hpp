#pragma once

#include "aurora_msgs/msg/dynamic_obstacle_track_array.hpp"
#include "aurora_prediction/kinematic_predictor.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aurora::ros {

struct DynamicTrackSnapshot {
  bool has_snapshot{false};
  bool valid_header{false};
  double stamp{0.0};
  std::vector<aurora::prediction::TrackState> tracks;
  std::size_t invalid_track_count{0U};
  bool occlusion_active{false};
  std::vector<std::uint64_t> occluded_track_ids;
  bool information_incomplete{false};
  std::string incomplete_detail;
  bool source_conflict{false};
};

class DynamicObstacleAdapter {
public:
  DynamicObstacleAdapter(std::string expected_frame, double time_tolerance);

  const std::string &expectedFrame() const noexcept { return expected_frame_; }
  double timeTolerance() const noexcept { return time_tolerance_; }

  // The returned snapshot records receipt even when the batch is malformed.
  // invalid_track_count makes incomplete input fail closed in the risk gate.
  DynamicTrackSnapshot convert(
      const aurora_msgs::msg::DynamicObstacleTrackArray &message) const;

private:
  static bool validRosTime(const builtin_interfaces::msg::Time &time);
  static double timeToSeconds(const builtin_interfaces::msg::Time &time);
  static bool toShape(const aurora_msgs::msg::DynamicObstacleTrack &message,
                      aurora::prediction::ObstacleShape *shape);
  bool toTrackState(const aurora_msgs::msg::DynamicObstacleTrack &message,
                    double batch_stamp, const std::string &batch_frame,
                    aurora::prediction::TrackState *track) const;

  std::string expected_frame_;
  double time_tolerance_{0.0};
};

}  // namespace aurora::ros
