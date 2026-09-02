#pragma once

#include "aurora_prediction/kinematic_predictor.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace aurora::tracking {

enum class LifecycleState {
  TENTATIVE,
  CONFIRMED,
  OCCLUDED,
  LOST,
};

enum class TrackingStatus {
  SUCCESS,
  PARTIAL_INPUT,
  INVALID_INPUT,
  STALE_INPUT,
};

const char *toString(LifecycleState state) noexcept;
const char *toString(TrackingStatus status) noexcept;

using Covariance3 = Eigen::Matrix3d;

// Detection fields intentionally describe one unassociated measurement. The
// tracker accepts the smallest useful 3D input and uses optional fields when
// the upstream detector provides them.
struct Detection {
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};

  bool has_position_covariance{false};
  Covariance3 position_covariance{Covariance3::Zero()};

  bool has_velocity{false};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  bool has_velocity_covariance{false};
  Covariance3 velocity_covariance{Covariance3::Zero()};

  bool has_shape{false};
  prediction::ObstacleShape shape;
};

struct TrackEstimate {
  prediction::TrackState state;
  LifecycleState lifecycle{LifecycleState::TENTATIVE};
  double last_detection_stamp{0.0};
  double missed_duration{0.0};
  std::size_t total_hits{0U};
  std::size_t consecutive_hits{0U};
};

struct Association {
  std::uint64_t track_id{0U};
  std::size_t detection_index{0U};
  double distance{std::numeric_limits<double>::infinity()};
  bool mahalanobis{false};
};

struct TrackingResult {
  TrackingStatus status{TrackingStatus::INVALID_INPUT};
  std::string detail;
  double stamp{0.0};
  std::size_t invalid_detection_count{0U};
  std::size_t associated_count{0U};
  std::size_t created_count{0U};
  std::size_t deleted_count{0U};
  std::vector<Association> associations;
  std::vector<TrackEstimate> tracks;
};

struct ObstacleTrackerOptions {
  // Gate values are distances, not squared distances.
  double mahalanobis_gate{3.0};
  double euclidean_gate{1.5};
  double covariance_tolerance{1e-9};
  double minimum_measurement_variance{1e-6};
  double default_position_variance{0.25};
  double default_velocity_variance{1.0};
  double process_noise_acceleration{1.0};

  std::size_t confirmation_match_count{2U};
  double lost_after{0.5};
  double deleted_after{2.0};
  std::uint64_t first_track_id{1U};

  prediction::ObstacleShape default_shape;
};

class ObstacleTracker {
public:
  explicit ObstacleTracker(ObstacleTrackerOptions options = {});

  const ObstacleTrackerOptions &options() const noexcept { return options_; }

  // A batch must be strictly newer than the previous accepted batch. Invalid
  // detections are never silently treated as an empty heartbeat.
  TrackingResult update(double stamp, const std::vector<Detection> &detections);

  std::vector<TrackEstimate> snapshot() const;
  void reset();

private:
  struct TrackRecord {
    TrackEstimate estimate;
  };

  struct CandidateMetric {
    bool valid{false};
    double distance{std::numeric_limits<double>::infinity()};
    double normalized_cost{std::numeric_limits<double>::infinity()};
    bool mahalanobis{false};
  };

  static void validateOptions(const ObstacleTrackerOptions &options);
  bool validateDetection(const Detection &detection) const;
  bool validateShape(const prediction::ObstacleShape &shape) const;
  bool prepareCovariance(const Covariance3 &input, Covariance3 *output) const;
  Covariance3 detectionPositionCovariance(const Detection &detection) const;
  Covariance3 detectionVelocityCovariance(const Detection &detection) const;
  prediction::ObstacleShape detectionShape(const Detection &detection) const;

  bool predictRecordTo(TrackRecord *record, double stamp) const;
  CandidateMetric associationMetric(const TrackRecord &record,
                                    const Detection &detection) const;
  std::vector<int> solveAssignment(
      const std::vector<std::vector<CandidateMetric>> &metrics) const;
  void updateMatchedRecord(TrackRecord *record, const Detection &detection,
                           double stamp) const;
  TrackRecord makeNewRecord(const Detection &detection, double stamp,
                            std::uint64_t track_id) const;
  static void sortEstimates(std::vector<TrackEstimate> *estimates);

  ObstacleTrackerOptions options_;
  prediction::KinematicPredictor predictor_;
  std::vector<TrackRecord> records_;
  std::uint64_t next_track_id_{1U};
  double last_update_stamp_{-1.0};
  bool has_update_{false};
};

}  // namespace aurora::tracking
