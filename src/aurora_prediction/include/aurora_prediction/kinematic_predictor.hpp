#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace aurora::prediction {

enum class PredictionModel {
  CV,
  CA,
};

enum class PredictionStatus {
  SUCCESS,
  INVALID_OPTIONS,
  INVALID_INPUT,
  HORIZON_EXCEEDED,
  SAMPLE_LIMIT,
};

const char *toString(PredictionModel model) noexcept;
const char *toString(PredictionStatus status) noexcept;

enum class ShapeType {
  SPHERE,
  BOX,
  CAPSULE,
  MULTI_SPHERE,
};

struct ObstacleShape {
  ShapeType type{ShapeType::SPHERE};
  Eigen::Vector3d dimensions{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

using Covariance6 = Eigen::Matrix<double, 6, 6>;

// The covariance ordering is [px, py, pz, vx, vy, vz]. The CA model keeps
// acceleration internally and publishes this 6x6 marginal for a stable
// transport contract shared with the CV model.
struct TrackState {
  std::uint64_t track_id{0};
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool has_covariance{false};
  Covariance6 covariance{Covariance6::Zero()};
  double existence_probability{1.0};
  ObstacleShape shape;
  PredictionModel model{PredictionModel::CV};
};

struct PredictedState {
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Covariance6 covariance{Covariance6::Zero()};
  double existence_probability{1.0};
  double mode_probability{1.0};
  ObstacleShape shape;
};

struct PredictionResult {
  PredictionStatus status{PredictionStatus::INVALID_INPUT};
  std::string detail;
  std::uint64_t track_id{0};
  double reference_stamp{0.0};
  PredictionModel model{PredictionModel::CV};
  bool covariance_defaulted{false};
  bool covariance_regularized{false};
  bool acceleration_covariance_defaulted{false};
  std::vector<PredictedState> states;
};

struct KinematicPredictorOptions {
  double sample_interval{0.1};
  double max_horizon{5.0};
  std::size_t max_samples{10000};

  // Continuous white-noise spectral densities. The CV model uses acceleration
  // noise; the CA model uses jerk noise.
  double process_noise_acceleration{1.0};
  double process_noise_jerk{1.0};

  double default_position_variance{0.25};
  double default_velocity_variance{1.0};
  double default_acceleration_variance{4.0};
  double covariance_tolerance{1e-9};
};

class KinematicPredictor {
public:
  explicit KinematicPredictor(KinematicPredictorOptions options = {});

  const KinematicPredictorOptions &options() const noexcept { return options_; }

  PredictionResult predict(const TrackState &track, double horizon) const;

private:
  static void validateOptions(const KinematicPredictorOptions &options);

  KinematicPredictorOptions options_;
};

}  // namespace aurora::prediction
