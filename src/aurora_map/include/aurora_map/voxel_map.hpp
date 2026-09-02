#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <vector>

namespace aurora::map {

struct GridIndex {
  int x{0};
  int y{0};
  int z{0};
};

bool operator==(const GridIndex &lhs, const GridIndex &rhs) noexcept;
bool operator!=(const GridIndex &lhs, const GridIndex &rhs) noexcept;

enum class MapState {
  FREE,
  OCCUPIED,
  UNKNOWN,
  OUT_OF_MAP,
};

struct VoxelMapConfig {
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3i dimensions{Eigen::Vector3i::Zero()};
  double resolution{0.5};
  double occupancy_threshold{0.8};
  double p_hit{0.65};
  double p_miss{0.35};
  double p_min{0.12};
  double p_max{0.90};
};

struct MapQueryResult {
  MapState state{MapState::UNKNOWN};
  double occupancy_probability{0.5};
  bool inflated{false};
  double observation_age{std::numeric_limits<double>::infinity()};
  double confidence{0.0};
  std::uint64_t map_version{0};
};

struct RayObservation {
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d endpoint{Eigen::Vector3d::Zero()};
  bool hit{true};
  double max_range{std::numeric_limits<double>::infinity()};
  double observation_age{0.0};
  double confidence{1.0};
};

struct RayUpdateStats {
  std::size_t traversed_voxels{0};
  std::size_t miss_updates{0};
  bool endpoint_marked{false};
};

class VoxelMap {
public:
  explicit VoxelMap(VoxelMapConfig config);

  const VoxelMapConfig &config() const noexcept { return config_; }
  const Eigen::Vector3d &origin() const noexcept { return config_.origin; }
  const Eigen::Vector3i &dimensions() const noexcept { return config_.dimensions; }
  double resolution() const noexcept { return config_.resolution; }
  std::size_t voxelCount() const noexcept { return cells_.size(); }
  std::uint64_t version() const noexcept { return map_version_; }

  bool isInMap(const Eigen::Vector3d &position) const noexcept;
  bool isInMap(const GridIndex &index) const noexcept;
  GridIndex worldToIndex(const Eigen::Vector3d &position) const;
  Eigen::Vector3d indexToWorld(const GridIndex &index) const;

  MapQueryResult query(const Eigen::Vector3d &position) const;
  MapQueryResult query(const GridIndex &index) const;

  // Sets a measured voxel. age is in seconds and confidence is in [0, 1].
  void setOccupancy(const GridIndex &index, double occupancy_probability,
                   double observation_age = 0.0, double confidence = 1.0);
  void setUnknown(const GridIndex &index);

  // Integrates one ray. Intermediate cells receive p_miss; a valid hit endpoint receives p_hit.
  RayUpdateStats integrateRay(const RayObservation &observation);

  // Adds a closed geometric box using cell-center inclusion and clips at map bounds.
  void addBox(const Eigen::Vector3d &minimum, const Eigen::Vector3d &maximum,
              double observation_age = 0.0, double confidence = 1.0);

  // Builds a spherical inflated layer around occupied voxels.
  void inflate(double radius);
  bool hasInflation() const noexcept { return inflation_valid_; }
  double inflationRadius() const noexcept { return inflation_radius_; }

private:
  struct Cell {
    double log_odds{0.0};
    double observation_age{std::numeric_limits<double>::infinity()};
    double confidence{0.0};
    bool known{false};
  };

  static void validateConfig(const VoxelMapConfig &config);
  static void validateMeasurement(double occupancy_probability, double observation_age,
                                  double confidence);
  static void validateRayObservation(const RayObservation &observation);
  static double probabilityToLogOdds(double probability);
  static double logOddsToProbability(double log_odds);

  std::size_t address(const GridIndex &index) const noexcept;
  void invalidateInflation() noexcept;
  MapQueryResult outOfMapResult() const noexcept;
  void applyLogOddsUpdate(const GridIndex &index, double update_probability,
                          double observation_age, double confidence);

  VoxelMapConfig config_;
  std::vector<Cell> cells_;
  std::vector<std::uint8_t> inflated_;
  bool inflation_valid_{false};
  double inflation_radius_{0.0};
  std::uint64_t map_version_{0};
};

}  // namespace aurora::map
