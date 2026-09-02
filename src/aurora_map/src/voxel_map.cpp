#include "aurora_map/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aurora::map {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kProbabilityEpsilon = 1e-12;

bool isFinite(double value) { return std::isfinite(value); }

}  // namespace

bool operator==(const GridIndex &lhs, const GridIndex &rhs) noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const GridIndex &lhs, const GridIndex &rhs) noexcept { return !(lhs == rhs); }

VoxelMap::VoxelMap(VoxelMapConfig config) : config_(std::move(config)) {
  validateConfig(config_);
  const std::int64_t count = static_cast<std::int64_t>(config_.dimensions.x()) *
                             static_cast<std::int64_t>(config_.dimensions.y()) *
                             static_cast<std::int64_t>(config_.dimensions.z());
  if (count <= 0 || static_cast<std::uint64_t>(count) >
                        static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    throw std::invalid_argument("voxel map size is invalid or too large");
  }
  cells_.resize(static_cast<std::size_t>(count));
  inflated_.assign(cells_.size(), 0U);
}

void VoxelMap::validateConfig(const VoxelMapConfig &config) {
  if (!config.origin.allFinite() || config.dimensions.x() <= 0 || config.dimensions.y() <= 0 ||
      config.dimensions.z() <= 0 || !isFinite(config.resolution) || config.resolution <= 0.0 ||
      !isFinite(config.occupancy_threshold) || config.occupancy_threshold <= 0.0 ||
      config.occupancy_threshold > 1.0 || !isFinite(config.p_hit) || config.p_hit <= 0.0 ||
      config.p_hit >= 1.0 || !isFinite(config.p_miss) || config.p_miss <= 0.0 ||
      config.p_miss >= 1.0 || !isFinite(config.p_min) || config.p_min <= 0.0 ||
      config.p_min >= 1.0 || !isFinite(config.p_max) || config.p_max <= 0.0 ||
      config.p_max >= 1.0 || config.p_min > config.p_max) {
    throw std::invalid_argument("invalid voxel map configuration");
  }
}

void VoxelMap::validateMeasurement(double occupancy_probability, double observation_age,
                                   double confidence) {
  if (!isFinite(occupancy_probability) || occupancy_probability < 0.0 ||
      occupancy_probability > 1.0 || !isFinite(observation_age) || observation_age < 0.0 ||
      !isFinite(confidence) || confidence < 0.0 || confidence > 1.0) {
    throw std::invalid_argument("invalid voxel measurement");
  }
}

void VoxelMap::validateRayObservation(const RayObservation &observation) {
  if (!observation.origin.allFinite() || !observation.endpoint.allFinite() ||
      (!isFinite(observation.max_range) && !std::isinf(observation.max_range)) ||
      observation.max_range < 0.0 || !isFinite(observation.observation_age) ||
      observation.observation_age < 0.0 || !isFinite(observation.confidence) ||
      observation.confidence < 0.0 || observation.confidence > 1.0) {
    throw std::invalid_argument("invalid ray observation");
  }
}

double VoxelMap::probabilityToLogOdds(double probability) {
  const double bounded = std::clamp(probability, kProbabilityEpsilon, 1.0 - kProbabilityEpsilon);
  return std::log(bounded / (1.0 - bounded));
}

double VoxelMap::logOddsToProbability(double log_odds) {
  if (log_odds >= 0.0) {
    const double exponential = std::exp(-log_odds);
    return 1.0 / (1.0 + exponential);
  }
  const double exponential = std::exp(log_odds);
  return exponential / (1.0 + exponential);
}

bool VoxelMap::isInMap(const Eigen::Vector3d &position) const noexcept {
  if (!position.allFinite()) {
    return false;
  }
  const Eigen::Vector3d maximum =
      config_.origin + config_.resolution * config_.dimensions.cast<double>();
  return (position.array() >= config_.origin.array()).all() && (position.array() < maximum.array()).all();
}

bool VoxelMap::isInMap(const GridIndex &index) const noexcept {
  return index.x >= 0 && index.x < config_.dimensions.x() && index.y >= 0 &&
         index.y < config_.dimensions.y() && index.z >= 0 && index.z < config_.dimensions.z();
}

GridIndex VoxelMap::worldToIndex(const Eigen::Vector3d &position) const {
  if (!isInMap(position)) {
    throw std::out_of_range("position is outside voxel map");
  }
  const Eigen::Vector3d relative = (position - config_.origin) / config_.resolution;
  return {static_cast<int>(std::floor(relative.x())), static_cast<int>(std::floor(relative.y())),
          static_cast<int>(std::floor(relative.z()))};
}

Eigen::Vector3d VoxelMap::indexToWorld(const GridIndex &index) const {
  if (!isInMap(index)) {
    throw std::out_of_range("voxel index is outside map");
  }
  return config_.origin +
         config_.resolution * (Eigen::Vector3d(index.x, index.y, index.z) +
                               Eigen::Vector3d::Constant(0.5));
}

std::size_t VoxelMap::address(const GridIndex &index) const noexcept {
  return (static_cast<std::size_t>(index.x) * static_cast<std::size_t>(config_.dimensions.y()) +
          static_cast<std::size_t>(index.y)) * static_cast<std::size_t>(config_.dimensions.z()) +
         static_cast<std::size_t>(index.z);
}

MapQueryResult VoxelMap::outOfMapResult() const noexcept {
  MapQueryResult result;
  result.state = MapState::OUT_OF_MAP;
  result.occupancy_probability = 1.0;
  result.inflated = true;
  result.map_version = map_version_;
  return result;
}

MapQueryResult VoxelMap::query(const Eigen::Vector3d &position) const {
  if (!isInMap(position)) {
    return outOfMapResult();
  }
  return query(worldToIndex(position));
}

MapQueryResult VoxelMap::query(const GridIndex &index) const {
  if (!isInMap(index)) {
    return outOfMapResult();
  }

  const std::size_t cell_address = address(index);
  const Cell &cell = cells_[cell_address];
  const double occupancy_probability = cell.known ? logOddsToProbability(cell.log_odds) : 0.5;
  const bool raw_occupied = cell.known && occupancy_probability >= config_.occupancy_threshold;
  const bool inflated_occupied = inflation_valid_ && inflated_[cell_address] != 0U;

  MapQueryResult result;
  result.occupancy_probability = occupancy_probability;
  result.inflated = inflated_occupied;
  result.observation_age = cell.observation_age;
  result.confidence = cell.confidence;
  result.map_version = map_version_;
  if (raw_occupied || inflated_occupied) {
    result.state = MapState::OCCUPIED;
  } else if (!cell.known) {
    result.state = MapState::UNKNOWN;
  } else {
    result.state = MapState::FREE;
  }
  return result;
}

void VoxelMap::invalidateInflation() noexcept {
  inflation_valid_ = false;
  inflation_radius_ = 0.0;
  std::fill(inflated_.begin(), inflated_.end(), 0U);
}

void VoxelMap::setOccupancy(const GridIndex &index, double occupancy_probability,
                            double observation_age, double confidence) {
  if (!isInMap(index)) {
    throw std::out_of_range("voxel index is outside map");
  }
  validateMeasurement(occupancy_probability, observation_age, confidence);
  Cell &cell = cells_[address(index)];
  cell.log_odds = probabilityToLogOdds(occupancy_probability);
  cell.observation_age = observation_age;
  cell.confidence = confidence;
  cell.known = true;
  invalidateInflation();
  ++map_version_;
}

void VoxelMap::setUnknown(const GridIndex &index) {
  if (!isInMap(index)) {
    throw std::out_of_range("voxel index is outside map");
  }
  Cell &cell = cells_[address(index)];
  cell.log_odds = 0.0;
  cell.observation_age = std::numeric_limits<double>::infinity();
  cell.confidence = 0.0;
  cell.known = false;
  invalidateInflation();
  ++map_version_;
}

void VoxelMap::applyLogOddsUpdate(const GridIndex &index, double update_probability,
                                  double observation_age, double confidence) {
  Cell &cell = cells_[address(index)];
  const double min_log_odds = probabilityToLogOdds(config_.p_min);
  const double max_log_odds = probabilityToLogOdds(config_.p_max);
  cell.log_odds = std::clamp(cell.log_odds + probabilityToLogOdds(update_probability),
                             min_log_odds, max_log_odds);
  cell.observation_age = observation_age;
  cell.confidence = confidence;
  cell.known = true;
}

void VoxelMap::addBox(const Eigen::Vector3d &minimum, const Eigen::Vector3d &maximum,
                      double observation_age, double confidence) {
  if (!minimum.allFinite() || !maximum.allFinite() ||
      (maximum.array() <= minimum.array()).any()) {
    throw std::invalid_argument("box maximum must be greater than minimum");
  }
  validateMeasurement(1.0, observation_age, confidence);

  const Eigen::Vector3d relative_min = (minimum - config_.origin) / config_.resolution;
  const Eigen::Vector3d relative_max = (maximum - config_.origin) / config_.resolution;
  GridIndex lower{static_cast<int>(std::floor(relative_min.x())),
                  static_cast<int>(std::floor(relative_min.y())),
                  static_cast<int>(std::floor(relative_min.z()))};
  GridIndex upper{static_cast<int>(std::ceil(relative_max.x()) - 1.0),
                  static_cast<int>(std::ceil(relative_max.y()) - 1.0),
                  static_cast<int>(std::ceil(relative_max.z()) - 1.0)};
  lower.x = std::max(0, lower.x);
  lower.y = std::max(0, lower.y);
  lower.z = std::max(0, lower.z);
  upper.x = std::min(config_.dimensions.x() - 1, upper.x);
  upper.y = std::min(config_.dimensions.y() - 1, upper.y);
  upper.z = std::min(config_.dimensions.z() - 1, upper.z);
  if (lower.x > upper.x || lower.y > upper.y || lower.z > upper.z) {
    return;
  }

  for (int x = lower.x; x <= upper.x; ++x) {
    for (int y = lower.y; y <= upper.y; ++y) {
      for (int z = lower.z; z <= upper.z; ++z) {
        Cell &cell = cells_[address({x, y, z})];
        cell.log_odds = probabilityToLogOdds(config_.p_max);
        cell.observation_age = observation_age;
        cell.confidence = confidence;
        cell.known = true;
      }
    }
  }
  invalidateInflation();
  ++map_version_;
}

RayUpdateStats VoxelMap::integrateRay(const RayObservation &observation) {
  validateRayObservation(observation);

  const Eigen::Vector3d difference = observation.endpoint - observation.origin;
  const double length = difference.norm();
  const bool endpoint_reached = length <= observation.max_range + kEpsilon;
  const Eigen::Vector3d effective_endpoint =
      endpoint_reached || length <= kEpsilon
          ? observation.endpoint
          : observation.origin + difference * (observation.max_range / length);

  auto clip_segment = [&](const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                          double &entry, double &exit) {
    entry = 0.0;
    exit = 1.0;
    const Eigen::Vector3d minimum = config_.origin;
    const Eigen::Vector3d maximum =
        config_.origin + config_.resolution * config_.dimensions.cast<double>();
    const Eigen::Vector3d segment = end - start;
    for (int axis = 0; axis < 3; ++axis) {
      const double coordinate = start(axis);
      const double component = segment(axis);
      if (std::abs(component) <= kEpsilon) {
        if (coordinate < minimum(axis) || coordinate > maximum(axis)) {
          return false;
        }
        continue;
      }
      double near_value = (minimum(axis) - coordinate) / component;
      double far_value = (maximum(axis) - coordinate) / component;
      if (near_value > far_value) {
        std::swap(near_value, far_value);
      }
      entry = std::max(entry, near_value);
      exit = std::min(exit, far_value);
      if (entry > exit + kEpsilon) {
        return false;
      }
    }
    return exit >= -kEpsilon && entry <= 1.0 + kEpsilon;
  };

  RayUpdateStats stats;
  if (length <= kEpsilon) {
    if (isInMap(observation.origin)) {
      const GridIndex index = worldToIndex(observation.origin);
      applyLogOddsUpdate(index, observation.hit ? config_.p_hit : config_.p_miss,
                         observation.observation_age, observation.confidence);
      stats.traversed_voxels = 1;
      stats.endpoint_marked = observation.hit;
      stats.miss_updates = observation.hit ? 0 : 1;
      invalidateInflation();
      ++map_version_;
    }
    return stats;
  }

  if ((effective_endpoint - observation.origin).norm() <= kEpsilon) {
    return stats;
  }

  double entry = 0.0;
  double exit = 1.0;
  if (!clip_segment(observation.origin, effective_endpoint, entry, exit)) {
    return stats;
  }

  const Eigen::Vector3d segment = effective_endpoint - observation.origin;
  const double entry_epsilon = std::min(1e-9, (exit - entry) * 0.5);
  const double initial_parameter = std::min(exit, entry + entry_epsilon);
  const Eigen::Vector3d initial_point = observation.origin + initial_parameter * segment;
  const Eigen::Vector3d map_min = config_.origin;
  const Eigen::Vector3d map_max =
      config_.origin + config_.resolution * config_.dimensions.cast<double>();
  Eigen::Vector3d safe_initial = initial_point;
  for (int axis = 0; axis < 3; ++axis) {
    safe_initial(axis) = std::clamp(safe_initial(axis), map_min(axis),
                                    std::nextafter(map_max(axis), map_min(axis)));
  }
  GridIndex current = worldToIndex(safe_initial);
  const Eigen::Vector3d ray_origin = observation.origin;
  const Eigen::Vector3d ray_direction = segment / segment.norm();
  const double entry_distance = entry * segment.norm();
  const double exit_distance = exit * segment.norm();

  GridIndex step{0, 0, 0};
  Eigen::Vector3d next_boundary = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_distance = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d delta_distance = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  for (int axis = 0; axis < 3; ++axis) {
    const int current_index = axis == 0 ? current.x : (axis == 1 ? current.y : current.z);
    if (ray_direction(axis) > kEpsilon) {
      step.x = axis == 0 ? 1 : step.x;
      step.y = axis == 1 ? 1 : step.y;
      step.z = axis == 2 ? 1 : step.z;
      next_boundary(axis) = map_min(axis) +
                            static_cast<double>(current_index + 1) * config_.resolution;
      delta_distance(axis) = config_.resolution / ray_direction(axis);
    } else if (ray_direction(axis) < -kEpsilon) {
      step.x = axis == 0 ? -1 : step.x;
      step.y = axis == 1 ? -1 : step.y;
      step.z = axis == 2 ? -1 : step.z;
      next_boundary(axis) = map_min(axis) +
                            static_cast<double>(current_index) * config_.resolution;
      delta_distance(axis) = -config_.resolution / ray_direction(axis);
    }
    if (std::isfinite(delta_distance(axis))) {
      max_distance(axis) = (next_boundary(axis) - ray_origin(axis)) / ray_direction(axis);
      while (max_distance(axis) <= entry_distance + kEpsilon) {
        max_distance(axis) += delta_distance(axis);
      }
    }
  }

  std::vector<GridIndex> traversed;
  const double tie_epsilon = 1e-10;
  while (isInMap(current)) {
    traversed.push_back(current);
    const double next_distance = std::min({max_distance.x(), max_distance.y(), max_distance.z()});
    if (next_distance > exit_distance + tie_epsilon) {
      break;
    }
    if (max_distance.x() <= next_distance + tie_epsilon) {
      current.x += step.x;
      max_distance.x() += delta_distance.x();
    }
    if (max_distance.y() <= next_distance + tie_epsilon) {
      current.y += step.y;
      max_distance.y() += delta_distance.y();
    }
    if (max_distance.z() <= next_distance + tie_epsilon) {
      current.z += step.z;
      max_distance.z() += delta_distance.z();
    }
  }

  if (traversed.empty()) {
    return stats;
  }
  stats.traversed_voxels = traversed.size();
  GridIndex endpoint_index{};
  bool mark_endpoint = false;
  if (observation.hit && endpoint_reached && isInMap(observation.endpoint)) {
    endpoint_index = worldToIndex(observation.endpoint);
    mark_endpoint = true;
  }

  for (const GridIndex &index : traversed) {
    if (mark_endpoint && index == endpoint_index) {
      applyLogOddsUpdate(index, config_.p_hit, observation.observation_age, observation.confidence);
      stats.endpoint_marked = true;
    } else {
      applyLogOddsUpdate(index, config_.p_miss, observation.observation_age, observation.confidence);
      ++stats.miss_updates;
    }
  }
  invalidateInflation();
  ++map_version_;
  return stats;
}

void VoxelMap::inflate(double radius) {
  if (!isFinite(radius) || radius < 0.0) {
    throw std::invalid_argument("inflation radius must be finite and non-negative");
  }

  std::fill(inflated_.begin(), inflated_.end(), 0U);
  const int offset_limit = static_cast<int>(std::ceil(radius / config_.resolution));
  for (int x = 0; x < config_.dimensions.x(); ++x) {
    for (int y = 0; y < config_.dimensions.y(); ++y) {
      for (int z = 0; z < config_.dimensions.z(); ++z) {
        const GridIndex source{x, y, z};
        const Cell &source_cell = cells_[address(source)];
        if (!source_cell.known || logOddsToProbability(source_cell.log_odds) < config_.occupancy_threshold) {
          continue;
        }
        for (int dx = -offset_limit; dx <= offset_limit; ++dx) {
          for (int dy = -offset_limit; dy <= offset_limit; ++dy) {
            for (int dz = -offset_limit; dz <= offset_limit; ++dz) {
              const double distance = config_.resolution *
                                      std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
              if (distance > radius + kEpsilon) {
                continue;
              }
              const GridIndex target{x + dx, y + dy, z + dz};
              if (isInMap(target)) {
                inflated_[address(target)] = 1U;
              }
            }
          }
        }
      }
    }
  }
  inflation_radius_ = radius;
  inflation_valid_ = true;
  ++map_version_;
}

}  // namespace aurora::map
