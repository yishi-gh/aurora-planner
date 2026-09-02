#include "aurora_search/astar_3d.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace aurora::search {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr std::size_t kInvalidNode = std::numeric_limits<std::size_t>::max();

struct OpenEntry {
  std::size_t flat_index{0};
  double g_score{0.0};
  double f_score{0.0};
  std::uint64_t sequence{0};
};

struct OpenEntryCompare {
  bool operator()(const OpenEntry &lhs, const OpenEntry &rhs) const noexcept {
    if (lhs.f_score != rhs.f_score) {
      return lhs.f_score > rhs.f_score;
    }
    if (lhs.g_score != rhs.g_score) {
      return lhs.g_score > rhs.g_score;
    }
    return lhs.sequence > rhs.sequence;
  }
};

std::size_t flatten(const aurora::map::GridIndex &index, const Eigen::Vector3i &dimensions) {
  return (static_cast<std::size_t>(index.x) * static_cast<std::size_t>(dimensions.y()) +
          static_cast<std::size_t>(index.y)) * static_cast<std::size_t>(dimensions.z()) +
         static_cast<std::size_t>(index.z);
}

aurora::map::GridIndex unflatten(std::size_t flat_index, const Eigen::Vector3i &dimensions) {
  const std::size_t plane_size = static_cast<std::size_t>(dimensions.y()) *
                                 static_cast<std::size_t>(dimensions.z());
  const int x = static_cast<int>(flat_index / plane_size);
  flat_index %= plane_size;
  const int y = static_cast<int>(flat_index / static_cast<std::size_t>(dimensions.z()));
  const int z = static_cast<int>(flat_index % static_cast<std::size_t>(dimensions.z()));
  return {x, y, z};
}

double heuristic(const aurora::map::GridIndex &from, const aurora::map::GridIndex &to,
                 double resolution) {
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  const double dz = static_cast<double>(from.z - to.z);
  return resolution * std::sqrt(dx * dx + dy * dy + dz * dz);
}

double stepCost(int dx, int dy, int dz, double resolution) {
  const int changed_axes = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
  return resolution * std::sqrt(static_cast<double>(changed_axes));
}

}  // namespace

const char *toString(SearchStatus status) noexcept {
  switch (status) {
    case SearchStatus::SUCCESS:
      return "SUCCESS";
    case SearchStatus::INVALID_OPTIONS:
      return "INVALID_OPTIONS";
    case SearchStatus::INVALID_START:
      return "INVALID_START";
    case SearchStatus::INVALID_GOAL:
      return "INVALID_GOAL";
    case SearchStatus::START_BLOCKED:
      return "START_BLOCKED";
    case SearchStatus::GOAL_BLOCKED:
      return "GOAL_BLOCKED";
    case SearchStatus::SEARCH_TIMEOUT:
      return "SEARCH_TIMEOUT";
    case SearchStatus::NO_PATH:
      return "NO_PATH";
  }
  return "UNKNOWN_STATUS";
}

SearchResult AStar3D::search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                             const SearchOptions &options) const {
  SearchResult result;

  if (!std::isfinite(options.max_compute_time_sec) || options.max_compute_time_sec < 0.0) {
    result.status = SearchStatus::INVALID_OPTIONS;
    result.detail = "max_compute_time_sec must be finite and non-negative";
    return result;
  }
  if (!start.allFinite()) {
    result.status = SearchStatus::INVALID_START;
    result.detail = "start position is not finite or is outside the map";
    return result;
  }
  if (!goal.allFinite()) {
    result.status = SearchStatus::INVALID_GOAL;
    result.detail = "goal position is not finite or is outside the map";
    return result;
  }
  if (!map_.isInMap(start)) {
    result.status = SearchStatus::INVALID_START;
    result.detail = "start position is outside the map";
    return result;
  }
  if (!map_.isInMap(goal)) {
    result.status = SearchStatus::INVALID_GOAL;
    result.detail = "goal position is outside the map";
    return result;
  }

  const aurora::map::GridIndex start_index = map_.worldToIndex(start);
  const aurora::map::GridIndex goal_index = map_.worldToIndex(goal);
  result.start_index = start_index;
  result.goal_index = goal_index;

  const auto is_traversable = [&](const aurora::map::GridIndex &index) {
    const auto state = map_.query(index).state;
    return state == aurora::map::MapState::FREE ||
           (options.allow_unknown && state == aurora::map::MapState::UNKNOWN);
  };
  if (!is_traversable(start_index)) {
    result.status = SearchStatus::START_BLOCKED;
    result.detail = "start voxel is occupied or unknown";
    return result;
  }
  if (!is_traversable(goal_index)) {
    result.status = SearchStatus::GOAL_BLOCKED;
    result.detail = "goal voxel is occupied or unknown";
    return result;
  }

  const Eigen::Vector3i dimensions = map_.dimensions();
  const double resolution = map_.resolution();
  const std::size_t start_flat = flatten(start_index, dimensions);
  const std::size_t goal_flat = flatten(goal_index, dimensions);
  const std::size_t node_count = map_.voxelCount();
  std::vector<double> g_scores(node_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parents(node_count, kInvalidNode);
  std::vector<std::uint8_t> closed(node_count, 0U);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryCompare> open_set;
  std::uint64_t sequence = 0;
  const auto search_start = std::chrono::steady_clock::now();

  const auto time_exceeded = [&]() {
    return options.max_compute_time_sec > 0.0 &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - search_start).count() >=
               options.max_compute_time_sec;
  };
  const auto make_timeout = [&]() {
    result.status = SearchStatus::SEARCH_TIMEOUT;
    result.detail = "search budget exceeded";
    return result;
  };

  g_scores[start_flat] = 0.0;
  open_set.push({start_flat, 0.0, heuristic(start_index, goal_index, resolution), sequence++});
  result.generated_nodes = 1;

  constexpr std::array<int, 3> kAxisOrder = {0, 1, 2};
  const auto transition_is_clear = [&](const aurora::map::GridIndex &current, int dx, int dy,
                                       int dz) {
    const std::array<int, 3> deltas = {dx, dy, dz};
    std::array<int, 3> changed_axes{};
    int changed_count = 0;
    for (const int axis : kAxisOrder) {
      if (deltas[axis] != 0) {
        changed_axes[changed_count++] = axis;
      }
    }

    // Check every proper non-empty subset of changed axes. For a diagonal
    // move this prevents both face and edge corner cutting in 3D.
    const int subset_count = 1 << changed_count;
    const int full_subset = subset_count - 1;
    for (int subset = 1; subset < full_subset; ++subset) {
      aurora::map::GridIndex intermediate = current;
      for (int bit = 0; bit < changed_count; ++bit) {
        if ((subset & (1 << bit)) != 0) {
          const int axis = changed_axes[bit];
          if (axis == 0) {
            intermediate.x += deltas[axis];
          } else if (axis == 1) {
            intermediate.y += deltas[axis];
          } else {
            intermediate.z += deltas[axis];
          }
        }
      }
      if (!map_.isInMap(intermediate) || !is_traversable(intermediate)) {
        return false;
      }
    }
    return true;
  };

  while (!open_set.empty()) {
    if (time_exceeded()) {
      return make_timeout();
    }
    if (options.max_expansions > 0 && result.expansions >= options.max_expansions) {
      return make_timeout();
    }

    const OpenEntry current_entry = open_set.top();
    open_set.pop();
    if (current_entry.g_score > g_scores[current_entry.flat_index] + kEpsilon ||
        closed[current_entry.flat_index] != 0U) {
      continue;
    }

    const std::size_t current_flat = current_entry.flat_index;
    const aurora::map::GridIndex current = unflatten(current_flat, dimensions);
    ++result.expansions;
    if (current_flat == goal_flat) {
      std::vector<aurora::map::GridIndex> index_path;
      for (std::size_t node = goal_flat; node != kInvalidNode; node = parents[node]) {
        index_path.push_back(unflatten(node, dimensions));
        if (node == start_flat) {
          break;
        }
      }
      if (index_path.empty() || flatten(index_path.back(), dimensions) != start_flat) {
        result.status = SearchStatus::NO_PATH;
        result.detail = "failed to reconstruct the A* parent chain";
        return result;
      }
      std::reverse(index_path.begin(), index_path.end());
      result.path.reserve(index_path.size() + 1);
      for (const auto &index : index_path) {
        result.path.push_back(map_.indexToWorld(index));
      }
      result.path.front() = start;
      if (result.path.size() == 1U) {
        if ((start - goal).norm() > 0.0) {
          result.path.push_back(goal);
        }
      } else {
        result.path.back() = goal;
      }
      result.status = SearchStatus::SUCCESS;
      result.cost = g_scores[current_flat];
      result.detail = "path found";
      return result;
    }
    closed[current_flat] = 1U;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const aurora::map::GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
          if (!map_.isInMap(neighbor) || !is_traversable(neighbor) ||
              !transition_is_clear(current, dx, dy, dz)) {
            continue;
          }

          const std::size_t neighbor_flat = flatten(neighbor, dimensions);
          const double tentative_g = g_scores[current_flat] + stepCost(dx, dy, dz, resolution);
          if (tentative_g + kEpsilon >= g_scores[neighbor_flat]) {
            continue;
          }

          if (!std::isfinite(g_scores[neighbor_flat])) {
            ++result.generated_nodes;
          }
          g_scores[neighbor_flat] = tentative_g;
          parents[neighbor_flat] = current_flat;
          closed[neighbor_flat] = 0U;
          const double f_score = tentative_g + heuristic(neighbor, goal_index, resolution);
          open_set.push({neighbor_flat, tentative_g, f_score, sequence++});
        }
      }
    }
    if (time_exceeded()) {
      return make_timeout();
    }
  }

  result.status = SearchStatus::NO_PATH;
  result.detail = "open set exhausted without reaching the goal";
  return result;
}

}  // namespace aurora::search
