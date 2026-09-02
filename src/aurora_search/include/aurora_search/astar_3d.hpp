#pragma once

#include "aurora_map/voxel_map.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace aurora::search {

struct SearchOptions {
  // Unknown voxels remain blocked by default. Callers may opt in when a
  // higher-level policy explicitly allows planning through unobserved space.
  bool allow_unknown{false};
  // Zero means no limit. The limit counts nodes inspected from the open set.
  std::size_t max_expansions{0};
  // Zero means no limit. The budget uses a monotonic clock.
  double max_compute_time_sec{0.0};
};

enum class SearchStatus {
  SUCCESS,
  INVALID_OPTIONS,
  INVALID_START,
  INVALID_GOAL,
  START_BLOCKED,
  GOAL_BLOCKED,
  SEARCH_TIMEOUT,
  NO_PATH,
};

const char *toString(SearchStatus status) noexcept;

struct SearchResult {
  SearchStatus status{SearchStatus::NO_PATH};
  std::vector<Eigen::Vector3d> path;
  double cost{std::numeric_limits<double>::infinity()};
  std::size_t expansions{0};
  std::size_t generated_nodes{0};
  aurora::map::GridIndex start_index{};
  aurora::map::GridIndex goal_index{};
  std::string detail;
};

// Static-map 3D A*. Dynamic obstacles and risk costs are intentionally kept out
// of this baseline API; they are added by later time-aware layers.
class AStar3D {
public:
  explicit AStar3D(const aurora::map::VoxelMap &map) noexcept : map_(map) {}

  SearchResult search(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                      const SearchOptions &options = {}) const;

private:
  const aurora::map::VoxelMap &map_;
};

}  // namespace aurora::search
