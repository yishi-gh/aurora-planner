#include "aurora_search/astar_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aurora::map::GridIndex;
using aurora::map::MapState;
using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;
using aurora::search::AStar3D;
using aurora::search::SearchOptions;
using aurora::search::SearchStatus;

VoxelMapConfig makeConfig(const Eigen::Vector3i &dimensions = Eigen::Vector3i(9, 9, 5)) {
  VoxelMapConfig config;
  config.origin = Eigen::Vector3d::Zero();
  config.dimensions = dimensions;
  config.resolution = 1.0;
  return config;
}

VoxelMap makeFreeMap(const Eigen::Vector3i &dimensions = Eigen::Vector3i(9, 9, 5)) {
  VoxelMap map(makeConfig(dimensions));
  for (int x = 0; x < dimensions.x(); ++x) {
    for (int y = 0; y < dimensions.y(); ++y) {
      for (int z = 0; z < dimensions.z(); ++z) {
        map.setOccupancy({x, y, z}, 0.0);
      }
    }
  }
  return map;
}

Eigen::Vector3d center(const VoxelMap &map, const GridIndex &index) {
  return map.indexToWorld(index);
}

}  // namespace

TEST(AStar3D, Finds26NeighborPathWithMetricCost) {
  VoxelMap map = makeFreeMap();
  AStar3D astar(map);
  const auto result = astar.search(center(map, {1, 1, 1}), center(map, {2, 2, 2}));

  ASSERT_EQ(result.status, SearchStatus::SUCCESS);
  ASSERT_EQ(result.path.size(), 2U);
  EXPECT_NEAR(result.cost, std::sqrt(3.0), 1e-12);
  EXPECT_EQ(result.expansions, 2U);
}

TEST(AStar3D, RoutesAroundOccupiedWall) {
  VoxelMap map = makeFreeMap();
  for (int y = 2; y <= 6; ++y) {
    map.setOccupancy({4, y, 1}, 0.95);
  }

  AStar3D astar(map);
  const auto result = astar.search(center(map, {1, 4, 1}), center(map, {7, 4, 1}));

  ASSERT_EQ(result.status, SearchStatus::SUCCESS);
  ASSERT_GT(result.path.size(), 2U);
  for (const auto &point : result.path) {
    EXPECT_NE(map.query(point).state, MapState::OCCUPIED);
  }
}

TEST(AStar3D, DoesNotCutDiagonalCorners) {
  VoxelMap map = makeFreeMap();
  map.setOccupancy({2, 1, 1}, 0.95);
  map.setOccupancy({1, 2, 1}, 0.95);

  AStar3D astar(map);
  const auto result = astar.search(center(map, {1, 1, 1}), center(map, {2, 2, 1}));

  ASSERT_EQ(result.status, SearchStatus::SUCCESS);
  EXPECT_GT(result.cost, std::sqrt(2.0));
  ASSERT_GE(result.path.size(), 3U);
}

TEST(AStar3D, InflationIsRespectedAsOccupied) {
  VoxelMap map = makeFreeMap();
  map.addBox(Eigen::Vector3d(4.0, 4.0, 1.0), Eigen::Vector3d(5.0, 5.0, 2.0));
  map.inflate(1.1);

  AStar3D astar(map);
  const auto result = astar.search(center(map, {1, 4, 1}), center(map, {7, 4, 1}));

  ASSERT_EQ(result.status, SearchStatus::SUCCESS);
  ASSERT_GT(result.path.size(), 2U);
  for (const auto &point : result.path) {
    EXPECT_NE(map.query(point).state, MapState::OCCUPIED);
  }
}

TEST(AStar3D, UnknownVoxelIsNotTraversable) {
  VoxelMap map = makeFreeMap();
  map.setUnknown({4, 4, 1});

  AStar3D astar(map);
  const auto result = astar.search(center(map, {1, 4, 1}), center(map, {7, 4, 1}));

  ASSERT_EQ(result.status, SearchStatus::SUCCESS);
  for (const auto &point : result.path) {
    EXPECT_NE(map.query(point).state, MapState::UNKNOWN);
  }
}

TEST(AStar3D, AllowsUnknownWhenExplicitlyConfigured) {
  VoxelMap map = makeFreeMap();
  map.setUnknown({1, 4, 1});

  AStar3D astar(map);
  SearchOptions options;
  options.allow_unknown = true;
  const auto result = astar.search(center(map, {1, 4, 1}), center(map, {7, 4, 1}), options);

  EXPECT_EQ(result.status, SearchStatus::SUCCESS);
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.front(), center(map, {1, 4, 1}));
}

TEST(AStar3D, ReportsBlockedEndpointsAndOutOfMapInputs) {
  VoxelMap map = makeFreeMap();
  AStar3D astar(map);

  map.setOccupancy({1, 1, 1}, 0.95);
  EXPECT_EQ(astar.search(center(map, {1, 1, 1}), center(map, {2, 2, 1})).status,
            SearchStatus::START_BLOCKED);

  map.setOccupancy({2, 2, 1}, 0.95);
  EXPECT_EQ(astar.search(center(map, {1, 2, 1}), center(map, {2, 2, 1})).status,
            SearchStatus::GOAL_BLOCKED);

  EXPECT_EQ(astar.search(Eigen::Vector3d(-0.01, 1.5, 1.5), center(map, {2, 2, 1})).status,
            SearchStatus::INVALID_START);
  EXPECT_EQ(astar.search(center(map, {1, 2, 1}), Eigen::Vector3d(9.0, 2.5, 1.5)).status,
            SearchStatus::INVALID_GOAL);
}

TEST(AStar3D, ReportsNoPathWhenWallSpansTheMap) {
  VoxelMap map = makeFreeMap(Eigen::Vector3i(7, 7, 4));
  for (int y = 0; y < map.dimensions().y(); ++y) {
    for (int z = 0; z < map.dimensions().z(); ++z) {
      map.setOccupancy({3, y, z}, 0.95);
    }
  }

  AStar3D astar(map);
  const auto result = astar.search(center(map, {1, 3, 1}), center(map, {5, 3, 1}));

  EXPECT_EQ(result.status, SearchStatus::NO_PATH);
  EXPECT_TRUE(result.path.empty());
}

TEST(AStar3D, ExpansionBudgetIsReportedAsTimeout) {
  VoxelMap map = makeFreeMap();
  AStar3D astar(map);
  SearchOptions options;
  options.max_expansions = 1;

  const auto result = astar.search(center(map, {1, 1, 1}), center(map, {7, 7, 3}), options);

  EXPECT_EQ(result.status, SearchStatus::SEARCH_TIMEOUT);
  EXPECT_EQ(result.expansions, 1U);
}

TEST(AStar3D, ComputeTimeBudgetIsReportedAsTimeout) {
  VoxelMap map = makeFreeMap();
  AStar3D astar(map);
  SearchOptions options;
  options.max_compute_time_sec = 1e-12;

  const auto result = astar.search(center(map, {1, 1, 1}), center(map, {7, 7, 3}), options);

  EXPECT_EQ(result.status, SearchStatus::SEARCH_TIMEOUT);
}

TEST(AStar3D, PreservesExactWorldEndpoints) {
  VoxelMap map = makeFreeMap();
  AStar3D astar(map);
  const Eigen::Vector3d start(0.1, 0.2, 0.3);
  const Eigen::Vector3d goal(7.7, 6.6, 3.4);

  const auto result = astar.search(start, goal);

  ASSERT_EQ(result.status, SearchStatus::SUCCESS);
  ASSERT_GE(result.path.size(), 2U);
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);
}

TEST(AStar3D, RejectsInvalidTimeBudget) {
  VoxelMap map = makeFreeMap();
  AStar3D astar(map);
  SearchOptions options;
  options.max_compute_time_sec = -1.0;

  const auto result = astar.search(center(map, {1, 1, 1}), center(map, {2, 2, 1}), options);

  EXPECT_EQ(result.status, SearchStatus::INVALID_OPTIONS);
}
