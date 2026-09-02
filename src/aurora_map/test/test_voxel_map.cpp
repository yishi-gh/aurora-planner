#include "aurora_map/voxel_map.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aurora::map::GridIndex;
using aurora::map::MapState;
using aurora::map::RayObservation;
using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;

VoxelMapConfig makeConfig() {
  VoxelMapConfig config;
  config.origin = Eigen::Vector3d(-2.0, -2.0, -1.0);
  config.dimensions = Eigen::Vector3i(8, 8, 6);
  config.resolution = 0.5;
  config.occupancy_threshold = 0.8;
  return config;
}

}  // namespace

TEST(VoxelMap, RejectsInvalidConfigurationAndMeasurements) {
  VoxelMapConfig config = makeConfig();
  config.resolution = 0.0;
  EXPECT_THROW({ VoxelMap map(config); }, std::invalid_argument);

  config = makeConfig();
  config.dimensions.x() = 0;
  EXPECT_THROW({ VoxelMap map(config); }, std::invalid_argument);

  VoxelMap map(makeConfig());
  EXPECT_THROW(map.setOccupancy({0, 0, 0}, 1.1), std::invalid_argument);
  EXPECT_THROW(map.setOccupancy({0, 0, 0}, 0.5, -0.1), std::invalid_argument);
  EXPECT_THROW(map.setOccupancy({0, 0, 0}, 0.5, 0.0, 1.1), std::invalid_argument);
  EXPECT_THROW(map.setOccupancy({-1, 0, 0}, 0.5), std::out_of_range);
  EXPECT_THROW(map.inflate(-0.01), std::invalid_argument);
}

TEST(VoxelMap, CoordinatesAndInitialUnknownState) {
  VoxelMap map(makeConfig());
  const Eigen::Vector3d center = map.indexToWorld({2, 3, 1});
  EXPECT_EQ(map.worldToIndex(center), (GridIndex{2, 3, 1}));
  EXPECT_TRUE(map.isInMap(center));
  EXPECT_FALSE(map.isInMap(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_FALSE(map.isInMap(GridIndex{-1, 0, 0}));

  const auto unknown = map.query(center);
  EXPECT_EQ(unknown.state, MapState::UNKNOWN);
  EXPECT_DOUBLE_EQ(unknown.occupancy_probability, 0.5);
  EXPECT_FALSE(unknown.inflated);
  EXPECT_TRUE(std::isinf(unknown.observation_age));
  EXPECT_DOUBLE_EQ(unknown.confidence, 0.0);
  EXPECT_EQ(unknown.map_version, 0U);

  const auto out = map.query(Eigen::Vector3d(10.0, 0.0, 0.0));
  EXPECT_EQ(out.state, MapState::OUT_OF_MAP);
  EXPECT_TRUE(out.inflated);
  EXPECT_DOUBLE_EQ(out.occupancy_probability, 1.0);
}

TEST(VoxelMap, StoresFreeAndOccupiedMeasurements) {
  VoxelMap map(makeConfig());
  const GridIndex free_index{1, 1, 1};
  const GridIndex occupied_index{2, 1, 1};
  map.setOccupancy(free_index, 0.1, 0.25, 0.75);
  map.setOccupancy(occupied_index, 0.95, 0.1, 0.9);

  const auto free = map.query(free_index);
  EXPECT_EQ(free.state, MapState::FREE);
  EXPECT_DOUBLE_EQ(free.occupancy_probability, 0.1);
  EXPECT_DOUBLE_EQ(free.observation_age, 0.25);
  EXPECT_DOUBLE_EQ(free.confidence, 0.75);

  const auto occupied = map.query(occupied_index);
  EXPECT_EQ(occupied.state, MapState::OCCUPIED);
  EXPECT_DOUBLE_EQ(occupied.occupancy_probability, 0.95);
  EXPECT_EQ(occupied.map_version, map.version());
  EXPECT_GT(map.version(), 0U);

  map.setUnknown(occupied_index);
  EXPECT_EQ(map.query(occupied_index).state, MapState::UNKNOWN);
}

TEST(VoxelMap, AddsAndInflatesBox) {
  VoxelMap map(makeConfig());
  map.addBox(Eigen::Vector3d(-0.5, -0.5, 0.0), Eigen::Vector3d(0.5, 0.5, 1.0));
  const GridIndex occupied_index = map.worldToIndex(Eigen::Vector3d(-0.25, -0.25, 0.25));
  EXPECT_EQ(map.query(occupied_index).state, MapState::OCCUPIED);

  const GridIndex adjacent_index = map.worldToIndex(Eigen::Vector3d(0.75, -0.25, 0.25));
  EXPECT_EQ(map.query(adjacent_index).state, MapState::UNKNOWN);

  map.inflate(0.5);
  EXPECT_TRUE(map.hasInflation());
  EXPECT_DOUBLE_EQ(map.inflationRadius(), 0.5);
  const auto inflated = map.query(adjacent_index);
  EXPECT_EQ(inflated.state, MapState::OCCUPIED);
  EXPECT_TRUE(inflated.inflated);

  const GridIndex diagonal_index = map.worldToIndex(Eigen::Vector3d(0.75, 0.75, 0.75));
  EXPECT_EQ(map.query(diagonal_index).state, MapState::UNKNOWN);
}

TEST(VoxelMap, MutationInvalidatesInflationAndBoxClips) {
  VoxelMap map(makeConfig());
  map.addBox(Eigen::Vector3d(-20.0, -20.0, -20.0), Eigen::Vector3d(-1.0, -1.0, 0.0));
  EXPECT_EQ(map.query(GridIndex{0, 0, 0}).state, MapState::OCCUPIED);
  map.inflate(1.0);
  EXPECT_TRUE(map.hasInflation());

  const GridIndex free_index{3, 3, 3};
  map.setOccupancy(free_index, 0.0);
  EXPECT_FALSE(map.hasInflation());
  EXPECT_FALSE(map.query(free_index).inflated);
}

TEST(VoxelMap, IntegratesHitRayWithLogOdds) {
  VoxelMap map(makeConfig());
  RayObservation observation;
  observation.origin = Eigen::Vector3d(-1.75, -1.75, -0.75);
  observation.endpoint = Eigen::Vector3d(0.75, -1.75, -0.75);
  observation.hit = true;
  observation.observation_age = 0.2;
  observation.confidence = 0.8;

  const auto stats = map.integrateRay(observation);
  EXPECT_EQ(stats.traversed_voxels, 6U);
  EXPECT_EQ(stats.miss_updates, 5U);
  EXPECT_TRUE(stats.endpoint_marked);
  EXPECT_EQ(map.query(GridIndex{0, 0, 0}).state, MapState::FREE);
  EXPECT_DOUBLE_EQ(map.query(GridIndex{0, 0, 0}).occupancy_probability, 0.35);
  EXPECT_EQ(map.query(GridIndex{5, 0, 0}).state, MapState::FREE);
  EXPECT_DOUBLE_EQ(map.query(GridIndex{5, 0, 0}).occupancy_probability, 0.65);
  EXPECT_DOUBLE_EQ(map.query(GridIndex{5, 0, 0}).observation_age, 0.2);
  EXPECT_DOUBLE_EQ(map.query(GridIndex{5, 0, 0}).confidence, 0.8);

  map.integrateRay(observation);
  EXPECT_GT(map.query(GridIndex{5, 0, 0}).occupancy_probability, 0.65);
  map.integrateRay(observation);
  EXPECT_EQ(map.query(GridIndex{5, 0, 0}).state, MapState::OCCUPIED);
  for (int i = 0; i < 100; ++i) {
    map.integrateRay(observation);
  }
  EXPECT_LE(map.query(GridIndex{5, 0, 0}).occupancy_probability, 0.90 + 1e-12);
}

TEST(VoxelMap, IntegratesMissRayWithRangeLimitAndClipping) {
  VoxelMap map(makeConfig());
  RayObservation miss;
  miss.origin = Eigen::Vector3d(-4.0, -1.25, -0.75);
  miss.endpoint = Eigen::Vector3d(1.75, -1.25, -0.75);
  miss.hit = false;
  miss.max_range = 3.0;
  const auto stats = map.integrateRay(miss);
  EXPECT_GT(stats.traversed_voxels, 0U);
  EXPECT_EQ(stats.miss_updates, stats.traversed_voxels);
  EXPECT_FALSE(stats.endpoint_marked);
  EXPECT_EQ(map.query(GridIndex{0, 1, 0}).state, MapState::FREE);
  EXPECT_EQ(map.query(GridIndex{3, 1, 0}).state, MapState::UNKNOWN);
}

TEST(VoxelMap, HandlesZeroLengthRaysAndLogOddsClamping) {
  VoxelMap map(makeConfig());
  RayObservation point;
  point.origin = Eigen::Vector3d(-1.75, -1.75, -0.75);
  point.endpoint = point.origin;
  point.hit = true;
  EXPECT_EQ(map.integrateRay(point).traversed_voxels, 1U);
  EXPECT_EQ(map.query(GridIndex{0, 0, 0}).state, MapState::FREE);
  map.integrateRay(point);
  map.integrateRay(point);
  EXPECT_EQ(map.query(GridIndex{0, 0, 0}).state, MapState::OCCUPIED);

  point.hit = false;
  for (int i = 0; i < 100; ++i) {
    map.integrateRay(point);
  }
  EXPECT_GE(map.query(GridIndex{0, 0, 0}).occupancy_probability, 0.12 - 1e-12);
}

TEST(VoxelMap, RejectsInvalidRayObservations) {
  VoxelMap map(makeConfig());
  RayObservation observation;
  observation.max_range = -1.0;
  EXPECT_THROW(map.integrateRay(observation), std::invalid_argument);
  observation = RayObservation{};
  observation.origin.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(map.integrateRay(observation), std::invalid_argument);
  observation = RayObservation{};
  observation.confidence = 2.0;
  EXPECT_THROW(map.integrateRay(observation), std::invalid_argument);
}
