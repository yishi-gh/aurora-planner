#include "aurora_map/voxel_map.hpp"
#include "aurora_math/path_resampler.hpp"
#include "aurora_math/uniform_bspline.hpp"
#include "aurora_search/astar_3d.hpp"
#include "aurora_trajectory/static_bspline_optimizer.hpp"

#include <gtest/gtest.h>


namespace {

using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;
using aurora::math::PathResamplingOptions;
using aurora::math::UniformBspline;
using aurora::search::AStar3D;
using aurora::search::SearchOptions;
using aurora::search::SearchStatus;
using aurora::trajectory::StaticBsplineOptimizer;
using aurora::trajectory::StaticOptimizerOptions;
using aurora::trajectory::StaticTrajectoryValidationOptions;

VoxelMap makeFreeMap() {
  VoxelMapConfig config;
  config.origin = Eigen::Vector3d(-10.0, -8.0, 0.0);
  config.dimensions = Eigen::Vector3i(40, 32, 16);
  config.resolution = 0.5;
  VoxelMap map(config);
  for (int x = 0; x < config.dimensions.x(); ++x) {
    for (int y = 0; y < config.dimensions.y(); ++y) {
      for (int z = 0; z < config.dimensions.z(); ++z) {
        map.setOccupancy({x, y, z}, 0.0);
      }
    }
  }
  return map;
}

}  // namespace

TEST(StaticPlannerPipeline, SearchesParameterizesOptimizesAndValidatesAroundWall) {
  VoxelMap map = makeFreeMap();
  map.addBox(Eigen::Vector3d(-0.7, -3.0, 0.0), Eigen::Vector3d(0.7, 3.0, 4.0));
  map.inflate(0.35);

  const Eigen::Vector3d start(-7.0, -5.0, 1.0);
  const Eigen::Vector3d goal(7.0, 5.0, 1.0);
  const AStar3D search(map);
  SearchOptions search_options;
  search_options.max_expansions = 100000;
  const auto search_result = search.search(start, goal, search_options);
  ASSERT_EQ(search_result.status, SearchStatus::SUCCESS);
  ASSERT_GE(search_result.path.size(), 2U);

  PathResamplingOptions resampling;
  resampling.spacing = 1.0;
  resampling.minimum_points = 31;
  const auto guide_points = aurora::math::resamplePath(search_result.path, resampling);
  ASSERT_GE(guide_points.size(), 31U);

  const double interval = 0.5;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const auto initial_control_points = UniformBspline::parameterizeToControlPoints(
      guide_points, interval, zero, zero, zero, zero);
  ASSERT_EQ(initial_control_points.cols(), static_cast<int>(guide_points.size()) + 2);

  StaticOptimizerOptions optimizer_options;
  optimizer_options.interval = interval;
  optimizer_options.clearance = 0.65;
  optimizer_options.max_velocity = 3.0;
  optimizer_options.max_acceleration = 6.0;
  optimizer_options.lambda_smooth = 0.08;
  optimizer_options.lambda_obstacle = 35.0;
  optimizer_options.lambda_feasibility = 0.15;
  optimizer_options.lambda_fitness = 0.01;
  optimizer_options.max_iterations = 180;
  optimizer_options.samples_per_span = 8;
  StaticBsplineOptimizer optimizer(map, initial_control_points, initial_control_points,
                                   optimizer_options);
  const auto initial_cost = optimizer.evaluate(initial_control_points, false);
  const auto initial_spline = UniformBspline(
      initial_control_points, interval, aurora::math::UniformBsplineKnotMode::EGO_UNCLAMPED);
  StaticTrajectoryValidationOptions validation_options;
  validation_options.samples_per_span = 16;
  validation_options.max_velocity = optimizer_options.max_velocity;
  validation_options.max_acceleration = optimizer_options.max_acceleration;
  const auto initial_validation =
      aurora::trajectory::validateStaticTrajectory(map, initial_spline, validation_options);

  const auto optimization_result = optimizer.optimize();
  const auto optimized_spline = UniformBspline(
      optimization_result.control_points, interval,
      aurora::math::UniformBsplineKnotMode::EGO_UNCLAMPED);
  const auto final_validation =
      aurora::trajectory::validateStaticTrajectory(map, optimized_spline, validation_options);

  EXPECT_LT(optimization_result.cost.total, initial_cost.total);
  EXPECT_EQ(optimization_result.control_points.rows(), 3);
  EXPECT_EQ(optimization_result.control_points.cols(), initial_control_points.cols());
  EXPECT_TRUE(optimization_result.control_points.allFinite());
  EXPECT_TRUE(initial_validation.valid || initial_validation.status ==
              aurora::trajectory::ValidationStatus::OCCUPIED ||
              initial_validation.status == aurora::trajectory::ValidationStatus::VELOCITY_LIMIT ||
              initial_validation.status == aurora::trajectory::ValidationStatus::ACCELERATION_LIMIT);
  EXPECT_TRUE(final_validation.valid) << final_validation.detail
                                      << " max_velocity=" << final_validation.maximum_velocity
                                      << " max_acceleration=" << final_validation.maximum_acceleration
                                      << " initial_max_velocity=" << initial_validation.maximum_velocity
                                      << " initial_max_acceleration=" << initial_validation.maximum_acceleration
                                      << " optimization_status="
                                      << aurora::trajectory::toString(optimization_result.status)
                                      << " iterations=" << optimization_result.iterations;
  EXPECT_EQ(final_validation.occupied_samples, 0U);
  EXPECT_EQ(final_validation.unknown_samples, 0U);
  EXPECT_NEAR(optimized_spline.evaluate(0.0)(0), start(0), 1e-8);
  EXPECT_NEAR(optimized_spline.evaluate(0.0)(1), start(1), 1e-8);
  EXPECT_NEAR(optimized_spline.evaluate(optimized_spline.duration())(0), goal(0), 1e-8);
  EXPECT_NEAR(optimized_spline.evaluate(optimized_spline.duration())(1), goal(1), 1e-8);
}
