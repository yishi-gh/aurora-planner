#include "aurora_trajectory/static_bspline_optimizer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using aurora::map::GridIndex;
using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;
using aurora::math::UniformBspline;
using aurora::trajectory::ControlPointMatrix;
using aurora::trajectory::StaticBsplineOptimizer;
using aurora::trajectory::StaticOptimizerOptions;
using aurora::trajectory::StaticTrajectoryValidationOptions;
using aurora::trajectory::ValidationStatus;

VoxelMap makeFreeMap() {
  VoxelMapConfig config;
  config.origin = Eigen::Vector3d(-5.0, -5.0, 0.0);
  config.dimensions = Eigen::Vector3i(40, 40, 16);
  config.resolution = 0.25;
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

ControlPointMatrix makeStraightControlPoints(int count = 9) {
  ControlPointMatrix points(3, count);
  for (int i = 0; i < count; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(count - 1);
    points.col(i) = Eigen::Vector3d(-2.0 + 4.0 * alpha, 0.0, 1.0);
  }
  return points;
}

void expectMatrixColumnNear(const ControlPointMatrix &actual, const ControlPointMatrix &expected,
                            int column, double tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(actual(axis, column), expected(axis, column), tolerance);
  }
}

}  // namespace

TEST(StaticBsplineOptimizer, RejectsInvalidInputs) {
  VoxelMap map = makeFreeMap();
  ControlPointMatrix points = makeStraightControlPoints();
  ControlPointMatrix too_short(3, 6);
  too_short.setZero();

  EXPECT_THROW(StaticBsplineOptimizer(map, too_short, too_short), std::invalid_argument);
  EXPECT_THROW(StaticBsplineOptimizer(map, points, ControlPointMatrix(3, 8)), std::invalid_argument);

  StaticOptimizerOptions invalid_options;
  invalid_options.interval = 0.0;
  EXPECT_THROW(StaticBsplineOptimizer(map, points, points, invalid_options), std::invalid_argument);
}

TEST(StaticBsplineOptimizer, GradientMatchesFiniteDifferenceInFreeSpace) {
  VoxelMap map = makeFreeMap();
  ControlPointMatrix points = makeStraightControlPoints();
  points(1, 4) = 0.35;

  StaticOptimizerOptions options;
  options.interval = 0.5;
  options.lambda_obstacle = 0.0;
  options.max_velocity = 3.0;
  options.max_acceleration = 6.0;
  StaticBsplineOptimizer optimizer(map, points, makeStraightControlPoints(), options);
  const auto analytical = optimizer.evaluate(points, true);

  constexpr double epsilon = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    ControlPointMatrix plus = points;
    ControlPointMatrix minus = points;
    plus(axis, 4) += epsilon;
    minus(axis, 4) -= epsilon;
    const double numerical =
        (optimizer.evaluate(plus, false).total - optimizer.evaluate(minus, false).total) /
        (2.0 * epsilon);
    EXPECT_NEAR(analytical.gradient(axis, 4), numerical, 2e-5);
  }
}

TEST(StaticBsplineOptimizer, OptimizationLowersCostAndKeepsBoundaryControlPoints) {
  VoxelMap map = makeFreeMap();
  map.addBox(Eigen::Vector3d(-0.4, -0.8, 0.0), Eigen::Vector3d(0.4, 0.8, 2.0));
  ControlPointMatrix initial = makeStraightControlPoints();
  ControlPointMatrix reference = initial;

  StaticOptimizerOptions options;
  options.interval = 0.5;
  options.clearance = 0.8;
  options.lambda_smooth = 0.08;
  options.lambda_obstacle = 35.0;
  options.lambda_fitness = 0.01;
  options.max_iterations = 80;
  StaticBsplineOptimizer optimizer(map, initial, reference, options);
  const auto initial_cost = optimizer.evaluate(initial, false);
  const auto result = optimizer.optimize();

  EXPECT_LT(result.cost.total, initial_cost.total);
  for (int i = 0; i < 3; ++i) {
    expectMatrixColumnNear(result.control_points, initial, i, 1e-12);
    expectMatrixColumnNear(result.control_points, initial, result.control_points.cols() - 1 - i,
                           1e-12);
  }
  EXPECT_TRUE(result.control_points.allFinite());
  EXPECT_TRUE(result.status == aurora::trajectory::OptimizationStatus::CONVERGED ||
              result.status == aurora::trajectory::OptimizationStatus::MAX_ITERATIONS ||
              result.status == aurora::trajectory::OptimizationStatus::STALLED ||
              result.status == aurora::trajectory::OptimizationStatus::TIMEOUT);
}

TEST(StaticBsplineOptimizer, ReportsWallClockBudgetTimeout) {
  VoxelMap map = makeFreeMap();
  const ControlPointMatrix initial = makeStraightControlPoints();

  StaticOptimizerOptions options;
  options.interval = 0.5;
  options.max_compute_time_sec = 1e-12;
  StaticBsplineOptimizer optimizer(map, initial, initial, options);
  const auto result = optimizer.optimize();

  EXPECT_EQ(result.status, aurora::trajectory::OptimizationStatus::TIMEOUT);
  EXPECT_TRUE(result.control_points.allFinite());
  EXPECT_TRUE(result.cost.gradient.allFinite());
}

TEST(StaticBsplineOptimizer, AppliesRiskCostAndKeepsRiskTimeAbsolute) {
  VoxelMap map = makeFreeMap();
  const ControlPointMatrix initial = makeStraightControlPoints();

  StaticOptimizerOptions options;
  options.interval = 0.5;
  options.lambda_smooth = 0.0;
  options.lambda_obstacle = 0.0;
  options.lambda_feasibility = 0.0;
  options.lambda_fitness = 0.0;
  options.lambda_risk = 4.0;
  options.risk_time_origin = 10.0;
  options.max_iterations = 30;
  options.initial_step = 0.1;
  options.gradient_clip = 10.0;

  std::vector<double> observed_stamps;
  options.risk_cost = [&observed_stamps](double stamp, const Eigen::Vector3d &position) {
    observed_stamps.push_back(stamp);
    aurora::trajectory::RiskCostEvaluation result;
    const double error = position.y() - 1.0;
    result.value = error * error;
    result.gradient = Eigen::Vector3d(0.0, 2.0 * error, 0.0);
    return result;
  };

  StaticBsplineOptimizer optimizer(map, initial, initial, options);
  const auto initial_cost = optimizer.evaluate(initial, false);
  const auto result = optimizer.optimize();

  EXPECT_GT(initial_cost.risk, 0.0);
  EXPECT_LT(result.cost.risk, initial_cost.risk);
  EXPECT_TRUE(result.risk_enabled);
  EXPECT_FALSE(result.risk_fallback);
  ASSERT_FALSE(observed_stamps.empty());
  EXPECT_DOUBLE_EQ(observed_stamps.front(), options.risk_time_origin);
  EXPECT_TRUE(result.control_points.allFinite());
  EXPECT_GT(result.control_points.row(1).segment(3, 3).mean(), 0.0);
  for (int axis = 0; axis < 3; ++axis) {
    expectMatrixColumnNear(result.control_points, initial, axis, 1e-12);
    expectMatrixColumnNear(result.control_points, initial,
                           result.control_points.cols() - 1 - axis, 1e-12);
  }
}

TEST(StaticBsplineOptimizer, FallsBackWhenRiskCallbackIsUnavailableOrOverBudget) {
  VoxelMap map = makeFreeMap();
  const ControlPointMatrix initial = makeStraightControlPoints();

  StaticOptimizerOptions unavailable_options;
  unavailable_options.interval = 0.5;
  unavailable_options.lambda_obstacle = 0.0;
  unavailable_options.lambda_risk = 1.0;
  unavailable_options.risk_cost = [](double, const Eigen::Vector3d &) {
    aurora::trajectory::RiskCostEvaluation result;
    result.valid = false;
    result.detail = "test risk source unavailable";
    return result;
  };
  StaticBsplineOptimizer unavailable(map, initial, initial, unavailable_options);
  const auto unavailable_result = unavailable.optimize();
  EXPECT_TRUE(unavailable_result.risk_fallback);
  EXPECT_FALSE(unavailable_result.risk_enabled);
  EXPECT_TRUE(unavailable_result.cost.risk_available);
  EXPECT_DOUBLE_EQ(unavailable_result.cost.risk, 0.0);

  StaticOptimizerOptions budget_options = unavailable_options;
  budget_options.risk_cost = [](double, const Eigen::Vector3d &) {
    aurora::trajectory::RiskCostEvaluation result;
    result.value = 1.0;
    result.gradient.setZero();
    return result;
  };
  budget_options.max_risk_evaluations = 1U;
  StaticBsplineOptimizer over_budget(map, initial, initial, budget_options);
  const auto over_budget_result = over_budget.optimize();
  EXPECT_TRUE(over_budget_result.risk_fallback);
  EXPECT_FALSE(over_budget_result.risk_enabled);
  EXPECT_DOUBLE_EQ(over_budget_result.cost.risk, 0.0);

  StaticOptimizerOptions throwing_options = unavailable_options;
  throwing_options.risk_cost = [](double, const Eigen::Vector3d &) ->
      aurora::trajectory::RiskCostEvaluation {
    throw std::runtime_error("synthetic risk failure");
  };
  StaticBsplineOptimizer throwing(map, initial, initial, throwing_options);
  const auto throwing_result = throwing.optimize();
  EXPECT_TRUE(throwing_result.risk_fallback);
  EXPECT_FALSE(throwing_result.risk_enabled);
  EXPECT_DOUBLE_EQ(throwing_result.cost.risk, 0.0);
}

TEST(StaticTrajectoryValidation, ReportsOccupancyUnknownAndValidStates) {
  VoxelMap map = makeFreeMap();
  const ControlPointMatrix points = makeStraightControlPoints();
  const UniformBspline spline(points, 0.5);
  StaticTrajectoryValidationOptions options;
  options.samples_per_span = 16;
  options.max_velocity = 3.0;
  options.max_acceleration = 6.0;

  const auto valid = aurora::trajectory::validateStaticTrajectory(map, spline, options);
  EXPECT_TRUE(valid.valid);
  EXPECT_EQ(valid.status, ValidationStatus::VALID);
  EXPECT_GT(valid.checked_samples, 0U);

  map.addBox(Eigen::Vector3d(-0.4, -0.8, 0.0), Eigen::Vector3d(0.4, 0.8, 2.0));
  const auto occupied = aurora::trajectory::validateStaticTrajectory(map, spline, options);
  EXPECT_FALSE(occupied.valid);
  EXPECT_EQ(occupied.status, ValidationStatus::OCCUPIED);
  EXPECT_GT(occupied.occupied_samples, 0U);

  VoxelMap unknown_map = makeFreeMap();
  unknown_map.setUnknown(unknown_map.worldToIndex(Eigen::Vector3d(0.0, 0.0, 1.0)));
  const auto unknown = aurora::trajectory::validateStaticTrajectory(unknown_map, spline, options);
  EXPECT_FALSE(unknown.valid);
  EXPECT_EQ(unknown.status, ValidationStatus::UNKNOWN);
  EXPECT_GT(unknown.unknown_samples, 0U);
}

TEST(StaticTrajectoryValidation, ReportsDynamicConstraintViolationAndInvalidOptions) {
  VoxelMap map = makeFreeMap();
  ControlPointMatrix points = makeStraightControlPoints();
  for (int i = 0; i < points.cols(); ++i) {
    points(0, i) *= 2.0;
  }
  const UniformBspline spline(points, 0.1);

  StaticTrajectoryValidationOptions options;
  options.max_velocity = 1.0;
  options.max_acceleration = 1.0;
  const auto invalid_dynamics = aurora::trajectory::validateStaticTrajectory(map, spline, options);
  EXPECT_FALSE(invalid_dynamics.valid);
  EXPECT_TRUE(invalid_dynamics.status == ValidationStatus::VELOCITY_LIMIT ||
              invalid_dynamics.status == ValidationStatus::ACCELERATION_LIMIT);

  options.samples_per_span = 0;
  const auto invalid_options = aurora::trajectory::validateStaticTrajectory(map, spline, options);
  EXPECT_FALSE(invalid_options.valid);
  EXPECT_EQ(invalid_options.status, ValidationStatus::INVALID_OPTIONS);
}
