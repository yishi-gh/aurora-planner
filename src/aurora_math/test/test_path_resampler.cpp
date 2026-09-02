#include "aurora_math/path_resampler.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using aurora::math::PathResamplingOptions;
using aurora::math::resamplePath;

void expectVectorNear(const Eigen::Vector3d &actual, const Eigen::Vector3d &expected,
                      double tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(actual(axis), expected(axis), tolerance);
  }
}

}  // namespace

TEST(PathResampler, RejectsInvalidInputs) {
  EXPECT_THROW(resamplePath({}), std::invalid_argument);

  PathResamplingOptions options;
  options.spacing = 0.0;
  EXPECT_THROW(resamplePath({Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()}, options),
               std::invalid_argument);

  options = {};
  options.minimum_points = 1;
  EXPECT_THROW(resamplePath({Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()}, options),
               std::invalid_argument);

  options = {};
  options.duplicate_epsilon = -1.0;
  EXPECT_THROW(resamplePath({Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()}, options),
               std::invalid_argument);

  Eigen::Vector3d invalid = Eigen::Vector3d::Zero();
  invalid(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(resamplePath({Eigen::Vector3d::Zero(), invalid}), std::invalid_argument);
}

TEST(PathResampler, ResamplesPolylineByArcLength) {
  PathResamplingOptions options;
  options.spacing = 0.5;
  options.minimum_points = 2;
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0)};

  const auto result = resamplePath(path, options);

  ASSERT_EQ(result.size(), 5U);
  expectVectorNear(result[0], Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12);
  expectVectorNear(result[1], Eigen::Vector3d(0.5, 0.0, 0.0), 1e-12);
  expectVectorNear(result[2], Eigen::Vector3d(1.0, 0.0, 0.0), 1e-12);
  expectVectorNear(result[3], Eigen::Vector3d(1.0, 0.5, 0.0), 1e-12);
  expectVectorNear(result[4], Eigen::Vector3d(1.0, 1.0, 0.0), 1e-12);
}

TEST(PathResampler, SubdividesShortPathToMinimumPointCount) {
  const Eigen::Vector3d start(0.1, 0.2, 0.3);
  const Eigen::Vector3d end(1.1, 0.2, 0.3);
  const auto result = resamplePath({start, end});

  ASSERT_EQ(result.size(), 7U);
  EXPECT_EQ(result.front(), start);
  EXPECT_EQ(result.back(), end);
  for (std::size_t index = 1; index < result.size(); ++index) {
    EXPECT_NEAR((result[index] - result[index - 1]).norm(), 1.0 / 6.0, 1e-12);
  }
}

TEST(PathResampler, RemovesConsecutiveDuplicatesAndHandlesSinglePoint) {
  PathResamplingOptions options;
  options.spacing = 0.25;
  options.minimum_points = 4;
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d near_duplicate(1e-10, 0.0, 0.0);
  const Eigen::Vector3d end(1.0, 0.0, 0.0);
  const auto result = resamplePath({start, near_duplicate, near_duplicate, end}, options);
  EXPECT_EQ(result.size(), 5U);
  EXPECT_EQ(result.front(), start);
  EXPECT_EQ(result.back(), end);

  const Eigen::Vector3d single(2.0, -1.0, 0.5);
  const auto stationary = resamplePath({single});
  ASSERT_EQ(stationary.size(), 7U);
  for (const auto &point : stationary) {
    EXPECT_EQ(point, single);
  }
}
