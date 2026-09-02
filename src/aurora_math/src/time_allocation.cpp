#include "aurora_math/time_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <stdexcept>

namespace aurora::math {
namespace {

bool isFinite(double value) { return std::isfinite(value); }

void validateOptions(const TimeAllocationOptions &options) {
  if (!isFinite(options.max_velocity) || options.max_velocity <= 0.0) {
    throw std::invalid_argument("time allocation max_velocity must be finite and positive");
  }
  if (!isFinite(options.max_acceleration) || options.max_acceleration <= 0.0) {
    throw std::invalid_argument("time allocation max_acceleration must be finite and positive");
  }
  if (!isFinite(options.max_jerk) || options.max_jerk <= 0.0) {
    throw std::invalid_argument("time allocation max_jerk must be finite and positive");
  }
  if (!isFinite(options.minimum_segment_time) || options.minimum_segment_time <= 0.0) {
    throw std::invalid_argument(
        "time allocation minimum_segment_time must be finite and positive");
  }
  if (!isFinite(options.time_scale) || options.time_scale <= 0.0) {
    throw std::invalid_argument("time allocation time_scale must be finite and positive");
  }
  if (options.minimum_points < 2U) {
    throw std::invalid_argument("time allocation minimum_points must be at least two");
  }
  if (!isFinite(options.numerical_tolerance) || options.numerical_tolerance < 0.0) {
    throw std::invalid_argument(
        "time allocation numerical_tolerance must be finite and non-negative");
  }
}

double finiteQuotient(double numerator, double denominator, const char *name) {
  const double result = numerator / denominator;
  if (!isFinite(result)) {
    throw std::overflow_error(std::string("time allocation ") + name +
                              " lower bound is not finite");
  }
  return result;
}

double finiteScaled(double value, double scale) {
  const double result = value * scale;
  if (!isFinite(result)) {
    throw std::overflow_error("time allocation scaled duration is not finite");
  }
  return result;
}

double segmentDistance(const WaypointMatrix &waypoints, Eigen::Index segment) {
  const Eigen::Vector3d delta = waypoints.col(segment + 1) - waypoints.col(segment);
  if (!delta.allFinite()) {
    throw std::overflow_error("time allocation waypoint difference is not finite");
  }

  // hypot avoids the avoidable squared-norm overflow for large finite inputs.
  const double xy_norm = std::hypot(delta.x(), delta.y());
  const double distance = std::hypot(xy_norm, delta.z());
  if (!isFinite(distance)) {
    throw std::overflow_error("time allocation segment distance is not finite");
  }
  return distance;
}

}  // namespace

Eigen::VectorXd allocateSegmentTimes(const WaypointMatrix &waypoints,
                                     const TimeAllocationOptions &options) {
  validateOptions(options);

  if (waypoints.cols() < 2 ||
      static_cast<std::size_t>(waypoints.cols()) < options.minimum_points) {
    throw std::invalid_argument(
        "time allocation requires at least minimum_points 3-D waypoints");
  }
  if (!waypoints.allFinite()) {
    throw std::invalid_argument("time allocation waypoints must be finite");
  }

  Eigen::VectorXd segment_times(waypoints.cols() - 1);
  for (Eigen::Index segment = 0; segment < segment_times.size(); ++segment) {
    const double distance = segmentDistance(waypoints, segment);
    if (distance <= options.numerical_tolerance) {
      segment_times(segment) = options.minimum_segment_time;
      continue;
    }

    const double velocity_bound =
        finiteQuotient(distance, options.max_velocity, "velocity");
    const double acceleration_ratio =
        finiteQuotient(distance, options.max_acceleration, "acceleration");
    const double acceleration_bound = 2.0 * std::sqrt(acceleration_ratio);
    if (!isFinite(acceleration_bound)) {
      throw std::overflow_error("time allocation acceleration lower bound is not finite");
    }

    const double jerk_ratio = finiteQuotient(distance, options.max_jerk, "jerk");
    const double jerk_argument = 32.0 * jerk_ratio;
    if (!isFinite(jerk_argument)) {
      throw std::overflow_error("time allocation jerk lower bound is not finite");
    }
    const double jerk_bound = std::cbrt(jerk_argument);
    if (!isFinite(jerk_bound)) {
      throw std::overflow_error("time allocation jerk lower bound is not finite");
    }

    const double lower_bound = std::max({velocity_bound, acceleration_bound, jerk_bound});
    const double scaled_bound = finiteScaled(lower_bound, options.time_scale);
    segment_times(segment) = std::max(options.minimum_segment_time, scaled_bound);
    if (!isFinite(segment_times(segment)) || segment_times(segment) <= 0.0) {
      throw std::overflow_error("time allocation produced an invalid segment duration");
    }
  }

  return segment_times;
}

}  // namespace aurora::math
