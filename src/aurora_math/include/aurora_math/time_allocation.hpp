#pragma once

#include <Eigen/Core>

#include <cstddef>

namespace aurora::math {

/**
 * Options for the deterministic, per-segment time allocation heuristic.
 *
 * The limits are used to form conservative rest-to-rest time scales from the
 * Euclidean distance between consecutive 3-D waypoints.  The result is a
 * useful initial time allocation, not a proof that a subsequently generated
 * trajectory satisfies all dynamic constraints.
 */
struct TimeAllocationOptions {
  double max_velocity{1.0};
  double max_acceleration{1.0};
  double max_jerk{1.0};
  double minimum_segment_time{0.01};
  double time_scale{1.0};
  std::size_t minimum_points{2U};
  double numerical_tolerance{1e-9};
};

using WaypointMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;

/**
 * Allocate one positive duration for every pair of consecutive 3-D points.
 *
 * For a non-zero segment of length d, the unscaled lower bound is the maximum
 * of d / max_velocity, 2 * sqrt(d / max_acceleration), and
 * cbrt(32 * d / max_jerk).  The latter two terms use a conservative
 * rest-to-rest envelope.  The selected bound is multiplied by time_scale and
 * clamped to minimum_segment_time.  Segments whose length is within
 * numerical_tolerance are assigned minimum_segment_time.
 *
 * @throws std::invalid_argument for invalid options, dimensions, or points.
 * @throws std::overflow_error when a valid finite input cannot produce a
 *         finite duration.
 */
Eigen::VectorXd allocateSegmentTimes(const WaypointMatrix &waypoints,
                                     const TimeAllocationOptions &options = {});

}  // namespace aurora::math
