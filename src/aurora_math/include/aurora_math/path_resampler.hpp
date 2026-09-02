#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace aurora::math {

struct PathResamplingOptions {
  double spacing{0.5};
  std::size_t minimum_points{7};
  double duplicate_epsilon{1e-9};
};

// Resamples a finite 3D polyline by arc length. Consecutive points closer than
// duplicate_epsilon are removed, while the exact input endpoints are retained.
// Short paths are uniformly subdivided to contain at least minimum_points.
std::vector<Eigen::Vector3d> resamplePath(
    const std::vector<Eigen::Vector3d> &path,
    const PathResamplingOptions &options = {});

}  // namespace aurora::math
