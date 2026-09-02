#include "aurora_math/path_resampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aurora::math {
namespace {

bool isFinite(double value) { return std::isfinite(value); }

std::vector<Eigen::Vector3d> removeConsecutiveDuplicates(
    const std::vector<Eigen::Vector3d> &path, double duplicate_epsilon) {
  std::vector<Eigen::Vector3d> cleaned;
  cleaned.reserve(path.size());
  cleaned.push_back(path.front());
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    if ((path[index] - cleaned.back()).norm() > duplicate_epsilon) {
      cleaned.push_back(path[index]);
    }
  }
  if ((path.back() - cleaned.back()).norm() > 0.0) {
    cleaned.push_back(path.back());
  }
  return cleaned;
}

Eigen::Vector3d interpolateAtDistance(const std::vector<Eigen::Vector3d> &path,
                                      const std::vector<double> &cumulative_lengths,
                                      double distance) {
  if (distance <= 0.0) {
    return path.front();
  }
  if (distance >= cumulative_lengths.back()) {
    return path.back();
  }

  const auto upper = std::upper_bound(cumulative_lengths.begin(), cumulative_lengths.end(), distance);
  const std::size_t segment = static_cast<std::size_t>(upper - cumulative_lengths.begin() - 1);
  const double segment_length = cumulative_lengths[segment + 1] - cumulative_lengths[segment];
  if (segment_length <= 0.0) {
    return path[segment + 1];
  }
  const double ratio = (distance - cumulative_lengths[segment]) / segment_length;
  return (1.0 - ratio) * path[segment] + ratio * path[segment + 1];
}

}  // namespace

std::vector<Eigen::Vector3d> resamplePath(const std::vector<Eigen::Vector3d> &path,
                                          const PathResamplingOptions &options) {
  if (path.empty()) {
    throw std::invalid_argument("path resampling requires at least one point");
  }
  if (!isFinite(options.spacing) || options.spacing <= 0.0) {
    throw std::invalid_argument("path resampling spacing must be finite and positive");
  }
  if (options.minimum_points < 2U) {
    throw std::invalid_argument("path resampling minimum_points must be at least two");
  }
  if (!isFinite(options.duplicate_epsilon) || options.duplicate_epsilon < 0.0) {
    throw std::invalid_argument("path resampling duplicate_epsilon must be finite and non-negative");
  }
  for (const Eigen::Vector3d &point : path) {
    if (!point.allFinite()) {
      throw std::invalid_argument("path resampling points must be finite");
    }
  }

  const std::vector<Eigen::Vector3d> cleaned =
      removeConsecutiveDuplicates(path, options.duplicate_epsilon);
  std::vector<double> cumulative_lengths(cleaned.size(), 0.0);
  for (std::size_t index = 1; index < cleaned.size(); ++index) {
    cumulative_lengths[index] = cumulative_lengths[index - 1] +
                                (cleaned[index] - cleaned[index - 1]).norm();
  }
  const double total_length = cumulative_lengths.back();
  const double spacing_ratio = total_length / options.spacing;
  if (!isFinite(spacing_ratio) ||
      spacing_ratio > static_cast<double>(std::numeric_limits<std::size_t>::max() - 2U)) {
    throw std::invalid_argument("path resampling would require too many samples");
  }

  const std::size_t full_spacing_steps = static_cast<std::size_t>(std::floor(spacing_ratio));
  std::size_t spacing_sample_count = full_spacing_steps + 1U;
  if (spacing_sample_count < 2U) {
    spacing_sample_count = 2U;
  }
  const std::size_t sample_count = std::max(spacing_sample_count, options.minimum_points);
  std::vector<Eigen::Vector3d> result;
  result.reserve(sample_count);

  if (sample_count == options.minimum_points && spacing_sample_count < options.minimum_points) {
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) /
                           static_cast<double>(sample_count - 1U);
      result.push_back(interpolateAtDistance(cleaned, cumulative_lengths, ratio * total_length));
    }
  } else {
    for (std::size_t sample = 0; sample < spacing_sample_count - 1U; ++sample) {
      result.push_back(interpolateAtDistance(cleaned, cumulative_lengths,
                                             static_cast<double>(sample) * options.spacing));
    }
    result.push_back(cleaned.back());
  }

  // Avoid accumulated floating-point error at the boundaries and preserve
  // the exact A* request endpoints for downstream trajectory initialization.
  result.front() = path.front();
  result.back() = path.back();
  return result;
}

}  // namespace aurora::math
