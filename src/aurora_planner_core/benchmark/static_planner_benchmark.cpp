#include "aurora_planner_core/static_local_planner.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using aurora::map::VoxelMap;
using aurora::map::VoxelMapConfig;
using aurora::planner::GlobalReference;
using aurora::planner::LocalPlannerOptions;
using aurora::planner::PlanningRequest;
using aurora::planner::PlanningStatus;
using aurora::planner::StaticLocalPlanner;

struct Options {
  std::size_t warmup{10U};
  std::size_t iterations{200U};
  bool soft_risk{false};
  std::string output;
};

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto valueAfter = [&](const std::string &name) {
      if (index + 1 >= argc) {
        throw std::invalid_argument(name + " requires a value");
      }
      return std::string(argv[++index]);
    };
    if (argument == "--warmup") {
      options.warmup = static_cast<std::size_t>(std::stoull(valueAfter(argument)));
    } else if (argument == "--iterations") {
      options.iterations = static_cast<std::size_t>(std::stoull(valueAfter(argument)));
    } else if (argument == "--soft-risk") {
      options.soft_risk = true;
    } else if (argument == "--output") {
      options.output = valueAfter(argument);
    } else if (argument == "--help") {
      std::cout << "usage: aurora_static_planner_benchmark [--warmup N] "
                   "[--iterations N] [--soft-risk] [--output PATH]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.iterations == 0U) {
    throw std::invalid_argument("--iterations must be positive");
  }
  return options;
}

VoxelMap makeFreeMap() {
  VoxelMapConfig config;
  config.origin = Eigen::Vector3d(-8.0, -5.0, 0.0);
  config.dimensions = Eigen::Vector3i(32, 20, 12);
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

PlanningRequest makeRequest(std::uint64_t request_id) {
  PlanningRequest request;
  request.request_id = request_id;
  request.planning_stamp = 10.0;
  request.vehicle_state.stamp = request.planning_stamp;
  request.vehicle_state.position = Eigen::Vector3d(-4.0, 0.0, 1.0);
  request.vehicle_state.velocity.setZero();
  request.vehicle_state.acceleration.setZero();
  request.global_reference = GlobalReference::fromWaypoints({
      Eigen::Vector3d(-4.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(4.0, 0.0, 1.0),
  });
  return request;
}

LocalPlannerOptions makePlannerOptions() {
  LocalPlannerOptions options;
  options.local_horizon = 3.0;
  options.resampling.spacing = 0.5;
  options.resampling.minimum_points = 9;
  options.optimizer.interval = 0.5;
  options.optimizer.clearance = 0.6;
  options.optimizer.max_velocity = 3.0;
  options.optimizer.max_acceleration = 6.0;
  options.optimizer.lambda_obstacle = 20.0;
  options.optimizer.max_iterations = 80;
  options.validation.samples_per_span = 16;
  options.validation.max_velocity = 3.0;
  options.validation.max_acceleration = 6.0;
  options.stitch_prefix_duration = 0.0;
  return options;
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const auto rank = static_cast<std::size_t>(std::ceil(fraction * values.size()));
  return values[std::max<std::size_t>(1U, rank) - 1U];
}

std::string makeReport(const Options &options, const VoxelMap &map,
                       const std::vector<double> &durations,
                       const std::map<std::string, std::size_t> &status_counts,
                       std::size_t risk_evaluations) {
  const double total = std::accumulate(durations.begin(), durations.end(), 0.0);
  const std::size_t successes =
      status_counts.count("SUCCESS") == 0U ? 0U : status_counts.at("SUCCESS");
  const auto milliseconds = [](double microseconds) { return microseconds / 1000.0; };
  std::ostringstream report;
  report << std::fixed << std::setprecision(6);
  report << "{\n";
  report << "  \"benchmark\": \"static_local_planner_v1\",\n";
  report << "  \"soft_risk_enabled\": " << (options.soft_risk ? "true" : "false") << ",\n";
  report << "  \"compiler\": \"" << __VERSION__ << "\",\n";
  report << "  \"map_voxel_count\": " << map.voxelCount() << ",\n";
  report << "  \"map_version\": " << map.version() << ",\n";
  report << "  \"warmup_iterations\": " << options.warmup << ",\n";
  report << "  \"iterations\": " << durations.size() << ",\n";
  report << "  \"success_count\": " << successes << ",\n";
  report << "  \"success_rate\": "
         << static_cast<double>(successes) / static_cast<double>(durations.size()) << ",\n";
  report << "  \"risk_evaluations\": " << risk_evaluations << ",\n";
  report << "  \"latency_ms\": {\n";
  report << "    \"mean\": " << milliseconds(total / durations.size()) << ",\n";
  report << "    \"p95\": " << milliseconds(percentile(durations, 0.95)) << ",\n";
  report << "    \"p99\": " << milliseconds(percentile(durations, 0.99)) << ",\n";
  report << "    \"max\": " << milliseconds(*std::max_element(durations.begin(), durations.end()))
         << "\n";
  report << "  },\n";
  report << "  \"status_counts\": {\n";
  bool first = true;
  for (const auto &[status, count] : status_counts) {
    if (!first) {
      report << ",\n";
    }
    first = false;
    report << "    \"" << status << "\": " << count;
  }
  report << "\n  }\n";
  report << "}\n";
  return report.str();
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const VoxelMap map = makeFreeMap();
    auto planner_options = makePlannerOptions();
    std::size_t risk_evaluations = 0U;
    std::optional<aurora::trajectory::RiskCostFunction> risk_cost;
    if (options.soft_risk) {
      // Keep the synthetic risk field active without making this benchmark a
      // dynamics-stress case; the trajectory validator remains the authority
      // for the max-acceleration contract.
      planner_options.optimizer.lambda_risk = 0.15;
      planner_options.optimizer.max_risk_evaluations = 10000U;
      risk_cost = [&](double absolute_stamp, const Eigen::Vector3d &position) {
        (void)absolute_stamp;
        ++risk_evaluations;
        aurora::trajectory::RiskCostEvaluation evaluation;
        const Eigen::Vector3d center(0.0, 0.2, 1.0);
        const Eigen::Vector3d displacement = position - center;
        const double distance = displacement.norm();
        const double warning_clearance = 1.0;
        const double clearance = distance - 0.8;
        evaluation.value = std::clamp(
            (warning_clearance - clearance) / warning_clearance, 0.0, 1.0);
        if (evaluation.value > 0.0 && distance > 1e-12) {
          evaluation.gradient = -displacement / (warning_clearance * distance);
        }
        return evaluation;
      };
    }
    const StaticLocalPlanner planner(planner_options);

    for (std::size_t index = 0; index < options.warmup; ++index) {
      const auto result = planner.plan(map, makeRequest(index), std::nullopt, risk_cost);
      if (risk_cost.has_value() && result.status != PlanningStatus::SUCCESS) {
        throw std::runtime_error("soft-risk warmup planning failed: " +
                                 std::string(aurora::planner::toString(result.status)) +
                                 ": " + result.detail);
      }
      if (result.status != PlanningStatus::SUCCESS) {
        throw std::runtime_error("warmup planning failed: " +
                                 std::string(aurora::planner::toString(result.status)) +
                                 ": " + result.detail);
      }
    }

    std::vector<double> durations;
    durations.reserve(options.iterations);
    std::map<std::string, std::size_t> status_counts;
    for (std::size_t index = 0; index < options.iterations; ++index) {
      const auto start = std::chrono::steady_clock::now();
      const auto result = planner.plan(map, makeRequest(options.warmup + index), std::nullopt,
                                       risk_cost);
      const auto end = std::chrono::steady_clock::now();
      durations.push_back(
          std::chrono::duration<double, std::micro>(end - start).count());
      ++status_counts[aurora::planner::toString(result.status)];
    }

    const std::string report = makeReport(options, map, durations, status_counts,
                                          risk_evaluations);
    std::cout << report;
    if (!options.output.empty()) {
      std::ofstream output(options.output);
      if (!output) {
        throw std::runtime_error("cannot open benchmark output: " + options.output);
      }
      output << report;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "benchmark error: " << error.what() << "\n";
    return 1;
  }
}
