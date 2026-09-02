# Changelog

All notable changes to AURORA-Planner are recorded here. The current entry is
a development baseline, not a declaration of flight readiness.

## 0.1.0 - Development baseline - 2026-09-02

### Added

- Independent ROS 2 fork of the EGO-style three-dimensional local flight-planning pipeline.
- Probabilistic voxel mapping, 26-neighbor A* search, cubic B-spline initialization, static optimization, and trajectory validation.
- Strict seventh-order minimum-snap reference generation and deterministic initial segment-time allocation.
- KF-CV/KF-CA dynamic-obstacle prediction, covariance propagation, association, and track lifecycle management.
- Unified uncertainty context covering map quality, delay, localization/execution covariance, and obstacle existence probability.
- Conservative dynamic 3-sigma gating, soft-risk B-spline optimization, bounded fallback, emergency-stop and stale-information handling.
- ROS 2 messages, PointCloud2/depth-image adapters, TF handling, rosbag2 regression scenarios, flight-admission boundary, and deterministic three-dimensional software-in-the-loop execution.

### Verification

- Ubuntu 24.04 + ROS 2 Jazzy: 13 packages built and 223 tests passed.
- Core, ROS adapter, flight-admission, and software-in-the-loop C++ sources pass the configured cppcheck checks.
- AURORA source packages and top-level release metadata use the user-confirmed GPL-3.0-only license.

### Known limitations

- Humble CI evidence, PX4/Gazebo/GZ integration, real sensor and flight-controller validation, target-hardware real-time measurements, and final installed-environment SBOM remain open.
- The deterministic software-in-the-loop executor is not a substitute for real flight dynamics or flight-controller acceptance.
