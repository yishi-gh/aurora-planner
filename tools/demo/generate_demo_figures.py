#!/usr/bin/env python3
"""Capture AURORA ROS 2 data and render material-ready demonstration figures.

The planner and deterministic software-in-the-loop executor are the data
source.  This script does not reimplement the planner; it subscribes to the
published validated trajectory, risk result, and execution state topics.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import time
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/aurora_demo_matplotlib")
Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
os.environ.setdefault("ROS_LOG_DIR", "/tmp/aurora_demo_ros_log")
Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
os.environ.setdefault("ROS_DOMAIN_ID", "26")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageOps

import rclpy
from aurora_msgs.msg import (
    DynamicObstacleTrack,
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
    VehicleState,
)
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


MAP_FRAME = "map"
REQUEST_ID = 82601
TRACK_ID = 7
STATIC_ENDPOINTS = (
    (10.0, 0.0, 1.0),
    (10.0, 2.5, 1.0),
    (10.0, -2.5, 1.0),
    (10.0, 0.0, 3.0),
    (10.0, 0.0, -1.0),
    (0.0, 4.0, 1.0),
    (0.0, -4.0, 1.0),
)
COLORS = {
    "ink": "#172033",
    "muted": "#5d6b82",
    "grid": "#d9e0ea",
    "blue": "#1976d2",
    "cyan": "#008c95",
    "orange": "#e07a18",
    "red": "#c53d4d",
    "green": "#16805d",
    "gray": "#8793a5",
    "paper": "#f7f9fc",
}


def seconds_from_time(message) -> float:
    return float(message.sec) + float(message.nanosec) * 1e-9


def point_array(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z], dtype=float)


def stamp_now(node: Node):
    return node.get_clock().now().to_msg()


def make_point_cloud(stamp, endpoints=STATIC_ENDPOINTS) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = MAP_FRAME
    message.height = 1
    message.width = len(endpoints)
    message.is_bigendian = False
    message.is_dense = True
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = b"".join(struct.pack("<fff", *point) for point in endpoints)
    return message


def make_dynamic_heartbeat(stamp) -> DynamicObstacleTrackArray:
    message = DynamicObstacleTrackArray()
    message.header.stamp = stamp
    message.header.frame_id = MAP_FRAME
    return message


def make_crossing_track(stamp) -> DynamicObstacleTrackArray:
    track = DynamicObstacleTrack()
    track.header.stamp = stamp
    track.header.frame_id = MAP_FRAME
    track.track_id = TRACK_ID
    track.pose.position = Point(x=0.0, y=-1.5, z=1.0)
    track.twist.linear.y = 1.0
    track.shape_type = DynamicObstacleTrack.SPHERE
    track.radius = 0.25
    track.existence_probability = 1.0
    track.prediction_model = DynamicObstacleTrack.CV
    track.has_state_covariance = True
    track.state_covariance = [0.0] * 36
    for index in (0, 7, 14):
        track.state_covariance[index] = 0.0025
    for index in (21, 28, 35):
        track.state_covariance[index] = 0.01
    message = DynamicObstacleTrackArray()
    message.header.stamp = stamp
    message.header.frame_id = MAP_FRAME
    message.tracks.append(track)
    return message


def make_request(stamp, request_id, include_reference=True) -> PlanningRequest:
    message = PlanningRequest()
    message.header.stamp = stamp
    message.header.frame_id = MAP_FRAME
    message.request_id = request_id
    message.vehicle_state.header.stamp = stamp
    message.vehicle_state.header.frame_id = MAP_FRAME
    message.vehicle_state.position = Point(x=-4.0, y=0.0, z=1.0)
    if include_reference:
        # The reference is spatially 3-D, while the crossing obstacle remains
        # near z=1.0 so that the actual risk gate sees the encounter.
        waypoints = (
            (-4.0, 0.0, 1.0),
            (-2.0, 0.0, 1.0),
            (0.0, 0.0, 1.0),
            (2.0, 0.0, 1.45),
            (4.0, 0.0, 1.7),
        )
        for x, y, z in waypoints:
            reference = GlobalReferencePoint()
            reference.position = Point(x=x, y=y, z=z)
            reference.has_time = False
            message.global_reference.append(reference)
    return message


class DemoCollector(Node):
    def __init__(self) -> None:
        super().__init__("aurora_demo_collector")
        reliable = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.request_publisher = self.create_publisher(
            PlanningRequest, "/aurora/planning_request", reliable
        )
        self.track_publisher = self.create_publisher(
            DynamicObstacleTrackArray, "/aurora/dynamic_obstacle_tracks", reliable
        )
        self.pointcloud_publisher = self.create_publisher(PointCloud2, "/points", sensor)
        self.results = []
        self.trajectories = []
        self.vehicle_states = []
        self.desired_poses = []
        self.result_subscription = self.create_subscription(
            PlanningResult, "/aurora/planning_result", self.results.append, reliable
        )
        self.trajectory_subscription = self.create_subscription(
            Trajectory, "/aurora/trajectory", self.trajectories.append, reliable
        )
        self.vehicle_subscription = self.create_subscription(
            VehicleState, "/aurora/sim/vehicle_state", self.vehicle_states.append, reliable
        )
        from geometry_msgs.msg import PoseStamped

        self.desired_subscription = self.create_subscription(
            PoseStamped, "/aurora/sim/desired_pose", self.desired_poses.append, reliable
        )

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def wait_for_interfaces(self, timeout: float = 15.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (
                self.count_subscribers("/aurora/planning_request") > 0
                and self.count_subscribers("/aurora/dynamic_obstacle_tracks") > 0
                and self.count_subscribers("/points") > 0
                and self.count_publishers("/aurora/planning_result") > 0
                and self.count_publishers("/aurora/trajectory") > 0
                and self.count_publishers("/aurora/sim/vehicle_state") > 0
            ):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("AURORA demo ROS 2 interfaces did not become discoverable")

    def wait_for_result(self, request_id: int, timeout: float = 20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id:
                    return result
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("timed out waiting for planning result %d" % request_id)


def terminate_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def start_nodes() -> tuple[subprocess.Popen, subprocess.Popen]:
    common = [
        "--ros-args",
        "-p",
        "map.reject_unknown:=false",
        "-p",
        "map.require_fresh_observation:=false",
        "-p",
        "planning.local_horizon:=6.0",
        "-p",
        "planning.resampling_spacing:=0.5",
        "-p",
        "planning.resampling_minimum_points:=9",
        "-p",
        "planning.optimizer_interval:=0.5",
        "-p",
        "planning.optimizer_max_iterations:=80",
        "-p",
        "planning.optimizer_samples_per_span:=8",
        "-p",
        "planning.validation_samples_per_span:=16",
        "-p",
        "planning.optimizer_lambda_risk:=0.0",
        "-p",
        "risk.sample_interval:=0.1",
        "-p",
        # The demo spends a short interval collecting execution samples before
        # injecting the dynamic event; keep the project default safety policy
        # unchanged and use a wider capture-only freshness window.
        "risk.max_prediction_age:=3.0",
    ]
    environment = os.environ.copy()
    environment.setdefault("ROS_DOMAIN_ID", "26")
    planner_log = "/tmp/aurora_demo_planner.log"
    sim_log = "/tmp/aurora_demo_sim.log"
    planner = subprocess.Popen(
        ["ros2", "run", "aurora_ros", "aurora_planner_node"] + common,
        stdout=open(planner_log, "w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        env=environment,
    )
    simulator = subprocess.Popen(
        ["ros2", "run", "aurora_sim", "aurora_sim_node", "--ros-args", "-p",
         "simulation.max_update_gap:=1.0"],
        stdout=open(sim_log, "w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        env=environment,
    )
    return planner, simulator


def capture(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    planner = None
    simulator = None
    rclpy.init()
    collector = DemoCollector()
    try:
        planner, simulator = start_nodes()
        collector.wait_for_interfaces()

        # Prime the planner state before integrating the first point cloud.
        heartbeat = make_dynamic_heartbeat(stamp_now(collector))
        for _ in range(4):
            collector.track_publisher.publish(heartbeat)
            collector.spin_for(0.08)
        prime_stamp = stamp_now(collector)
        prime = make_request(prime_stamp, REQUEST_ID - 1, include_reference=False)
        for _ in range(3):
            collector.request_publisher.publish(prime)
            collector.spin_for(0.08)

        cloud_stamp = stamp_now(collector)
        cloud = make_point_cloud(cloud_stamp)
        for _ in range(5):
            collector.pointcloud_publisher.publish(cloud)
            collector.spin_for(0.08)

        # Refresh the dynamic-information heartbeat immediately before the
        # request so the planner does not reject the static capture while the
        # point-cloud/map update is being drained.
        fresh_heartbeat = make_dynamic_heartbeat(stamp_now(collector))
        for _ in range(3):
            collector.track_publisher.publish(fresh_heartbeat)
            collector.spin_for(0.05)
        request_stamp = stamp_now(collector)
        request = make_request(request_stamp, REQUEST_ID)
        collector.request_publisher.publish(request)
        static_result = collector.wait_for_result(REQUEST_ID)
        if not static_result.has_trajectory:
            raise RuntimeError(
                "static demo request did not produce a trajectory: %s" % static_result.detail
            )
        static_trajectory = static_result.trajectory

        # Let the real software-in-the-loop node consume the validated result
        # before injecting the dynamic crossing event.
        collector.spin_for(0.8)
        dynamic_stamp = stamp_now(collector)
        crossing = make_crossing_track(dynamic_stamp)
        for _ in range(3):
            collector.track_publisher.publish(crossing)
            collector.spin_for(0.08)

        dynamic_result = None
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and dynamic_result is None:
            for result in collector.results:
                if (
                    result.request_id == REQUEST_ID
                    and result.status == PlanningResult.VALIDATION_FAILED
                ):
                    dynamic_result = result
                    break
            rclpy.spin_once(collector, timeout_sec=0.05)
        if dynamic_result is None:
            observed = [
                {"status": int(item.status), "detail": item.detail}
                for item in collector.results
                if item.request_id == REQUEST_ID
            ]
            raise RuntimeError(
                "dynamic crossing did not produce a risk rejection; observed=%s" % observed
            )

        # Keep the output data tied to the messages actually received.
        metadata = {
            "source": "live ROS 2 topic capture from aurora_ros + aurora_sim",
            "frame_id": MAP_FRAME,
            "request_id": REQUEST_ID,
            "trajectory_id": int(static_trajectory.trajectory_id),
            "map_version": int(static_trajectory.map_version),
            "static_status": int(static_result.status),
            "static_detail": static_result.detail,
            "dynamic_status": int(dynamic_result.status),
            "dynamic_detail": dynamic_result.detail,
            "dynamic_risk_level": int(dynamic_result.risk_report.risk_level),
            "dynamic_risk": float(dynamic_result.risk_report.dynamic_risk),
            "worst_obstacle_id": int(dynamic_result.risk_report.worst_obstacle_id),
            "trajectory_message_count": len(collector.trajectories),
            "execution_state_message_count": len(collector.vehicle_states),
            "desired_pose_message_count": len(collector.desired_poses),
            "static_endpoints": [list(point) for point in STATIC_ENDPOINTS],
            "dynamic_track": {
                "track_id": TRACK_ID,
                "position": [0.0, -1.5, 1.0],
                "velocity": [0.0, 1.0, 0.0],
                "position_variance": 0.0025,
                "velocity_variance": 0.01,
                "model": "CV",
            },
            "commands": {
                "planner": "ros2 run aurora_ros aurora_planner_node",
                "executor": "ros2 run aurora_sim aurora_sim_node",
            },
        }
        (output / "demo_manifest.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        render_figures(output, static_trajectory, static_result, dynamic_result, crossing,
                       collector)
        return metadata
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if simulator is not None:
            terminate_process(simulator)
        if planner is not None:
            terminate_process(planner)


def knot_vector(control_count: int, degree: int, ego_unclamped: bool) -> np.ndarray:
    values = []
    for index in range(control_count + degree + 1):
        if ego_unclamped and index <= degree:
            values.append(float(index - degree))
        elif index <= degree:
            values.append(0.0)
        elif index >= control_count:
            values.append(float(control_count - degree))
        else:
            values.append(float(index - degree))
    return np.asarray(values, dtype=float)


def de_boor(control_points: np.ndarray, degree: int, knots: np.ndarray, u: float) -> np.ndarray:
    count = len(control_points)
    u = float(np.clip(u, knots[degree], knots[count]))
    if u >= knots[count] - 1e-12:
        span = count - 1
    else:
        span = degree
        for candidate in range(degree, count):
            if knots[candidate] <= u < knots[candidate + 1]:
                span = candidate
                break
    work = np.array(control_points[span - degree: span + 1], dtype=float)
    for order in range(1, degree + 1):
        for index in range(degree, order - 1, -1):
            knot_index = span - degree + index
            denominator = knots[knot_index + degree - order + 1] - knots[knot_index]
            alpha = ((u - knots[knot_index]) / denominator) if denominator > 1e-12 else 0.0
            work[index] = (1.0 - alpha) * work[index - 1] + alpha * work[index]
    return work[degree]


def sample_trajectory(message: Trajectory, samples_per_segment: int = 80):
    times = []
    points = []
    start_stamp = min(
        seconds_from_time(segment.start_stamp) for segment in message.segments
    )
    for segment in message.segments:
        control_points = np.asarray(
            [point_array(point) for point in segment.control_points], dtype=float
        )
        degree = int(segment.degree)
        knots = knot_vector(
            len(control_points), degree,
            int(segment.knot_mode) == segment.EGO_UNCLAMPED,
        )
        local_count = max(2, samples_per_segment)
        segment_start = seconds_from_time(segment.start_stamp)
        for index in range(local_count):
            fraction = index / float(local_count - 1)
            absolute = segment_start + fraction * float(segment.duration)
            spline_time = float(segment.source_start_time) + fraction * float(segment.duration)
            points.append(de_boor(control_points, degree, knots, spline_time / float(segment.dt)))
            times.append(absolute - start_stamp)
    sampled_times = np.asarray(times, dtype=float)
    sampled_points = np.asarray(points, dtype=float)
    order = np.argsort(sampled_times, kind="stable")
    sampled_times = sampled_times[order]
    sampled_points = sampled_points[order]
    keep = np.ones(len(sampled_times), dtype=bool)
    if len(keep) > 1:
        keep[1:] = np.diff(sampled_times) > 1e-8
    return sampled_times[keep], sampled_points[keep], start_stamp


def plot_style(ax):
    ax.set_facecolor(COLORS["paper"])
    ax.grid(True, color=COLORS["grid"], linewidth=0.7, alpha=0.8)
    ax.tick_params(colors=COLORS["muted"], labelsize=9)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.label.set_color(COLORS["ink"])
    ax.view_init(elev=24, azim=-62)


def set_3d_bounds(ax, arrays, padding=0.7):
    values = np.vstack([array for array in arrays if len(array)])
    low = values.min(axis=0) - padding
    high = values.max(axis=0) + padding
    center = 0.5 * (low + high)
    radius = 0.5 * max(high - low)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1.3, 1.0, 0.8))


def save_figure(fig, path: Path) -> None:
    fig.savefig(path, dpi=180, facecolor=COLORS["paper"], bbox_inches="tight")
    plt.close(fig)


def render_static(output: Path, trajectory: Trajectory, result: PlanningResult) -> Path:
    _, path, _ = sample_trajectory(trajectory)
    reference = np.asarray(
        [(-4.0, 0.0, 1.0), (-2.0, 0.0, 1.0), (0.0, 0.0, 1.0),
         (2.0, 0.0, 1.45), (4.0, 0.0, 1.7)], dtype=float
    )
    endpoints = np.asarray(STATIC_ENDPOINTS, dtype=float)
    fig = plt.figure(figsize=(10.5, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    plot_style(ax)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color=COLORS["blue"], linewidth=3.2,
            label="validated B-spline")
    ax.plot(reference[:, 0], reference[:, 1], reference[:, 2], color=COLORS["orange"],
            linestyle="--", linewidth=1.7, marker="o", markersize=4, label="reference")
    ax.scatter(*path[0], color=COLORS["green"], s=72, marker="o", label="start")
    ax.scatter(*path[-1], color=COLORS["red"], s=82, marker="*", label="local goal")
    ax.scatter(endpoints[:, 0], endpoints[:, 1], endpoints[:, 2], color=COLORS["gray"],
               s=20, marker="^", alpha=0.72, label="observed ray endpoints")
    ax.set_title("AURORA-Planner | 3-D local trajectory", loc="left", pad=18,
                 color=COLORS["ink"], fontsize=16, fontweight="bold")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.text2D(0.02, 0.02,
              "ROS 2 topic capture  |  VALIDATED  |  map v%d" % trajectory.map_version,
              transform=ax.transAxes, color=COLORS["muted"], fontsize=9)
    set_3d_bounds(ax, [path, reference, endpoints])
    path_out = output / "aurora-3d-planning.png"
    save_figure(fig, path_out)
    return path_out


def render_dynamic(output: Path, trajectory: Trajectory, result: PlanningResult,
                   track_batch: DynamicObstacleTrackArray) -> Path:
    _, path, trajectory_start = sample_trajectory(trajectory)
    track = track_batch.tracks[0]
    track_stamp = seconds_from_time(track.header.stamp)
    trajectory_end = trajectory_start + (path.shape[0] and float(
        seconds_from_time(trajectory.segments[-1].start_stamp)
        + trajectory.segments[-1].duration - trajectory_start
    ))
    horizon = max(2.8, trajectory_end - track_stamp)
    times = np.linspace(0.0, horizon, 100)
    p0 = point_array(track.pose.position)
    v0 = np.array([track.twist.linear.x, track.twist.linear.y, track.twist.linear.z])
    predicted = p0[None, :] + times[:, None] * v0[None, :]
    pvar = float(track.state_covariance[0])
    vvar = float(track.state_covariance[21])
    sigma = np.sqrt(np.maximum(0.0, pvar + times * times * vvar + times**3 / 3.0))
    theta = np.linspace(0.0, 2.0 * math.pi, 18)
    vertices = []
    for center, radius in zip(predicted[::4], (3.0 * sigma)[::4]):
        vertices.append([
            center + np.array([radius * math.cos(angle), 0.0, radius * math.sin(angle)])
            for angle in theta
        ])

    fig = plt.figure(figsize=(10.5, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    plot_style(ax)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color=COLORS["blue"], linewidth=3.0,
            label="planned UAV trajectory")
    ax.plot(predicted[:, 0], predicted[:, 1], predicted[:, 2], color=COLORS["red"],
            linewidth=2.4, label="CV predicted obstacle")
    ax.scatter(*p0, color=COLORS["orange"], s=76, marker="o", label="obstacle observation")
    if vertices:
        ax.add_collection3d(Poly3DCollection(vertices, color=COLORS["red"], alpha=0.10,
                                             linewidths=0.25, edgecolors=COLORS["red"]))
    risk_point = predicted[len(predicted) // 2]
    if math.isfinite(seconds_from_time(result.risk_report.worst_time)):
        risk_time = seconds_from_time(result.risk_report.worst_time) - track_stamp
        risk_index = int(np.clip(np.searchsorted(times, risk_time), 0, len(times) - 1))
        risk_point = predicted[risk_index]
    ax.scatter(*risk_point, color=COLORS["red"], s=115, marker="X", label="risk gate")
    ax.set_title("AURORA-Planner | dynamic prediction and risk gate", loc="left", pad=18,
                 color=COLORS["ink"], fontsize=16, fontweight="bold")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.text2D(0.02, 0.02,
              "CV model  |  3-sigma envelope  |  HIGH risk  |  obstacle #%d"
              % result.risk_report.worst_obstacle_id,
              transform=ax.transAxes, color=COLORS["muted"], fontsize=9)
    set_3d_bounds(ax, [path, predicted, p0[None, :]], padding=0.8)
    path_out = output / "aurora-dynamic-risk.png"
    save_figure(fig, path_out)
    return path_out


def render_execution(output: Path, collector: DemoCollector) -> Path:
    actual = np.asarray([point_array(message.position) for message in collector.vehicle_states])
    desired = np.asarray([point_array(message.pose.position) for message in collector.desired_poses])
    fig = plt.figure(figsize=(11.5, 6.5))
    ax = fig.add_subplot(121, projection="3d")
    plot_style(ax)
    if len(actual):
        ax.plot(actual[:, 0], actual[:, 1], actual[:, 2], color=COLORS["green"],
                linewidth=2.5, label="executed state")
    if len(desired):
        ax.plot(desired[:, 0], desired[:, 1], desired[:, 2], color=COLORS["blue"],
                linestyle="--", linewidth=1.8, label="desired pose")
    ax.set_title("3-D execution", loc="left", pad=18, color=COLORS["ink"],
                 fontsize=15, fontweight="bold")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    if len(actual) or len(desired):
        set_3d_bounds(ax, [array for array in (actual, desired) if len(array)])

    error_ax = fig.add_subplot(122)
    error_ax.set_facecolor(COLORS["paper"])
    error_ax.grid(True, color=COLORS["grid"], linewidth=0.7)
    if len(actual) and len(desired):
        count = min(len(actual), len(desired))
        error = np.linalg.norm(actual[:count] - desired[:count], axis=1)
        error_time = np.linspace(0.0, max(0.1, count / 50.0), count)
        error_ax.plot(error_time, error, color=COLORS["cyan"], linewidth=2.3)
        error_ax.fill_between(error_time, error, color=COLORS["cyan"], alpha=0.10)
        error_ax.set_ylabel("position error [m]")
        error_ax.set_xlabel("execution time [s]")
        error_ax.set_title("tracking error", loc="left", color=COLORS["ink"],
                           fontsize=15, fontweight="bold")
        error_ax.text(0.02, 0.96, "deterministic S-i-L", transform=error_ax.transAxes,
                      va="top", color=COLORS["muted"], fontsize=9)
    else:
        error_ax.text(0.5, 0.5, "execution samples unavailable", ha="center", va="center",
                      color=COLORS["muted"])
        error_ax.set_axis_off()
    fig.suptitle("AURORA-Planner | validated trajectory execution", x=0.05, ha="left",
                 color=COLORS["ink"], fontsize=16, fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    path_out = output / "aurora-3d-execution.png"
    save_figure(fig, path_out)
    return path_out


def render_overview(output: Path, figure_paths) -> Path:
    canvas = Image.new("RGB", (1800, 1280), COLORS["paper"])
    positions = ((0, 0), (900, 0), (0, 640))
    for path, position in zip(figure_paths, positions):
        image = Image.open(path).convert("RGB")
        image = ImageOps.contain(image, (880, 620))
        canvas.paste(image, (position[0] + (880 - image.width) // 2,
                             position[1] + (620 - image.height) // 2))
    title = Image.new("RGB", (1800, 80), COLORS["paper"])
    canvas.paste(title, (0, 1200))
    path_out = output / "aurora-demo-overview.png"
    canvas.save(path_out, format="PNG", optimize=True)
    return path_out


def render_figures(output: Path, trajectory: Trajectory, static_result: PlanningResult,
                   dynamic_result: PlanningResult, track: DynamicObstacleTrackArray,
                   collector: DemoCollector) -> None:
    static_path = render_static(output, trajectory, static_result)
    dynamic_path = render_dynamic(output, trajectory, dynamic_result, track)
    execution_path = render_execution(output, collector)
    render_overview(output, (static_path, dynamic_path, execution_path))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).resolve().parents[2] / "demo",
        help="directory for PNG figures and demo_manifest.json",
    )
    args = parser.parse_args()
    metadata = capture(args.output.resolve())
    print(json.dumps(metadata, indent=2))
    print("figures=%s" % args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
