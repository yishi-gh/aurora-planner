#!/usr/bin/env python3

import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
import rosbag2_py
from aurora_msgs.msg import PlanningResult, Trajectory
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools" / "rosbag2"))
from generate_static_baseline import (  # noqa: E402
    generate_static_baseline,
    PLANNING_REQUEST_ID,
)


SCENARIO_DIR = Path(tempfile.mkdtemp(prefix="aurora_static_baseline_", dir="/tmp"))
BAG_URI = SCENARIO_DIR / "static_3d_baseline"
MANIFEST_PATH = generate_static_baseline(BAG_URI)
REPORT_PATH = SCENARIO_DIR / "replay_report.json"


def read_bag_time_bounds(bag_uri: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    timestamps = []
    while reader.has_next():
        _topic, _serialized, timestamp = reader.read_next()
        timestamps.append(timestamp)
    if not timestamps:
        raise AssertionError("generated bag contains no messages")
    return min(timestamps), max(timestamps), len(timestamps)


def finite_or_none(value):
    return value if math.isfinite(value) else None


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node_static_bag",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": str(SCENARIO_DIR / "planner_logs"),
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "use_sim_time": True,
                "map.reject_unknown": False,
                "map.require_fresh_observation": False,
                "planning.local_horizon": 6.0,
                "planning.resampling_spacing": 0.5,
                "planning.resampling_minimum_points": 9,
                "planning.optimizer_interval": 0.5,
                "planning.optimizer_max_iterations": 80,
                "planning.optimizer_samples_per_span": 8,
                "planning.validation_samples_per_span": 16,
                "risk.sample_interval": 0.1,
                "risk.max_prediction_age": 0.5,
            }
        ],
    )
    bag_play = launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            str(BAG_URI),
            "--clock-topics-all",
            "--rate",
            "1.0",
            "--disable-keyboard-controls",
            "--wait-for-all-acked",
            "1000",
        ],
        output="screen",
        additional_env={"RCUTILS_LOGGING_BUFFERED_STREAM": "1"},
    )
    return (
        launch.LaunchDescription(
            [
                planner_node,
                launch.actions.TimerAction(period=1.0, actions=[bag_play]),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"planner": planner_node, "bag_play": bag_play},
    )


class TestStaticRosbag2Replay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_static_rosbag2_observer")
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.results = []
        self.trajectories = []
        self.result_subscription = self.node.create_subscription(
            PlanningResult,
            "/aurora/planning_result",
            self.results.append,
            reliable_qos,
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory,
            "/aurora/trajectory",
            self.trajectories.append,
            reliable_qos,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.result_subscription)
        self.node.destroy_subscription(self.trajectory_subscription)
        self.node.destroy_node()

    def wait_for_result(self, request_id, timeout_sec=20.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id:
                    return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for rosbag2 planning result %d" % request_id)

    def write_report(self, result, bag_start_ns, bag_end_ns, bag_message_count):
        report = {
            "scenario_manifest": str(MANIFEST_PATH),
            "bag_start_ns": bag_start_ns,
            "bag_end_ns": bag_end_ns,
            "bag_message_count": bag_message_count,
            "request_id": result.request_id,
            "result_header_stamp_ns": (
                result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec
            ),
            "result_status": result.status,
            "has_trajectory": result.has_trajectory,
            "trajectory": {
                "trajectory_id": result.trajectory.trajectory_id,
                "map_version": result.trajectory.map_version,
                "validation_state": result.trajectory.validation_state,
                "segment_count": len(result.trajectory.segments),
            },
            "risk_report": {
                "model_id": result.risk_report.model_id,
                "total_risk": result.risk_report.total_risk,
                "dynamic_risk": result.risk_report.dynamic_risk,
                "minimum_clearance": finite_or_none(
                    result.risk_report.minimum_clearance
                ),
                "risk_level": result.risk_report.risk_level,
                "dynamic_information_available": result.risk_report.dynamic_information_available,
                "dynamic_information_stale": result.risk_report.dynamic_information_stale,
            },
            "trajectory_message_count": len(self.trajectories),
            "unvalidated_trajectory_count": sum(
                message.validation_state != Trajectory.VALIDATED
                for message in self.trajectories
            ),
        }
        REPORT_PATH.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    def test_static_bag_replays_to_validated_3d_trajectory(self):
        expected = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        bag_start_ns, bag_end_ns, bag_message_count = read_bag_time_bounds(BAG_URI)
        self.assertEqual(bag_start_ns, expected["bag_start_ns"])
        self.assertEqual(bag_end_ns, expected["bag_end_ns"])
        self.assertEqual(bag_message_count, len(expected["events"]))

        result = self.wait_for_result(PLANNING_REQUEST_ID)
        # Result and trajectory are different topics, so allow their DDS
        # callbacks to drain instead of relying on cross-topic arrival order.
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        try:
            self.assertEqual(result.header.stamp.sec * 1_000_000_000 + result.header.stamp.nanosec,
                             expected["bag_end_ns"])
            self.assertEqual(result.request_id, expected["expected_request_id"])
            self.assertEqual(result.status, PlanningResult.SUCCESS)
            self.assertTrue(result.has_trajectory)
            self.assertGreater(len(result.trajectory.segments), 0)
            self.assertGreater(result.trajectory.map_version, 0)
            self.assertEqual(result.trajectory.validation_state, Trajectory.VALIDATED)
            self.assertEqual(
                result.trajectory.safety_report.map_version,
                result.trajectory.map_version,
            )
            self.assertEqual(result.risk_report.model_id, "aurora.conservative_3sigma_v1")
            self.assertTrue(result.risk_report.dynamic_information_available)
            self.assertFalse(result.risk_report.dynamic_information_stale)
            self.assertEqual(result.risk_report.risk_level, result.risk_report.LOW)
            self.assertEqual(result.risk_report.dynamic_risk, 0.0)
            self.assertGreaterEqual(len(self.trajectories), 1)
            self.assertTrue(
                all(
                    message.validation_state == Trajectory.VALIDATED
                    for message in self.trajectories
                )
            )
            self.assertFalse(
                any(
                    message.validation_state
                    in (Trajectory.DEGRADED, Trajectory.REJECTED)
                    for message in self.trajectories
                )
            )
        finally:
            self.write_report(result, bag_start_ns, bag_end_ns, bag_message_count)


@launch_testing.post_shutdown_test()
class TestStaticRosbag2ReplayShutdown(unittest.TestCase):
    def test_bag_play_exited_successfully(self, proc_info, bag_play):
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_play)
