#!/usr/bin/env python3

import json
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
from aurora_msgs.msg import EmergencyStopState, PlannerStatus, PlanningResult, Trajectory
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools" / "rosbag2"))
from generate_information_stale_and_occlusion import (  # noqa: E402
    OCCLUSION_REQUEST_ID,
    generate_explicit_occlusion,
)


SCENARIO_DIR = Path(tempfile.mkdtemp(prefix="aurora_dynamic_occlusion_", dir="/tmp"))
BAG_URI = SCENARIO_DIR / "explicit_dynamic_occlusion"
MANIFEST_PATH = generate_explicit_occlusion(BAG_URI)


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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node_dynamic_occlusion_bag",
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
                "risk.stale_hold_duration": 0.5,
                "risk.information_watchdog_rate_hz": 10.0,
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


class TestDynamicOcclusionRosbag2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_dynamic_occlusion_rosbag2_observer")
        reliable_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.results = []
        self.trajectories = []
        self.statuses = []
        self.emergency_states = []
        self.result_subscription = self.node.create_subscription(
            PlanningResult, "/aurora/planning_result", self.results.append, reliable_qos
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory, "/aurora/trajectory", self.trajectories.append, reliable_qos
        )
        self.status_subscription = self.node.create_subscription(
            PlannerStatus, "/aurora/planner_status", self.statuses.append, reliable_qos
        )
        self.emergency_subscription = self.node.create_subscription(
            EmergencyStopState,
            "/aurora/emergency_stop_state",
            self.emergency_states.append,
            reliable_qos,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.result_subscription)
        self.node.destroy_subscription(self.trajectory_subscription)
        self.node.destroy_subscription(self.status_subscription)
        self.node.destroy_subscription(self.emergency_subscription)
        self.node.destroy_node()

    def wait_until(self, predicate, timeout_sec=20.0, detail="condition"):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for %s" % detail)

    def result_for(self, status):
        return [
            result
            for result in self.results
            if result.request_id == OCCLUSION_REQUEST_ID and result.status == status
        ]

    def test_explicit_occlusion_is_not_an_empty_heartbeat(self):
        expected = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        bag_start_ns, bag_end_ns, bag_message_count = read_bag_time_bounds(BAG_URI)
        self.assertEqual(bag_start_ns, expected["bag_start_ns"])
        self.assertEqual(bag_end_ns, expected["bag_end_ns"])
        self.assertEqual(bag_message_count, len(expected["events"]))

        self.wait_until(
            lambda: bool(self.result_for(PlanningResult.SUCCESS)),
            detail="initial successful planning result",
        )
        self.wait_until(
            lambda: len(self.trajectories) >= 1,
            detail="initial validated trajectory publication",
        )
        initial_trajectory_count = len(self.trajectories)

        self.wait_until(
            lambda: bool(self.result_for(PlanningResult.VALIDATION_FAILED)),
            detail="explicit occlusion rejection result",
        )
        occlusion_result = self.result_for(PlanningResult.VALIDATION_FAILED)[-1]
        self.assertFalse(occlusion_result.has_trajectory)
        self.assertTrue(occlusion_result.risk_report.dynamic_information_available)
        self.assertTrue(occlusion_result.risk_report.dynamic_information_stale)
        self.assertIn("occluded", occlusion_result.detail)
        self.assertEqual(
            occlusion_result.safety_report.status,
            occlusion_result.safety_report.INFORMATION_STALE,
        )

        self.wait_until(
            lambda: any(
                state.active
                and state.latched
                and state.reason == EmergencyStopState.INFORMATION_STALE
                for state in self.emergency_states
            ),
            detail="occlusion emergency stop",
        )
        self.assertEqual(len(self.trajectories), initial_trajectory_count)
        self.assertTrue(
            all(item.validation_state == Trajectory.VALIDATED for item in self.trajectories)
        )


@launch_testing.post_shutdown_test()
class TestDynamicOcclusionRosbag2Shutdown(unittest.TestCase):
    def test_bag_play_exited_successfully(self, proc_info, bag_play):
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_play)
