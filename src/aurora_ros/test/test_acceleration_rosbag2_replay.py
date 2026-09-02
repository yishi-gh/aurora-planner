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
from generate_acceleration_and_stopping import (  # noqa: E402
    ACCELERATION_REQUEST_ID,
    ACCELERATION_TRACK_ID,
    generate_accelerating_crossing,
)


SCENARIO_DIR = Path(tempfile.mkdtemp(prefix="aurora_acceleration_", dir="/tmp"))
BAG_URI = SCENARIO_DIR / "constant_acceleration_crossing"
MANIFEST_PATH = generate_accelerating_crossing(BAG_URI)
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node_acceleration_bag",
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


class TestAccelerationRosbag2Replay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_acceleration_rosbag2_observer")
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
        self.status_subscription = self.node.create_subscription(
            PlannerStatus,
            "/aurora/planner_status",
            self.statuses.append,
            reliable_qos,
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

    def results_with_status(self, status):
        return [
            result
            for result in self.results
            if result.request_id == ACCELERATION_REQUEST_ID and result.status == status
        ]

    def write_report(self, bag_start_ns, bag_end_ns, bag_message_count):
        request_results = [
            {
                "request_id": result.request_id,
                "header_stamp_ns": result.header.stamp.sec * 1_000_000_000
                + result.header.stamp.nanosec,
                "status": result.status,
                "has_trajectory": result.has_trajectory,
                "map_version": result.trajectory.map_version
                if result.has_trajectory
                else result.safety_report.map_version,
                "risk_level": result.risk_report.risk_level,
                "dynamic_risk": result.risk_report.dynamic_risk,
                "worst_obstacle_id": result.risk_report.worst_obstacle_id,
            }
            for result in self.results
            if result.request_id == ACCELERATION_REQUEST_ID
        ]
        report = {
            "scenario_manifest": str(MANIFEST_PATH),
            "bag_start_ns": bag_start_ns,
            "bag_end_ns": bag_end_ns,
            "bag_message_count": bag_message_count,
            "request_id": ACCELERATION_REQUEST_ID,
            "request_results": request_results,
            "trajectory_message_count": len(self.trajectories),
            "unvalidated_trajectory_count": sum(
                message.validation_state != Trajectory.VALIDATED
                for message in self.trajectories
            ),
            "dynamic_replan_status_count": sum(
                status.request_id == ACCELERATION_REQUEST_ID
                and status.replan_trigger == PlannerStatus.DYNAMIC_OBSTACLE_UPDATED
                for status in self.statuses
            ),
            "emergency_stop_latched": any(
                state.active and state.latched for state in self.emergency_states
            ),
        }
        REPORT_PATH.write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )

    def test_acceleration_enters_envelope_and_fails_closed(self):
        expected = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        bag_start_ns, bag_end_ns, bag_message_count = read_bag_time_bounds(BAG_URI)
        self.assertEqual(bag_start_ns, expected["bag_start_ns"])
        self.assertEqual(bag_end_ns, expected["bag_end_ns"])
        self.assertEqual(bag_message_count, len(expected["events"]))

        try:
            self.wait_until(
                lambda: bool(self.results_with_status(PlanningResult.SUCCESS)),
                detail="initial successful planning result",
            )
            self.assertTrue(
                any(
                    result.has_trajectory
                    and result.trajectory.validation_state == Trajectory.VALIDATED
                    for result in self.results_with_status(PlanningResult.SUCCESS)
                )
            )

            self.wait_until(
                lambda: bool(self.results_with_status(PlanningResult.VALIDATION_FAILED)),
                detail="constant-acceleration dynamic collision rejection result",
            )
            collision_result = self.results_with_status(PlanningResult.VALIDATION_FAILED)[-1]
            self.assertFalse(collision_result.has_trajectory)
            self.assertEqual(len(collision_result.trajectory.segments), 0)
            self.assertEqual(
                collision_result.risk_report.risk_level,
                collision_result.risk_report.HIGH,
            )
            self.assertEqual(collision_result.risk_report.dynamic_risk, 1.0)
            self.assertEqual(
                collision_result.risk_report.worst_obstacle_id,
                ACCELERATION_TRACK_ID,
            )
            self.assertTrue(collision_result.risk_report.dynamic_information_available)
            self.assertFalse(collision_result.risk_report.dynamic_information_stale)
            self.wait_until(
                lambda: any(
                    status.request_id == ACCELERATION_REQUEST_ID
                    and status.replan_trigger == PlannerStatus.DYNAMIC_OBSTACLE_UPDATED
                    for status in self.statuses
                ),
                detail="acceleration dynamic obstacle active-replan status",
            )
            self.wait_until(
                lambda: any(state.active and state.latched for state in self.emergency_states),
                detail="latched emergency-stop state",
            )
            self.assertTrue(
                all(
                    message.validation_state == Trajectory.VALIDATED
                    for message in self.trajectories
                )
            )
        finally:
            self.write_report(bag_start_ns, bag_end_ns, bag_message_count)


@launch_testing.post_shutdown_test()
class TestAccelerationRosbag2ReplayShutdown(unittest.TestCase):
    def test_bag_play_exited_successfully(self, proc_info, bag_play):
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_play)
