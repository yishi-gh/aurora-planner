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
    BRAKING_STAMP_NS,
    STOPPED_STAMP_NS,
    STOPPING_REQUEST_ID,
    STOPPING_TRACK_ID,
    generate_stopping_before_crossing,
)


SCENARIO_DIR = Path(tempfile.mkdtemp(prefix="aurora_stopping_", dir="/tmp"))
BAG_URI = SCENARIO_DIR / "constant_acceleration_braking_stop"
MANIFEST_PATH = generate_stopping_before_crossing(BAG_URI)
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


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node_stopping_bag",
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
                # This scenario isolates the measured stop state from
                # stochastic acceleration uncertainty. Risk inflation is
                # exercised by the acceleration-crossing scenario.
                "prediction.default_acceleration_variance": 0.0,
                "prediction.process_noise_jerk": 0.0,
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


class TestStoppingRosbag2Replay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_stopping_rosbag2_observer")
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

    def results_for_request(self):
        return [result for result in self.results if result.request_id == STOPPING_REQUEST_ID]

    def result_at_stamp(self, stamp):
        return [
            result
            for result in self.results_for_request()
            if stamp_ns(result) == stamp
        ]

    def write_report(self, bag_start_ns, bag_end_ns, bag_message_count):
        report = {
            "scenario_manifest": str(MANIFEST_PATH),
            "bag_start_ns": bag_start_ns,
            "bag_end_ns": bag_end_ns,
            "bag_message_count": bag_message_count,
            "request_id": STOPPING_REQUEST_ID,
            "request_results": [
                {
                    "request_id": result.request_id,
                    "header_stamp_ns": stamp_ns(result),
                    "status": result.status,
                    "detail": result.detail,
                    "has_trajectory": result.has_trajectory,
                    "risk_level": result.risk_report.risk_level,
                    "dynamic_risk": result.risk_report.dynamic_risk,
                    "worst_obstacle_id": result.risk_report.worst_obstacle_id,
                    "risk_detail": result.risk_report.detail,
                    "safety_status": result.safety_report.status,
                    "safety_accepted": result.safety_report.accepted,
                    "safety_detail": result.safety_report.detail,
                }
                for result in self.results_for_request()
            ],
            "trajectory_message_count": len(self.trajectories),
            "unvalidated_trajectory_count": sum(
                message.validation_state != Trajectory.VALIDATED
                for message in self.trajectories
            ),
            "dynamic_replan_status_count": sum(
                status.request_id == STOPPING_REQUEST_ID
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

    def test_braking_and_stop_remain_safe(self):
        expected = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        bag_start_ns, bag_end_ns, bag_message_count = read_bag_time_bounds(BAG_URI)
        self.assertEqual(bag_start_ns, expected["bag_start_ns"])
        self.assertEqual(bag_end_ns, expected["bag_end_ns"])
        self.assertEqual(bag_message_count, len(expected["events"]))

        try:
            self.wait_until(
                lambda: bool(self.result_at_stamp(BRAKING_STAMP_NS)),
                detail="braking-state planning result",
            )
            braking_result = self.result_at_stamp(BRAKING_STAMP_NS)[-1]
            self.assertEqual(braking_result.status, PlanningResult.SUCCESS)
            self.assertTrue(braking_result.has_trajectory)
            self.assertEqual(braking_result.risk_report.risk_level, braking_result.risk_report.LOW)

            self.wait_until(
                lambda: bool(self.result_at_stamp(STOPPED_STAMP_NS)),
                detail="stopped-state planning result",
            )
            stopped_result = self.result_at_stamp(STOPPED_STAMP_NS)[-1]
            self.assertEqual(stopped_result.status, PlanningResult.SUCCESS)
            self.assertTrue(stopped_result.has_trajectory)
            self.assertEqual(stopped_result.risk_report.risk_level, stopped_result.risk_report.LOW)
            self.assertEqual(stopped_result.risk_report.dynamic_risk, 0.0)
            self.assertEqual(
                stopped_result.risk_report.worst_obstacle_id,
                STOPPING_TRACK_ID,
            )
            self.assertTrue(stopped_result.risk_report.dynamic_information_available)
            self.assertFalse(stopped_result.risk_report.dynamic_information_stale)
            self.assertGreaterEqual(
                sum(
                    status.request_id == STOPPING_REQUEST_ID
                    and status.replan_trigger == PlannerStatus.DYNAMIC_OBSTACLE_UPDATED
                    for status in self.statuses
                ),
                2,
            )
            self.assertTrue(self.trajectories)
            self.assertTrue(
                all(
                    message.validation_state == Trajectory.VALIDATED
                    for message in self.trajectories
                )
            )
            self.assertFalse(
                any(state.active and state.latched for state in self.emergency_states)
            )
        finally:
            self.write_report(bag_start_ns, bag_end_ns, bag_message_count)


@launch_testing.post_shutdown_test()
class TestStoppingRosbag2ReplayShutdown(unittest.TestCase):
    def test_bag_play_exited_successfully(self, proc_info, bag_play):
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_play)
