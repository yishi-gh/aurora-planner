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
from aurora_msgs.srv import SetEmergencyStop
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools" / "rosbag2"))
from generate_information_stale_and_occlusion import (  # noqa: E402
    STALE_RECOVERY_REQUEST_ID,
    STALE_REQUEST_ID,
    generate_prediction_stale,
)
from generate_static_baseline import (  # noqa: E402
    make_dynamic_heartbeat,
    make_request,
)


SCENARIO_DIR = Path(tempfile.mkdtemp(prefix="aurora_prediction_stale_", dir="/tmp"))
BAG_URI = SCENARIO_DIR / "prediction_information_stale"
MANIFEST_PATH = generate_prediction_stale(BAG_URI)


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
        name="aurora_planner_node_prediction_stale_bag",
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


class TestPredictionInformationStaleRosbag2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_prediction_stale_rosbag2_observer")
        reliable_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.request_publisher = self.node.create_publisher(
            type(make_request(0, 0, False)), "/aurora/planning_request", reliable_qos
        )
        self.dynamic_publisher = self.node.create_publisher(
            type(make_dynamic_heartbeat(0)),
            "/aurora/dynamic_obstacle_tracks",
            reliable_qos,
        )
        self.emergency_client = self.node.create_client(
            SetEmergencyStop, "/aurora/set_emergency_stop"
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
        self.node.destroy_publisher(self.request_publisher)
        self.node.destroy_publisher(self.dynamic_publisher)
        self.node.destroy_client(self.emergency_client)
        self.node.destroy_node()

    def wait_until(self, predicate, timeout_sec=20.0, detail="condition"):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for %s" % detail)

    def wait_for_result(self, request_id, status=None, minimum_index=0):
        def find_result():
            return next(
                (
                    result
                    for result in self.results[minimum_index:]
                    if result.request_id == request_id
                    and (status is None or result.status == status)
                ),
                None,
            )

        self.wait_until(lambda: find_result() is not None, detail="planning result")
        return find_result()

    def call_reset(self):
        self.assertTrue(self.emergency_client.wait_for_service(timeout_sec=5.0))
        request = SetEmergencyStop.Request()
        request.engage = False
        request.reason = "prediction stale recovery test reset"
        future = self.emergency_client.call_async(request)
        self.wait_until(lambda: future.done(), timeout_sec=5.0, detail="reset response")
        response = future.result()
        self.assertTrue(response.accepted)
        self.assertFalse(response.latched)

    def publish_until_result(self, request):
        before = len(self.results)
        for _ in range(5):
            self.request_publisher.publish(request)
            self.wait_until(
                lambda: len(self.results) > before,
                timeout_sec=2.0,
                detail="planning result after request",
            )
            result = next(
                (item for item in self.results[before:] if item.request_id == request.request_id),
                None,
            )
            if result is not None:
                return result
        self.fail("no result for request %d" % request.request_id)

    def test_stale_hold_latch_and_explicit_recovery(self):
        expected = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        bag_start_ns, bag_end_ns, bag_message_count = read_bag_time_bounds(BAG_URI)
        self.assertEqual(bag_start_ns, expected["bag_start_ns"])
        self.assertEqual(bag_end_ns, expected["bag_end_ns"])
        self.assertEqual(bag_message_count, len(expected["events"]))

        initial = self.wait_for_result(STALE_REQUEST_ID, PlanningResult.SUCCESS)
        self.assertTrue(initial.has_trajectory)
        self.assertGreater(len(initial.trajectory.segments), 0)
        self.wait_until(
            lambda: len(self.trajectories) >= 1,
            detail="initial validated trajectory publication",
        )
        initial_trajectory_count = len(self.trajectories)

        stale = self.wait_for_result(
            STALE_REQUEST_ID, PlanningResult.VALIDATION_FAILED
        )
        self.assertFalse(stale.has_trajectory)
        self.assertTrue(stale.risk_report.dynamic_information_available)
        self.assertTrue(stale.risk_report.dynamic_information_stale)
        self.assertGreaterEqual(stale.risk_report.information_age, 0.5)
        self.assertEqual(
            stale.safety_report.status,
            stale.safety_report.INFORMATION_STALE,
        )

        self.wait_until(
            lambda: any(
                state.active
                and state.latched
                and state.reason == EmergencyStopState.INFORMATION_STALE
                for state in self.emergency_states
            ),
            detail="stale-information emergency stop",
        )
        self.assertEqual(len(self.trajectories), initial_trajectory_count)
        self.assertTrue(
            all(item.validation_state == Trajectory.VALIDATED for item in self.trajectories)
        )

        self.call_reset()
        self.wait_until(
            lambda: any(not state.active and not state.latched for state in self.emergency_states),
            detail="inactive reset state",
        )

        recovery_batch = make_dynamic_heartbeat(bag_end_ns)
        for _ in range(5):
            self.dynamic_publisher.publish(recovery_batch)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        recovery_request = make_request(
            bag_end_ns, STALE_RECOVERY_REQUEST_ID, include_reference=True
        )
        recovery_result = self.publish_until_result(recovery_request)
        self.assertEqual(recovery_result.status, PlanningResult.SUCCESS)
        self.assertTrue(recovery_result.has_trajectory)
        self.assertFalse(recovery_result.risk_report.dynamic_information_stale)
        self.wait_until(
            lambda: len(self.trajectories) >= initial_trajectory_count + 1,
            detail="recovery validated trajectory publication",
        )
        self.assertGreaterEqual(len(self.trajectories), initial_trajectory_count + 1)


@launch_testing.post_shutdown_test()
class TestPredictionInformationStaleRosbag2Shutdown(unittest.TestCase):
    def test_bag_play_exited_successfully(self, proc_info, bag_play):
        launch_testing.asserts.assertExitCodes(proc_info, process=bag_play)
