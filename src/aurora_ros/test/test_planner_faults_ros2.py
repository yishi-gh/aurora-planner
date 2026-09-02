#!/usr/bin/env python3

import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from aurora_msgs.msg import (
    DynamicObstacleTrackArray,
    EmergencyStopState,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
)
from aurora_msgs.srv import SetEmergencyStop
from geometry_msgs.msg import Point
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node_faults",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_planner_faults_ros2_test",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "map.reject_unknown": False,
                "map.require_fresh_observation": True,
                "map.max_observation_age": 0.35,
                "planning.local_horizon": 6.0,
                "planning.resampling_spacing": 0.5,
                "planning.resampling_minimum_points": 9,
                "planning.optimizer_interval": 0.5,
                "planning.optimizer_max_iterations": 80,
                "planning.optimizer_samples_per_span": 8,
                "planning.validation_samples_per_span": 16,
                "risk.sample_interval": 0.1,
                "risk.max_prediction_age": 10.0,
                "risk.stale_hold_duration": 0.2,
                "risk.information_watchdog_rate_hz": 20.0,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [planner_node, launch_testing.actions.ReadyToTest()]
        ),
        {"planner": planner_node},
    )


class TestPlannerFaultsRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_planner_faults_observer")
        reliable_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.request_publisher = self.node.create_publisher(
            PlanningRequest, "/aurora/planning_request", reliable_qos
        )
        self.track_publisher = self.node.create_publisher(
            DynamicObstacleTrackArray, "/aurora/dynamic_obstacle_tracks", reliable_qos
        )
        self.pointcloud_publisher = self.node.create_publisher(
            PointCloud2, "/points", 10
        )
        self.result_subscription = self.node.create_subscription(
            PlanningResult, "/aurora/planning_result", self._record_result, reliable_qos
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory, "/aurora/trajectory", self._record_trajectory, reliable_qos
        )
        self.emergency_subscription = self.node.create_subscription(
            EmergencyStopState,
            "/aurora/emergency_stop_state",
            self._record_emergency,
            reliable_qos,
        )
        self.results = []
        self.trajectories = []
        self.emergency_states = []
        self.emergency_client = self.node.create_client(
            SetEmergencyStop, "/aurora/set_emergency_stop"
        )

    def _record_result(self, message):
        self.results.append(message)

    def _record_trajectory(self, message):
        self.trajectories.append(message)

    def _record_emergency(self, message):
        self.emergency_states.append(message)

    def tearDown(self):
        self.node.destroy_subscription(self.result_subscription)
        self.node.destroy_subscription(self.trajectory_subscription)
        self.node.destroy_subscription(self.emergency_subscription)
        self.node.destroy_publisher(self.request_publisher)
        self.node.destroy_publisher(self.track_publisher)
        self.node.destroy_publisher(self.pointcloud_publisher)
        self.node.destroy_client(self.emergency_client)
        self.node.destroy_node()

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))

    def wait_for_interfaces(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if (
                self.node.count_subscribers("/aurora/planning_request") > 0
                and self.node.count_subscribers("/aurora/dynamic_obstacle_tracks") > 0
                and self.node.count_subscribers("/points") > 0
                and self.node.count_publishers("/aurora/planning_result") > 0
                and self.node.count_publishers("/aurora/trajectory") > 0
                and self.emergency_client.wait_for_service(timeout_sec=0.0)
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("planner fault-test interfaces did not become discoverable")

    def now_message(self):
        return self.node.get_clock().now().to_msg()

    @staticmethod
    def make_request(request_id, stamp):
        request = PlanningRequest()
        request.header.stamp = stamp
        request.header.frame_id = "map"
        request.request_id = request_id
        request.vehicle_state.header.stamp = stamp
        request.vehicle_state.header.frame_id = "map"
        request.vehicle_state.position = Point(x=-4.0, y=0.0, z=1.0)
        for x in (-4.0, 4.0):
            reference = GlobalReferencePoint()
            reference.position = Point(x=x, y=0.0, z=1.0)
            reference.has_time = False
            request.global_reference.append(reference)
        return request

    @staticmethod
    def make_cloud(stamp, frame_id="map"):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = frame_id
        cloud.height = 1
        cloud.width = 1
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.data = struct.pack("<fff", 10.0, 0.0, 1.0)
        return cloud

    @staticmethod
    def make_empty_tracks(stamp):
        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    def wait_for_result(self, request_id, timeout_sec=10.0, minimum_index=0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results[minimum_index:]:
                if result.request_id == request_id:
                    return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for planning result %d" % request_id)

    def publish_request_and_wait(self, request):
        before = len(self.results)
        self.request_publisher.publish(request)
        return self.wait_for_result(request.request_id, minimum_index=before)

    def publish_valid_map(self, stamp):
        cloud = self.make_cloud(stamp)
        for _ in range(3):
            self.pointcloud_publisher.publish(cloud)
            self.spin_for(0.05)

    def call_reset(self):
        request = SetEmergencyStop.Request()
        request.engage = False
        request.reason = "fault regression reset"
        future = self.emergency_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), "reset service call timed out")
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.accepted)
        self.assertFalse(response.active)
        self.assertFalse(response.latched)

    def test_map_tf_time_and_recovery_faults_close_the_loop(self):
        self.wait_for_interfaces()

        # A cloud with no available transform must not make the map appear fresh.
        bad_cloud_stamp = self.now_message()
        self.pointcloud_publisher.publish(self.make_cloud(bad_cloud_stamp, "missing_sensor_frame"))
        self.spin_for(0.2)
        bad_map_result = self.publish_request_and_wait(
            self.make_request(500, self.now_message())
        )
        self.assertEqual(bad_map_result.status, PlanningResult.VALIDATION_FAILED)
        self.assertFalse(bad_map_result.has_trajectory)
        self.assertEqual(
            bad_map_result.safety_report.status,
            bad_map_result.safety_report.INFORMATION_STALE,
        )
        self.assertEqual(bad_map_result.risk_report.static_risk, 1.0)
        self.assertIn("point-cloud", bad_map_result.detail)

        # A valid observation plus a dynamic heartbeat authorizes a normal request.
        valid_stamp = self.now_message()
        self.publish_valid_map(valid_stamp)
        self.track_publisher.publish(self.make_empty_tracks(valid_stamp))
        valid_result = self.publish_request_and_wait(
            self.make_request(501, self.now_message())
        )
        self.assertEqual(valid_result.status, PlanningResult.SUCCESS, valid_result.detail)
        self.assertTrue(valid_result.has_trajectory)
        self.assertEqual(valid_result.trajectory.validation_state, Trajectory.VALIDATED)

        # Older planning time must be rejected before it can replace the accepted state.
        rollback_stamp = valid_stamp
        rollback_stamp.sec -= 1
        rollback_result = self.publish_request_and_wait(
            self.make_request(502, rollback_stamp)
        )
        self.assertEqual(rollback_result.status, PlanningResult.INVALID_REQUEST)
        self.assertFalse(rollback_result.has_trajectory)
        self.assertIn("backwards", rollback_result.detail)

        # Keep dynamic information fresh while deliberately dropping point clouds.
        # The active validated trajectory may be held briefly, then the map fault
        # must latch the same fail-safe stop boundary as other safety information.
        for _ in range(5):
            self.track_publisher.publish(self.make_empty_tracks(self.now_message()))
            self.spin_for(0.08)
        self.spin_for(0.8)
        self.assertTrue(
            any(
                state.active
                and state.latched
                and state.reason == EmergencyStopState.INFORMATION_STALE
                for state in self.emergency_states
            ),
            "map dropout did not latch an information-stale emergency stop",
        )

        # Reset clears the old map timestamp. Recovery requires a fresh cloud,
        # a fresh dynamic snapshot, and a new explicit request.
        self.call_reset()
        recovered_stamp = self.now_message()
        self.publish_valid_map(recovered_stamp)
        self.track_publisher.publish(self.make_empty_tracks(recovered_stamp))
        recovered_result = self.publish_request_and_wait(
            self.make_request(503, self.now_message())
        )
        self.assertEqual(recovered_result.status, PlanningResult.SUCCESS, recovered_result.detail)
        self.assertTrue(recovered_result.has_trajectory)
