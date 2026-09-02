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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from aurora_msgs.msg import (
    DynamicObstacleTrack,
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    EmergencyStopState,
    PlannerStatus,
    Trajectory,
)
from aurora_msgs.srv import SetEmergencyStop
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_planner_launch_test",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
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

    return (
        launch.LaunchDescription(
            [planner_node, launch_testing.actions.ReadyToTest()]
        ),
        {"planner": planner_node},
    )


class TestPlannerNodeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_planner_integration_test")
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.request_publisher = self.node.create_publisher(
            PlanningRequest, "/aurora/planning_request", reliable_qos
        )
        self.track_publisher = self.node.create_publisher(
            DynamicObstacleTrackArray, "/aurora/dynamic_obstacle_tracks", reliable_qos
        )
        self.emergency_client = self.node.create_client(
            SetEmergencyStop, "/aurora/set_emergency_stop"
        )
        self.pointcloud_publisher = self.node.create_publisher(
            PointCloud2, "/points", 10
        )
        self.results = []
        self.trajectories = []
        self.emergency_states = []
        self.statuses = []
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
        self.emergency_subscription = self.node.create_subscription(
            EmergencyStopState,
            "/aurora/emergency_stop_state",
            self.emergency_states.append,
            reliable_qos,
        )
        self.status_subscription = self.node.create_subscription(
            PlannerStatus,
            "/aurora/planner_status",
            self.statuses.append,
            reliable_qos,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.result_subscription)
        self.node.destroy_subscription(self.trajectory_subscription)
        self.node.destroy_subscription(self.emergency_subscription)
        self.node.destroy_subscription(self.status_subscription)
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
                and self.node.count_publishers("/aurora/planning_result") > 0
                and self.node.count_publishers("/aurora/trajectory") > 0
                and self.node.count_publishers("/aurora/planner_status") > 0
                and self.emergency_client.wait_for_service(timeout_sec=0.0)
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("planner ROS 2 interfaces did not become discoverable")

    def now_message(self):
        return self.node.get_clock().now().to_msg()

    def call_emergency_stop(self, engage, reason):
        request = SetEmergencyStop.Request()
        request.engage = engage
        request.reason = reason
        future = self.emergency_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.assertTrue(future.done(), "emergency-stop service call timed out")
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.accepted)
        return response

    @staticmethod
    def make_request(request_id, stamp, include_reference=True):
        request = PlanningRequest()
        request.header.stamp = stamp
        request.header.frame_id = "map"
        request.request_id = request_id
        request.vehicle_state.header.stamp = stamp
        request.vehicle_state.header.frame_id = "map"
        request.vehicle_state.position = Point(x=-4.0, y=0.0, z=1.0)
        if include_reference:
            for x in (-4.0, 4.0):
                reference = GlobalReferencePoint()
                reference.position = Point(x=x, y=0.0, z=1.0)
                reference.has_time = False
                request.global_reference.append(reference)
        return request

    @staticmethod
    def make_free_space_cloud(stamp):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "map"
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
        # The ray starts at the vehicle state (-4, 0, 1) and marks the
        # straight planning corridor as free. The endpoint is outside the
        # local goal horizon and is harmlessly marked occupied.
        cloud.data = struct.pack("<fff", 10.0, 0.0, 1.0)
        return cloud

    @staticmethod
    def make_empty_track_batch(stamp):
        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    @staticmethod
    def make_crossing_track(stamp):
        track = DynamicObstacleTrack()
        track.header.stamp = stamp
        track.header.frame_id = "map"
        track.track_id = 7
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

        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        batch.tracks.append(track)
        return batch

    def wait_for_result(self, request_id, timeout_sec=15.0, minimum_index=0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results[minimum_index:]:
                if result.request_id == request_id:
                    return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for planning result %d" % request_id)

    def publish_request_and_wait(self, request):
        # Reliable QoS and repeated publication make discovery timing explicit
        # without making the production node depend on a latched request.
        for _ in range(5):
            self.request_publisher.publish(request)
            result = self.find_result(request.request_id)
            if result is not None:
                return result
            self.spin_for(0.1)
        return self.wait_for_result(request.request_id)

    def find_result(self, request_id):
        for result in self.results:
            if result.request_id == request_id:
                return result
        return None

    def test_dynamic_crossing_is_gated_end_to_end(self):
        self.wait_for_interfaces()

        # Seed the latest vehicle state used as the point-cloud ray origin.
        seed_stamp = self.now_message()
        self.request_publisher.publish(
            self.make_request(90, seed_stamp, include_reference=False)
        )
        self.spin_for(0.3)

        cloud_stamp = self.now_message()
        cloud = self.make_free_space_cloud(cloud_stamp)
        for _ in range(5):
            self.pointcloud_publisher.publish(cloud)
            self.spin_for(0.1)

        safe_stamp = self.now_message()
        self.track_publisher.publish(self.make_empty_track_batch(safe_stamp))
        self.spin_for(0.2)
        safe_result = self.publish_request_and_wait(
            self.make_request(101, safe_stamp)
        )
        self.assertEqual(safe_result.status, PlanningResult.SUCCESS)
        self.assertTrue(safe_result.has_trajectory)
        self.assertGreater(len(safe_result.trajectory.segments), 0)
        self.assertGreaterEqual(len(self.trajectories), 1)
        self.assertEqual(self.trajectories[-1].validation_state, Trajectory.VALIDATED)
        self.assertEqual(safe_result.risk_report.risk_level, safe_result.risk_report.LOW)

        # A newer, non-colliding dynamic snapshot is still a planning event.
        # The latest complete 3D aircraft request is reused and its request_id
        # remains the result correlation key.
        self.trajectories.clear()
        heartbeat_stamp = self.now_message()
        result_count_before_heartbeat = len(self.results)
        self.track_publisher.publish(self.make_empty_track_batch(heartbeat_stamp))
        heartbeat_result = self.wait_for_result(
            101, minimum_index=result_count_before_heartbeat
        )
        self.spin_for(0.2)
        self.assertEqual(heartbeat_result.status, PlanningResult.SUCCESS)
        self.assertTrue(heartbeat_result.has_trajectory)
        self.assertGreater(len(heartbeat_result.trajectory.segments), 0)
        self.assertGreaterEqual(len(self.trajectories), 1)
        self.assertTrue(
            any(
                status.request_id == 101
                and status.replan_trigger == PlannerStatus.DYNAMIC_OBSTACLE_UPDATED
                for status in self.statuses
            ),
            "dynamic obstacle update did not trigger an explicit replan: %s"
            % [
                (status.request_id, status.planner_state, status.planner_action,
                 status.replan_trigger, status.detail)
                for status in self.statuses
            ],
        )

        # Repeating the same snapshot timestamp must not retrigger planning.
        result_count_after_heartbeat = len(self.results)
        self.track_publisher.publish(self.make_empty_track_batch(heartbeat_stamp))
        self.spin_for(0.3)
        self.assertEqual(len(self.results), result_count_after_heartbeat)

        self.trajectories.clear()
        crossing_stamp = self.now_message()
        result_count_before_dynamic_update = len(self.results)
        self.track_publisher.publish(self.make_crossing_track(crossing_stamp))
        self.spin_for(0.2)
        collision_result = self.wait_for_result(
            101, minimum_index=result_count_before_dynamic_update
        )
        self.assertEqual(collision_result.status, PlanningResult.VALIDATION_FAILED)
        self.assertFalse(collision_result.has_trajectory)
        self.assertEqual(len(collision_result.trajectory.segments), 0)
        self.assertEqual(
            collision_result.risk_report.risk_level,
            collision_result.risk_report.HIGH,
        )
        self.assertTrue(collision_result.risk_report.dynamic_information_available)
        self.assertFalse(collision_result.risk_report.dynamic_information_stale)
        self.assertEqual(collision_result.risk_report.worst_obstacle_id, 7)
        self.assertEqual(collision_result.risk_report.dynamic_risk, 1.0)
        self.assertEqual(len(self.trajectories), 0)

        self.assertTrue(
            any(state.active and state.latched for state in self.emergency_states),
            "dynamic collision did not latch the fail-safe emergency state",
        )

    def test_emergency_stop_service_requires_new_request_after_reset(self):
        self.wait_for_interfaces()

        # Start from a known state even when this test follows the dynamic
        # collision test in the same launched planner process.
        reset_response = self.call_emergency_stop(False, "test initial reset")
        self.assertFalse(reset_response.active)
        self.assertFalse(reset_response.latched)
        self.spin_for(0.3)

        seed_stamp = self.now_message()
        self.request_publisher.publish(
            self.make_request(190, seed_stamp, include_reference=False)
        )
        self.spin_for(0.2)
        cloud_stamp = self.now_message()
        cloud = self.make_free_space_cloud(cloud_stamp)
        for _ in range(5):
            self.pointcloud_publisher.publish(cloud)
            self.spin_for(0.1)

        engage_response = self.call_emergency_stop(True, "operator test stop")
        self.assertTrue(engage_response.active)
        self.assertTrue(engage_response.latched)

        # Both a request and a newer dynamic update arrive while latched. They
        # must remain unexecuted and must be discarded by reset.
        old_request_stamp = self.now_message()
        old_request = self.make_request(201, old_request_stamp)
        self.request_publisher.publish(old_request)
        self.spin_for(0.2)
        self.track_publisher.publish(self.make_empty_track_batch(self.now_message()))
        self.spin_for(0.4)
        self.assertFalse(any(result.request_id == 201 for result in self.results))

        reset_response = self.call_emergency_stop(False, "operator test reset")
        self.assertFalse(reset_response.active)
        self.assertFalse(reset_response.latched)
        self.spin_for(0.5)
        self.assertFalse(
            any(result.request_id == 201 for result in self.results),
            "a request queued during emergency stop was executed after reset",
        )
        self.assertTrue(
            any(not state.active and not state.latched for state in self.emergency_states),
            "reset did not publish an inactive emergency state",
        )

        # No request is restored from the cleared latest-request slot. A new
        # explicit request is required to start planning again.
        new_stamp = self.now_message()
        self.track_publisher.publish(self.make_empty_track_batch(new_stamp))
        self.spin_for(0.2)
        result = self.publish_request_and_wait(self.make_request(202, new_stamp))
        self.assertEqual(result.status, PlanningResult.SUCCESS)
        self.assertTrue(result.has_trajectory)
