#!/usr/bin/env python3

import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from aurora_msgs.msg import (
    EmergencyStopState,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
    UnassociatedObstacleDetection,
    UnassociatedObstacleDetectionArray,
)
from geometry_msgs.msg import Point
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_planner_node_internal_lost",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_internal_detection_lost_ros2_test",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "dynamic_input_mode": "internal_detections",
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
                "risk.max_prediction_age": 2.0,
                "risk.stale_hold_duration": 0.3,
                "tracking.lost_after": 0.15,
                "tracking.deleted_after": 2.0,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [planner_node, launch_testing.actions.ReadyToTest()]
        ),
        {"planner": planner_node},
    )


class TestInternalDetectionLostRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_internal_detection_lost_observer")
        reliable_qos = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.request_publisher = self.node.create_publisher(
            PlanningRequest, "/aurora/planning_request", reliable_qos
        )
        self.detection_publisher = self.node.create_publisher(
            UnassociatedObstacleDetectionArray,
            "/aurora/dynamic_obstacle_detections",
            sensor_qos,
        )
        self.pointcloud_publisher = self.node.create_publisher(PointCloud2, "/points", 10)
        self.results = []
        self.trajectories = []
        self.emergency_states = []
        self.result_subscription = self.node.create_subscription(
            PlanningResult, "/aurora/planning_result", self.results.append, reliable_qos
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory, "/aurora/trajectory", self.trajectories.append, reliable_qos
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
        self.node.destroy_subscription(self.emergency_subscription)
        self.node.destroy_publisher(self.request_publisher)
        self.node.destroy_publisher(self.detection_publisher)
        self.node.destroy_publisher(self.pointcloud_publisher)
        self.node.destroy_node()

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def wait_for_interfaces(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if (
                self.node.count_subscribers("/aurora/planning_request") > 0
                and self.node.count_subscribers(
                    "/aurora/dynamic_obstacle_detections"
                )
                > 0
                and self.node.count_publishers("/aurora/planning_result") > 0
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("internal lost interfaces did not become discoverable")

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
            request.global_reference.append(reference)
        return request

    @staticmethod
    def make_cloud(stamp):
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
        cloud.data = struct.pack("<fff", 10.0, 0.0, 1.0)
        return cloud

    @staticmethod
    def make_detection_batch(stamp):
        detection = UnassociatedObstacleDetection()
        detection.header.stamp = stamp
        detection.header.frame_id = "map"
        # Keep this track outside the conservative long-horizon prediction
        # envelope. The test is exercising LOST/information-stale handling,
        # not dynamic collision handling.
        detection.position = Point(x=100.0, y=100.0, z=100.0)
        detection.has_velocity = True
        detection.velocity.x = 0.0
        detection.has_shape = True
        detection.shape_type = UnassociatedObstacleDetection.SPHERE
        detection.radius = 0.2
        batch = UnassociatedObstacleDetectionArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        batch.detections.append(detection)
        return batch

    @staticmethod
    def make_empty_batch(stamp):
        batch = UnassociatedObstacleDetectionArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    def wait_for_result(self, request_id, status, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id and result.status == status:
                    return result
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail(
            "timed out waiting for lost-state result: %s"
            % [
                (item.request_id, item.status, item.detail)
                for item in self.results
                if item.request_id == request_id
            ]
        )

    def test_lost_internal_track_enters_stale_and_emergency_stop(self):
        self.wait_for_interfaces()

        seed_stamp = self.node.get_clock().now().to_msg()
        seed_request = self.make_request(969, seed_stamp)
        seed_request.global_reference.clear()
        for _ in range(5):
            self.request_publisher.publish(seed_request)
            self.spin_for(0.05)

        cloud_stamp = self.node.get_clock().now().to_msg()
        cloud = self.make_cloud(cloud_stamp)
        for _ in range(5):
            self.pointcloud_publisher.publish(cloud)
            self.spin_for(0.05)

        first_stamp = self.node.get_clock().now().to_msg()
        first_batch = self.make_empty_batch(first_stamp)
        for _ in range(5):
            self.detection_publisher.publish(first_batch)
            self.spin_for(0.05)
        first_request_stamp = self.node.get_clock().now().to_msg()
        first_request = self.make_request(970, first_request_stamp)
        for _ in range(5):
            self.request_publisher.publish(first_request)
            if any(
                item.request_id == 970 and item.status == PlanningResult.SUCCESS
                for item in self.results
            ):
                break
            self.spin_for(0.1)
        initial = self.wait_for_result(970, PlanningResult.SUCCESS)
        self.assertTrue(initial.has_trajectory)
        initial_trajectory_count = len(self.trajectories)

        observed_stamp = self.node.get_clock().now().to_msg()
        self.detection_publisher.publish(self.make_detection_batch(observed_stamp))
        self.spin_for(0.2)
        confirmed_stamp = self.node.get_clock().now().to_msg()
        self.detection_publisher.publish(self.make_detection_batch(confirmed_stamp))
        self.spin_for(0.2)

        self.spin_for(0.3)
        trajectories_before_lost = len(self.trajectories)
        lost_stamp = self.node.get_clock().now().to_msg()
        self.detection_publisher.publish(self.make_empty_batch(lost_stamp))
        deadline = time.monotonic() + 15.0
        stale = None
        while time.monotonic() < deadline:
            stale = next(
                (
                    item
                    for item in self.results
                    if item.request_id == 970
                    and item.status == PlanningResult.VALIDATION_FAILED
                    and item.risk_report.dynamic_information_stale
                    and "lost threshold" in item.detail
                ),
                None,
            )
            if stale is not None:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIsNotNone(stale, "lost state did not produce a stale result")
        self.assertFalse(stale.has_trajectory)
        self.assertTrue(stale.risk_report.dynamic_information_stale)
        self.assertEqual(
            stale.safety_report.status,
            stale.safety_report.INFORMATION_STALE,
        )
        self.assertIn("lost threshold", stale.detail)
        self.assertEqual(len(self.trajectories), trajectories_before_lost)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if any(
                state.active
                and state.latched
                and state.reason == EmergencyStopState.INFORMATION_STALE
                for state in self.emergency_states
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("lost internal track did not latch information-stale stop")


@launch_testing.post_shutdown_test()
class TestInternalDetectionLostRos2Shutdown(unittest.TestCase):
    def test_planner_exited_successfully(self, proc_info, planner):
        launch_testing.asserts.assertExitCodes(proc_info, process=planner)
