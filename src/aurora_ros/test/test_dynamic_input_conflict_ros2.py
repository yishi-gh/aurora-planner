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
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
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
        name="aurora_planner_node_input_conflict",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_dynamic_input_conflict_ros2_test",
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
                "risk.max_prediction_age": 5.0,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [planner_node, launch_testing.actions.ReadyToTest()]
        ),
        {"planner": planner_node},
    )


class TestDynamicInputConflictRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_dynamic_input_conflict_observer")
        reliable_qos = QoSProfile(
            depth=10,
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
        self.track_publisher = self.node.create_publisher(
            DynamicObstacleTrackArray, "/aurora/dynamic_obstacle_tracks", reliable_qos
        )
        self.detection_publisher = self.node.create_publisher(
            UnassociatedObstacleDetectionArray,
            "/aurora/dynamic_obstacle_detections",
            sensor_qos,
        )
        self.pointcloud_publisher = self.node.create_publisher(PointCloud2, "/points", 10)
        self.results = []
        self.trajectories = []
        self.result_subscription = self.node.create_subscription(
            PlanningResult, "/aurora/planning_result", self.results.append, reliable_qos
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory, "/aurora/trajectory", self.trajectories.append, reliable_qos
        )

    def tearDown(self):
        self.node.destroy_subscription(self.result_subscription)
        self.node.destroy_subscription(self.trajectory_subscription)
        self.node.destroy_publisher(self.request_publisher)
        self.node.destroy_publisher(self.track_publisher)
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
                and self.node.count_subscribers("/aurora/dynamic_obstacle_tracks") > 0
                and self.node.count_subscribers(
                    "/aurora/dynamic_obstacle_detections"
                )
                > 0
                and self.node.count_publishers("/aurora/planning_result") > 0
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("dynamic input interfaces did not become discoverable")

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
    def make_external_empty(stamp):
        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    @staticmethod
    def make_internal_empty(stamp):
        batch = UnassociatedObstacleDetectionArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    def wait_for_result(self, request_id, status=None, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id and (
                    status is None or result.status == status
                ):
                    return result
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("timed out waiting for conflict test result")

    def test_fresh_external_and_internal_sources_are_rejected(self):
        self.wait_for_interfaces()

        seed_stamp = self.node.get_clock().now().to_msg()
        seed_request = self.make_request(949, seed_stamp)
        seed_request.global_reference.clear()
        for _ in range(5):
            self.request_publisher.publish(seed_request)
            self.spin_for(0.05)

        cloud_stamp = self.node.get_clock().now().to_msg()
        cloud = self.make_cloud(cloud_stamp)
        for _ in range(5):
            self.pointcloud_publisher.publish(cloud)
            self.spin_for(0.05)

        external_stamp = self.node.get_clock().now().to_msg()
        external_batch = self.make_external_empty(external_stamp)
        for _ in range(5):
            self.track_publisher.publish(external_batch)
            self.spin_for(0.05)
        request_stamp = self.node.get_clock().now().to_msg()
        request = self.make_request(950, request_stamp)
        for _ in range(5):
            self.request_publisher.publish(request)
            if any(
                item.request_id == 950 and item.status == PlanningResult.SUCCESS
                for item in self.results
            ):
                break
            self.spin_for(0.1)
        initial = self.wait_for_result(950, PlanningResult.SUCCESS)
        self.assertTrue(initial.has_trajectory)
        initial_trajectory_count = len(self.trajectories)

        internal_stamp = self.node.get_clock().now().to_msg()
        self.detection_publisher.publish(self.make_internal_empty(internal_stamp))
        conflict = self.wait_for_result(950, PlanningResult.VALIDATION_FAILED)

        self.assertFalse(conflict.has_trajectory)
        self.assertTrue(conflict.risk_report.dynamic_information_stale)
        self.assertEqual(
            conflict.safety_report.status,
            conflict.safety_report.INFORMATION_STALE,
        )
        self.assertIn("input mixing", conflict.detail)
        self.assertEqual(len(self.trajectories), initial_trajectory_count)


@launch_testing.post_shutdown_test()
class TestDynamicInputConflictRos2Shutdown(unittest.TestCase):
    def test_planner_exited_successfully(self, proc_info, planner):
        launch_testing.asserts.assertExitCodes(proc_info, process=planner)
