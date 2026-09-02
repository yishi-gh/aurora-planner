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
    DynamicObstacleTrack,
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
)
from geometry_msgs.msg import Point
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


PREFIX = "/aurora/soft_risk"


def parameters():
    return {
        "map.reject_unknown": False,
        "map.require_fresh_observation": False,
        "vehicle.radius": 0.2,
        "map.inflation_radius": 0.2,
        "planning.local_horizon": 6.0,
        "planning.resampling_spacing": 0.5,
        "planning.resampling_minimum_points": 9,
        "planning.optimizer_interval": 0.5,
        "planning.optimizer_max_iterations": 100,
        "planning.optimizer_samples_per_span": 8,
        "planning.optimizer_lambda_risk": 12.0,
        "planning.validation_samples_per_span": 16,
        "risk.enable_soft_cost": True,
        "risk.max_prediction_age": 0.5,
        "risk.warning_clearance": 0.5,
        "risk.sample_interval": 0.1,
        "prediction.max_horizon": 10.0,
        "prediction.process_noise_acceleration": 0.0,
        "topics.pointcloud": PREFIX + "/points",
        "topics.planning_request": PREFIX + "/request",
        "topics.trajectory": PREFIX + "/trajectory",
        "topics.planning_result": PREFIX + "/result",
        "topics.planner_status": PREFIX + "/status",
        "topics.emergency_state": PREFIX + "/emergency_state",
        "topics.dynamic_obstacle_tracks": PREFIX + "/tracks",
        "topics.dynamic_obstacle_detections": PREFIX + "/detections",
        "services.emergency_stop": PREFIX + "/emergency_stop",
    }


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_soft_risk_planner",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_soft_risk_ros2",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[parameters()],
    )
    return (
        launch.LaunchDescription(
            [planner_node, launch_testing.actions.ReadyToTest()]
        ),
        {"planner": planner_node},
    )


class TestSoftRiskRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_soft_risk_observer")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.request_publisher = self.node.create_publisher(
            PlanningRequest, PREFIX + "/request", qos
        )
        self.track_publisher = self.node.create_publisher(
            DynamicObstacleTrackArray, PREFIX + "/tracks", qos
        )
        self.cloud_publisher = self.node.create_publisher(
            PointCloud2, PREFIX + "/points", 10
        )
        self.results = []
        self.trajectories = []
        self.result_subscription = self.node.create_subscription(
            PlanningResult, PREFIX + "/result", self.results.append, qos
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory, PREFIX + "/trajectory", self.trajectories.append, qos
        )

    def tearDown(self):
        self.node.destroy_subscription(self.result_subscription)
        self.node.destroy_subscription(self.trajectory_subscription)
        self.node.destroy_publisher(self.request_publisher)
        self.node.destroy_publisher(self.track_publisher)
        self.node.destroy_publisher(self.cloud_publisher)
        self.node.destroy_node()

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))

    def wait_for_interfaces(self):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if (
                self.node.count_subscribers(PREFIX + "/request") > 0
                and self.node.count_subscribers(PREFIX + "/tracks") > 0
                and self.node.count_subscribers(PREFIX + "/points") > 0
                and self.node.count_publishers(PREFIX + "/result") > 0
                and self.node.count_publishers(PREFIX + "/trajectory") > 0
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("soft-risk planner interfaces did not become discoverable")

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
                request.global_reference.append(reference)
        return request

    @staticmethod
    def make_free_cloud(stamp):
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
        cloud.data = struct.pack("<fff", 4.0, 0.0, 1.0)
        return cloud

    @staticmethod
    def make_near_static_track(stamp):
        track = DynamicObstacleTrack()
        track.header.stamp = stamp
        track.header.frame_id = "map"
        track.track_id = 26
        track.pose.position = Point(x=0.0, y=-0.65, z=1.0)
        track.shape_type = DynamicObstacleTrack.SPHERE
        track.radius = 0.15
        track.existence_probability = 1.0
        track.prediction_model = DynamicObstacleTrack.CV
        track.has_state_covariance = True
        track.state_covariance = [0.0] * 36
        for index in (0, 7, 14):
            track.state_covariance[index] = 0.0001
        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        batch.tracks.append(track)
        return batch

    def wait_for_result(self, request_id, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id:
                    return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for soft-risk planning result")

    def test_soft_risk_moves_trajectory_and_keeps_hard_gate(self):
        self.wait_for_interfaces()
        seed_stamp = self.node.get_clock().now().to_msg()
        self.request_publisher.publish(
            self.make_request(2600, seed_stamp, include_reference=False)
        )
        self.spin_for(0.2)

        stamp = self.node.get_clock().now().to_msg()
        cloud = self.make_free_cloud(stamp)
        for _ in range(5):
            self.cloud_publisher.publish(cloud)
            self.spin_for(0.08)

        track_batch = self.make_near_static_track(stamp)
        for _ in range(5):
            self.track_publisher.publish(track_batch)
            self.spin_for(0.08)

        request = self.make_request(2601, stamp)
        for _ in range(5):
            self.request_publisher.publish(request)
            self.spin_for(0.08)
            if any(result.request_id == request.request_id for result in self.results):
                break
        result = self.wait_for_result(request.request_id)

        self.assertEqual(result.status, PlanningResult.SUCCESS, result.detail)
        self.assertTrue(result.has_trajectory)
        self.assertEqual(result.trajectory.validation_state, Trajectory.VALIDATED)
        self.assertTrue(result.risk_report.dynamic_information_available)
        self.assertLess(result.risk_report.dynamic_risk, 1.0)
        self.assertGreater(len(self.trajectories), 0)

        y_values = [
            point.y
            for segment in result.trajectory.segments
            for point in segment.control_points
        ]
        self.assertGreater(max(y_values), 0.02, "soft risk did not bend the 3D trajectory away")
