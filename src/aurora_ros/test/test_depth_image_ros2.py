#!/usr/bin/env python3

import struct
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from aurora_msgs.msg import (
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
)
from geometry_msgs.msg import Point
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_node = launch_ros.actions.Node(
        package="aurora_ros",
        executable="aurora_planner_node",
        name="aurora_depth_image_planner",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_depth_image_ros2_launch_test",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "depth.enabled": True,
                "topics.depth_image": "/aurora/depth/image",
                "topics.camera_info": "/aurora/depth/camera_info",
                "map.reject_unknown": False,
                "map.require_fresh_observation": True,
                "map.max_observation_age": 1.0,
                "planning.local_horizon": 6.0,
                "planning.resampling_spacing": 0.5,
                "planning.resampling_minimum_points": 9,
                "planning.optimizer_interval": 0.5,
                "planning.optimizer_max_iterations": 80,
                "planning.optimizer_samples_per_span": 8,
                "planning.validation_samples_per_span": 16,
                "risk.sample_interval": 0.1,
                "risk.max_prediction_age": 1.0,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [
                planner_node,
                launch.actions.TimerAction(
                    period=5.0,
                    actions=[
                        launch.actions.EmitEvent(
                            event=launch.events.Shutdown(
                                reason="depth image regression completed"
                            )
                        )
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"planner": planner_node},
    )


class TestDepthImageRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_depth_image_observer")
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
        self.camera_info_publisher = self.node.create_publisher(
            CameraInfo, "/aurora/depth/camera_info", sensor_qos
        )
        self.depth_publisher = self.node.create_publisher(
            Image, "/aurora/depth/image", sensor_qos
        )
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
        self.node.destroy_publisher(self.camera_info_publisher)
        self.node.destroy_publisher(self.depth_publisher)
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
                self.node.count_subscribers("/aurora/depth/camera_info") > 0
                and self.node.count_subscribers("/aurora/depth/image") > 0
                and self.node.count_subscribers("/aurora/planning_request") > 0
                and self.node.count_subscribers("/aurora/dynamic_obstacle_tracks") > 0
                and self.node.count_publishers("/aurora/planning_result") > 0
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("depth image interfaces did not become discoverable")

    def now_message(self):
        return self.node.get_clock().now().to_msg()

    @staticmethod
    def make_camera_info(stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "map"
        info.width = 2
        info.height = 1
        info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5
        return info

    @staticmethod
    def make_depth_image(stamp, valid=True):
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "map"
        image.height = 1
        image.width = 2
        image.encoding = "16UC1" if valid else "mono8"
        image.is_bigendian = False
        image.step = 4 if valid else 2
        image.data = struct.pack("<HH", 10000, 10000) if valid else b"\x00\x00"
        return image

    @staticmethod
    def make_request(request_id, stamp):
        request = PlanningRequest()
        request.header.stamp = stamp
        request.header.frame_id = "map"
        request.request_id = request_id
        request.vehicle_state.header.stamp = stamp
        request.vehicle_state.header.frame_id = "map"
        request.vehicle_state.position = Point(x=0.0, y=0.0, z=0.0)
        for z in (0.0, 4.0):
            reference = GlobalReferencePoint()
            reference.position = Point(x=0.0, y=0.0, z=z)
            request.global_reference.append(reference)
        return request

    @staticmethod
    def make_empty_tracks(stamp):
        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    def wait_for_result(self, request_id, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id:
                    return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for depth planning result %d" % request_id)

    def publish_request_and_wait(self, request):
        for _ in range(5):
            self.request_publisher.publish(request)
            self.spin_for(0.1)
            for result in self.results:
                if result.request_id == request.request_id:
                    return result
        return self.wait_for_result(request.request_id)

    def publish_sensor_pair(self, stamp, valid):
        camera_info = self.make_camera_info(stamp)
        image = self.make_depth_image(stamp, valid=valid)
        for _ in range(5):
            self.camera_info_publisher.publish(camera_info)
            self.depth_publisher.publish(image)
            self.spin_for(0.05)

    def test_invalid_depth_does_not_refresh_map_and_valid_depth_closes_loop(self):
        self.wait_for_interfaces()

        invalid_stamp = self.now_message()
        self.publish_sensor_pair(invalid_stamp, valid=False)
        self.track_publisher.publish(self.make_empty_tracks(invalid_stamp))
        invalid_result = self.publish_request_and_wait(
            self.make_request(700, self.now_message())
        )
        self.assertEqual(invalid_result.status, PlanningResult.VALIDATION_FAILED)
        self.assertFalse(invalid_result.has_trajectory)
        self.assertEqual(
            invalid_result.safety_report.status,
            invalid_result.safety_report.INFORMATION_STALE,
        )

        valid_stamp = self.now_message()
        self.publish_sensor_pair(valid_stamp, valid=True)
        self.track_publisher.publish(self.make_empty_tracks(valid_stamp))
        valid_result = self.publish_request_and_wait(
            self.make_request(701, self.now_message())
        )
        self.assertEqual(valid_result.status, PlanningResult.SUCCESS, valid_result.detail)
        self.assertTrue(valid_result.has_trajectory)
        self.assertEqual(valid_result.trajectory.validation_state, Trajectory.VALIDATED)
        self.assertGreaterEqual(len(self.trajectories), 1)
