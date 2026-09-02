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
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
)
from geometry_msgs.msg import Point
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


SCENARIOS = {
    "free": "/aurora/map_quality/free",
    "unknown": "/aurora/map_quality/unknown",
    "occupied": "/aurora/map_quality/occupied",
    "out_of_map": "/aurora/map_quality/out_of_map",
}


def node_parameters(prefix, **extra):
    params = {
        "map.reject_unknown": False,
        "map.require_fresh_observation": False,
        "risk.enable_map_quality": True,
        "risk.require_map_quality": True,
        "risk.max_prediction_age": 1.0,
        "planning.local_horizon": 6.0,
        "planning.resampling_spacing": 0.5,
        "planning.resampling_minimum_points": 9,
        "planning.optimizer_interval": 0.5,
        "planning.optimizer_max_iterations": 80,
        "planning.optimizer_samples_per_span": 8,
        "planning.validation_samples_per_span": 16,
        "risk.sample_interval": 0.1,
        "topics.pointcloud": prefix + "/points",
        "topics.planning_request": prefix + "/request",
        "topics.trajectory": prefix + "/trajectory",
        "topics.planning_result": prefix + "/result",
        "topics.planner_status": prefix + "/status",
        "topics.emergency_state": prefix + "/emergency_state",
        "topics.dynamic_obstacle_tracks": prefix + "/tracks",
        "topics.dynamic_obstacle_detections": prefix + "/detections",
        "services.emergency_stop": prefix + "/emergency_stop",
    }
    params.update(extra)
    return params


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    nodes = []
    for name, prefix in SCENARIOS.items():
        extra = {}
        if name == "occupied":
            extra.update(
                {
                    "map.origin_y": -0.5,
                    "map.dimensions_y": 1,
                    "map.origin_z": 0.75,
                    "map.dimensions_z": 1,
                    "map.inflation_radius": 0.0,
                    "planning.optimizer_clearance": 0.0,
                }
            )
        elif name == "out_of_map":
            extra.update(
                {
                    "map.origin_x": -5.0,
                    "map.dimensions_x": 20,
                    "planning.local_horizon": 100.0,
                }
            )
        nodes.append(
            launch_ros.actions.Node(
                package="aurora_ros",
                executable="aurora_planner_node",
                name="aurora_map_quality_" + name,
                output="screen",
                additional_env={
                    "ROS_LOG_DIR": "/tmp/aurora_map_quality_" + name,
                    "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
                },
                parameters=[node_parameters(prefix, **extra)],
            )
        )
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()]), {}


class TestMapQualityRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_map_quality_observer")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publishers = {}
        self.track_publishers = {}
        self.results = {name: [] for name in SCENARIOS}
        self.trajectories = {name: [] for name in SCENARIOS}
        for name, prefix in SCENARIOS.items():
            self.publishers[name] = self.node.create_publisher(
                PlanningRequest, prefix + "/request", qos
            )
            self.track_publishers[name] = self.node.create_publisher(
                DynamicObstacleTrackArray, prefix + "/tracks", qos
            )
            self.node.create_subscription(
                PlanningResult,
                prefix + "/result",
                self.results[name].append,
                qos,
            )
            self.node.create_subscription(
                Trajectory,
                prefix + "/trajectory",
                self.trajectories[name].append,
                qos,
            )
        self.cloud_publisher = self.node.create_publisher(
            PointCloud2, SCENARIOS["free"] + "/points", 10
        )

    def tearDown(self):
        for publisher in self.publishers.values():
            self.node.destroy_publisher(publisher)
        for publisher in self.track_publishers.values():
            self.node.destroy_publisher(publisher)
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
            ready = True
            for name, prefix in SCENARIOS.items():
                ready = ready and self.node.count_subscribers(prefix + "/request") > 0
                ready = ready and self.node.count_subscribers(prefix + "/tracks") > 0
                ready = ready and self.node.count_subscribers(prefix + "/points") > 0
                ready = ready and self.node.count_publishers(prefix + "/result") > 0
            if ready:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("map quality planner interfaces did not become discoverable")

    @staticmethod
    def make_request(
        request_id, stamp, goal_x=4.0, y=0.0, z=1.0, include_reference=True
    ):
        request = PlanningRequest()
        request.header.stamp = stamp
        request.header.frame_id = "map"
        request.request_id = request_id
        request.vehicle_state.header.stamp = stamp
        request.vehicle_state.header.frame_id = "map"
        request.vehicle_state.position = Point(x=-4.0, y=y, z=z)
        if include_reference:
            for x in (-4.0, goal_x):
                reference = GlobalReferencePoint()
                reference.position = Point(x=x, y=y, z=z)
                reference.has_time = False
                request.global_reference.append(reference)
        return request

    @staticmethod
    def make_free_cloud(stamp):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "map"
        points = [(20.0, y, z) for y in (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0)
                  for z in (0.0, 1.0, 2.0)]
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        # Endpoints on the map boundary are outside the valid half-open map
        # interval, so the interior rays mark a wide free corridor without
        # creating artificial occupied endpoints.
        cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
        return cloud

    @staticmethod
    def make_empty_tracks(stamp):
        batch = DynamicObstacleTrackArray()
        batch.header.stamp = stamp
        batch.header.frame_id = "map"
        return batch

    def wait_for_result(self, name, request_id, timeout_sec=20.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results[name]:
                if result.request_id == request_id:
                    return result
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.1, remaining))
        self.fail("timed out waiting for %s result %d" % (name, request_id))

    def publish_request(self, name, request):
        for _ in range(5):
            self.publishers[name].publish(request)
            self.spin_for(0.1)
            for result in self.results[name]:
                if result.request_id == request.request_id:
                    return result
        return self.wait_for_result(name, request.request_id)

    def send_empty_heartbeat(self, name, stamp):
        for _ in range(5):
            self.track_publishers[name].publish(self.make_empty_tracks(stamp))
            self.spin_for(0.05)

    def test_map_quality_controls_ros2_publish_path(self):
        self.wait_for_interfaces()

        free_stamp = self.node.get_clock().now().to_msg()
        self.publishers["free"].publish(
            self.make_request(100, free_stamp, goal_x=4.0, include_reference=False)
        )
        self.spin_for(0.2)
        for _ in range(5):
            self.cloud_publisher.publish(self.make_free_cloud(self.node.get_clock().now().to_msg()))
            self.spin_for(0.1)
        self.send_empty_heartbeat("free", self.node.get_clock().now().to_msg())
        free_result = self.publish_request(
            "free", self.make_request(101, self.node.get_clock().now().to_msg(), goal_x=4.0)
        )
        self.assertEqual(
            free_result.status,
            PlanningResult.SUCCESS,
            free_result.detail
            + " | safety="
            + str(free_result.safety_report.status)
            + " | risk="
            + free_result.risk_report.detail,
        )
        self.assertTrue(free_result.has_trajectory)
        self.assertEqual(free_result.trajectory.validation_state, Trajectory.VALIDATED)
        self.assertEqual(free_result.risk_report.static_risk, 0.0)
        self.assertEqual(free_result.risk_report.total_risk, 0.0)

        unknown_stamp = self.node.get_clock().now().to_msg()
        self.send_empty_heartbeat("unknown", unknown_stamp)
        unknown_result = self.publish_request(
            "unknown", self.make_request(201, unknown_stamp, goal_x=4.0)
        )
        # With map.reject_unknown=false, A* may produce a candidate through
        # UNKNOWN voxels. The enabled map-quality risk gate then rejects that
        # candidate before publication.
        self.assertEqual(unknown_result.status, PlanningResult.VALIDATION_FAILED)
        self.assertFalse(unknown_result.has_trajectory)
        self.assertIn("unknown", unknown_result.detail.lower())
        self.assertEqual(unknown_result.risk_report.risk_level, unknown_result.risk_report.HIGH)
        self.assertIn("map_unknown", unknown_result.detail.lower())
        self.assertEqual(len(self.trajectories["unknown"]), 0)

        occupied_cloud_publisher = self.node.create_publisher(
            PointCloud2, SCENARIOS["occupied"] + "/points", 10
        )
        seed_stamp = self.node.get_clock().now().to_msg()
        self.publishers["occupied"].publish(
            self.make_request(300, seed_stamp, include_reference=False)
        )
        self.spin_for(0.2)
        occupied_cloud = self.make_free_cloud(seed_stamp)
        occupied_cloud.header.frame_id = "map"
        occupied_cloud.data = struct.pack("<fff", 0.0, -0.25, 1.0)
        occupied_cloud.header.stamp = self.node.get_clock().now().to_msg()
        occupied_cloud.width = 1
        occupied_cloud.row_step = 12
        occupied_cloud.point_step = 12
        for _ in range(5):
            occupied_cloud_publisher.publish(occupied_cloud)
            self.spin_for(0.1)
        occupied_stamp = self.node.get_clock().now().to_msg()
        self.send_empty_heartbeat("occupied", occupied_stamp)
        occupied_request = self.make_request(301, occupied_stamp, goal_x=4.0, y=-0.25)
        occupied_result = self.publish_request("occupied", occupied_request)
        self.assertEqual(occupied_result.status, PlanningResult.SEARCH_FAILED)
        self.assertFalse(occupied_result.has_trajectory)
        self.assertEqual(len(self.trajectories["occupied"]), 0)
        self.node.destroy_publisher(occupied_cloud_publisher)

        out_of_map_stamp = self.node.get_clock().now().to_msg()
        self.send_empty_heartbeat("out_of_map", out_of_map_stamp)
        out_of_map_result = self.publish_request(
            "out_of_map", self.make_request(401, out_of_map_stamp, goal_x=10.0)
        )
        self.assertEqual(out_of_map_result.status, PlanningResult.SEARCH_FAILED)
        self.assertFalse(out_of_map_result.has_trajectory)
        self.assertEqual(len(self.trajectories["out_of_map"]), 0)
