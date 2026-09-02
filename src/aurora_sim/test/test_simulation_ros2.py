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
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
    PlanningResult,
    Trajectory,
    TrajectoryExecutionStatus,
)
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
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
            "ROS_LOG_DIR": "/tmp/aurora_simulation_planner",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "map.reject_unknown": False,
                # This test seeds the planner before publishing its first
                # cloud. Map freshness is exercised by the dedicated planner
                # fault regression, so keep the software-in-the-loop fixture
                # focused on the planner-to-executor contract.
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
    simulator_node = launch_ros.actions.Node(
        package="aurora_sim",
        executable="aurora_sim_node",
        name="aurora_sim_node",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_simulation_executor",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "simulation.initial_x": -4.0,
                "simulation.initial_y": 0.0,
                "simulation.initial_z": 1.0,
                "simulation.publish_rate_hz": 50.0,
            }
        ],
    )
    rejected_simulator_node = launch_ros.actions.Node(
        package="aurora_sim",
        executable="aurora_sim_node",
        name="aurora_rejecting_sim_node",
        output="screen",
        additional_env={
            "ROS_LOG_DIR": "/tmp/aurora_simulation_rejecting_executor",
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
        parameters=[
            {
                "simulation.reject_trajectories": True,
                "topics.trajectory": "/aurora/sim/rejected_trajectory",
                "topics.vehicle_state": "/aurora/sim/rejected_vehicle_state",
                "topics.desired_pose": "/aurora/sim/rejected_desired_pose",
                "topics.emergency_state": "/aurora/sim/rejected_emergency_state",
                "topics.execution_status": "/aurora/sim/rejected_status",
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [
                planner_node,
                simulator_node,
                rejected_simulator_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "planner": planner_node,
            "simulator": simulator_node,
            "rejected_simulator": rejected_simulator_node,
        },
    )


class TestSimulationRos2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("aurora_simulation_test")
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.request_publisher = self.node.create_publisher(
            PlanningRequest, "/aurora/planning_request", reliable_qos
        )
        self.pointcloud_publisher = self.node.create_publisher(
            PointCloud2, "/points", 10
        )
        self.dynamic_publisher = self.node.create_publisher(
            DynamicObstacleTrackArray,
            "/aurora/dynamic_obstacle_tracks",
            reliable_qos,
        )
        self.trajectory_publisher = self.node.create_publisher(
            Trajectory, "/aurora/trajectory", reliable_qos
        )
        self.rejected_trajectory_publisher = self.node.create_publisher(
            Trajectory, "/aurora/sim/rejected_trajectory", reliable_qos
        )
        self.results = []
        self.execution_statuses = []
        self.trajectories = []
        self.desired_poses = []
        self.rejected_statuses = []
        self.result_subscription = self.node.create_subscription(
            PlanningResult, "/aurora/planning_result", self.results.append, reliable_qos
        )
        self.execution_subscription = self.node.create_subscription(
            TrajectoryExecutionStatus,
            "/aurora/sim/execution_status",
            self.execution_statuses.append,
            reliable_qos,
        )
        self.trajectory_subscription = self.node.create_subscription(
            Trajectory, "/aurora/trajectory", self.trajectories.append, reliable_qos
        )
        self.desired_subscription = self.node.create_subscription(
            PoseStamped,
            "/aurora/sim/desired_pose",
            self.desired_poses.append,
            reliable_qos,
        )
        self.rejected_status_subscription = self.node.create_subscription(
            TrajectoryExecutionStatus,
            "/aurora/sim/rejected_status",
            self.rejected_statuses.append,
            reliable_qos,
        )

    def tearDown(self):
        for subscription in (
            self.result_subscription,
            self.execution_subscription,
            self.trajectory_subscription,
            self.desired_subscription,
            self.rejected_status_subscription,
        ):
            self.node.destroy_subscription(subscription)
        for publisher in (
            self.request_publisher,
            self.pointcloud_publisher,
            self.dynamic_publisher,
            self.trajectory_publisher,
            self.rejected_trajectory_publisher,
        ):
            self.node.destroy_publisher(publisher)
        self.node.destroy_node()

    def spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.05, remaining))

    def wait_for_interfaces(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if (
                self.node.count_subscribers("/aurora/planning_request") > 0
                and self.node.count_subscribers("/aurora/trajectory") > 0
                and self.node.count_publishers("/aurora/planning_result") > 0
                and self.node.count_publishers("/aurora/sim/execution_status") > 0
                and self.node.count_publishers("/aurora/sim/desired_pose") > 0
                and self.node.count_subscribers("/aurora/sim/rejected_trajectory") > 0
                and self.node.count_publishers("/aurora/sim/rejected_status") > 0
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("planner and simulator interfaces did not become discoverable")

    def now_message(self):
        return self.node.get_clock().now().to_msg()

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
            for x, z in ((-4.0, 1.0), (0.0, 2.0), (4.0, 1.0)):
                reference = GlobalReferencePoint()
                reference.position = Point(x=x, y=0.0, z=z)
                reference.has_time = False
                request.global_reference.append(reference)
        return request

    @staticmethod
    def make_cloud(stamp):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "map"
        cloud.height = 1
        endpoints = []
        for y in (-4.0, 0.0, 4.0):
            for z in (-1.0, 1.0, 3.0):
                endpoints.append((10.0, y, z))
        cloud.width = len(endpoints)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = b"".join(struct.pack("<fff", *endpoint) for endpoint in endpoints)
        return cloud

    @staticmethod
    def make_empty_tracks(stamp):
        tracks = DynamicObstacleTrackArray()
        tracks.header.stamp = stamp
        tracks.header.frame_id = "map"
        return tracks

    @staticmethod
    def make_three_dimensional_command(stamp, trajectory_id, validation_state):
        message = Trajectory()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.trajectory_id = trajectory_id
        message.validation_state = validation_state
        message.safety_report.accepted = validation_state == Trajectory.VALIDATED
        from aurora_msgs.msg import TrajectorySegment

        segment = TrajectorySegment()
        segment.start_stamp = stamp
        segment.source_start_time = 0.0
        segment.duration = 1.0
        segment.dt = 0.25
        segment.degree = 3
        segment.knot_mode = TrajectorySegment.CLAMPED
        control_points = [
            (-4.0, 0.0, 1.0),
            (-4.0, 0.0, 1.0),
            (-3.0, 0.0, 1.0),
            (-1.0, 0.0, 2.5),
            (1.0, 0.0, 3.0),
            (3.0, 0.0, 3.0),
            (4.0, 0.0, 3.0),
        ]
        segment.control_points = [Point(x=x, y=y, z=z) for x, y, z in control_points]
        message.segments.append(segment)
        return message

    def wait_for_result(self, request_id, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for result in self.results:
                if result.request_id == request_id:
                    return result
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail("timed out waiting for planning result %d" % request_id)

    def wait_for_execution_status(self, trajectory_id, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for status in self.execution_statuses:
                if status.trajectory_id == trajectory_id:
                    return status
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail("timed out waiting for execution status %d" % trajectory_id)

    def wait_for_rejected_status(self, trajectory_id, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            for status in self.rejected_statuses:
                if status.trajectory_id == trajectory_id:
                    return status
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail("timed out waiting for rejected execution status %d" % trajectory_id)

    def test_planner_trajectory_is_executed_in_three_dimensions(self):
        self.wait_for_interfaces()

        seed_stamp = self.now_message()
        self.dynamic_publisher.publish(self.make_empty_tracks(seed_stamp))
        self.request_publisher.publish(
            self.make_request(6000, seed_stamp, include_reference=False)
        )
        self.spin_for(0.2)
        cloud = self.make_cloud(seed_stamp)
        for _ in range(5):
            self.pointcloud_publisher.publish(cloud)
            self.spin_for(0.05)

        request_stamp = self.now_message()
        request = self.make_request(6001, request_stamp)
        for _ in range(5):
            self.dynamic_publisher.publish(self.make_empty_tracks(request_stamp))
            self.request_publisher.publish(request)
            self.spin_for(0.1)
            if any(result.request_id == 6001 for result in self.results):
                break
        result = self.wait_for_result(6001)
        self.assertEqual(result.status, PlanningResult.SUCCESS, result.detail)
        self.assertTrue(result.has_trajectory)
        self.assertEqual(result.trajectory.validation_state, Trajectory.VALIDATED)

        status = self.wait_for_execution_status(result.trajectory.trajectory_id)
        self.assertTrue(status.accepted, status.detail)
        self.assertIn(
            status.status,
            (TrajectoryExecutionStatus.ACTIVE, TrajectoryExecutionStatus.COMPLETED),
        )
        self.assertTrue(self.desired_poses)
        self.assertTrue(
            all(
                pose.pose.position.x == pose.pose.position.x
                and pose.pose.position.y == pose.pose.position.y
                and pose.pose.position.z == pose.pose.position.z
                for pose in self.desired_poses
            )
        )

        # Exercise the ROS conversion and execution path with a deterministic
        # three-dimensional command whose vertical excursion is unambiguous.
        manual_stamp = self.now_message()
        manual = self.make_three_dimensional_command(
            manual_stamp, 6002, Trajectory.VALIDATED
        )
        for _ in range(3):
            self.trajectory_publisher.publish(manual)
            self.spin_for(0.1)
        manual_status = self.wait_for_execution_status(6002)
        self.assertTrue(manual_status.accepted, manual_status.detail)
        self.spin_for(0.35)
        self.assertGreater(
            max(pose.pose.position.z for pose in self.desired_poses), 1.1
        )

        rejected = self.make_three_dimensional_command(
            self.now_message(), 6003, Trajectory.DEGRADED
        )
        self.trajectory_publisher.publish(rejected)
        rejected_status = self.wait_for_execution_status(6003)
        self.assertFalse(rejected_status.accepted)
        self.assertEqual(
            rejected_status.status,
            TrajectoryExecutionStatus.REJECTED_UNVALIDATED,
        )

        controller_rejection = self.make_three_dimensional_command(
            self.now_message(), 6004, Trajectory.VALIDATED
        )
        self.rejected_trajectory_publisher.publish(controller_rejection)
        controller_status = self.wait_for_rejected_status(6004)
        self.assertFalse(controller_status.accepted)
        self.assertEqual(
            controller_status.status,
            TrajectoryExecutionStatus.REJECTED_UNSAFE,
        )


@launch_testing.post_shutdown_test()
class TestSimulationRos2Shutdown(unittest.TestCase):
    def test_nodes_exited_successfully(
        self, proc_info, planner, simulator, rejected_simulator
    ):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=planner
        )
        launch_testing.asserts.assertExitCodes(
            proc_info, process=simulator
        )
        launch_testing.asserts.assertExitCodes(
            proc_info, process=rejected_simulator
        )
