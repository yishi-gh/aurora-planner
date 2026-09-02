#!/usr/bin/env python3
"""Generate a deterministic internal-detection crossing rosbag2 scenario."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from aurora_msgs.msg import (
    UnassociatedObstacleDetection,
    UnassociatedObstacleDetectionArray,
)
from geometry_msgs.msg import Point
from rclpy.serialization import serialize_message

from generate_static_baseline import (
    BASE_STAMP_NS,
    MAP_FRAME,
    POINTCLOUD_TOPIC,
    make_point_cloud,
    make_request,
    stamp_message,
)


INTERNAL_TOPIC = "/aurora/dynamic_obstacle_detections"
STATE_PRIMING_REQUEST_ID = 3300
PLANNING_REQUEST_ID = 3301
BENIGN_TRACK_ID = 1
CROSSING_TRACK_ID = 2
BENIGN_DETECTION_STAMP_NS = BASE_STAMP_NS + 300_000_000
CROSSING_DETECTION_STAMP_NS = BASE_STAMP_NS + 500_000_000


def make_empty_batch(stamp_ns: int) -> UnassociatedObstacleDetectionArray:
    message = UnassociatedObstacleDetectionArray()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    return message


def make_detection_batch(
    stamp_ns: int, position, velocity
) -> UnassociatedObstacleDetectionArray:
    detection = UnassociatedObstacleDetection()
    detection.header.stamp = stamp_message(stamp_ns)
    detection.header.frame_id = MAP_FRAME
    detection.position = Point(x=position[0], y=position[1], z=position[2])
    detection.has_position_covariance = True
    detection.position_covariance = [0.0] * 9
    for index in (0, 4, 8):
        detection.position_covariance[index] = 0.0025
    detection.has_velocity = True
    detection.velocity.x = velocity[0]
    detection.velocity.y = velocity[1]
    detection.velocity.z = velocity[2]
    detection.has_velocity_covariance = True
    detection.velocity_covariance = [0.0] * 9
    for index in (0, 4, 8):
        detection.velocity_covariance[index] = 0.01
    detection.has_shape = True
    detection.shape_type = UnassociatedObstacleDetection.SPHERE
    detection.radius = 0.25

    message = UnassociatedObstacleDetectionArray()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    message.detections.append(detection)
    return message


def make_crossing_detection(stamp_ns: int) -> UnassociatedObstacleDetectionArray:
    return make_detection_batch(stamp_ns, (0.0, -1.5, 1.0), (0.0, 1.0, 0.0))


def make_benign_detection(stamp_ns: int) -> UnassociatedObstacleDetectionArray:
    # The first update is intentionally outside the planning horizon. It
    # exercises the internal source and active-replan event without making the
    # current trajectory unsafe before the crossing update arrives.
    return make_detection_batch(stamp_ns, (100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


def generate_internal_detection_crossing(output_uri: Path) -> Path:
    """Write a SQLite3 bag and return the scenario manifest path."""

    output_uri = Path(output_uri)
    if output_uri.exists():
        raise FileExistsError("refusing to overwrite existing bag: %s" % output_uri)
    output_uri.parent.mkdir(parents=True, exist_ok=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            1, POINTCLOUD_TOPIC, "sensor_msgs/msg/PointCloud2", "cdr"
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            2,
            INTERNAL_TOPIC,
            "aurora_msgs/msg/UnassociatedObstacleDetectionArray",
            "cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            3, "/aurora/planning_request", "aurora_msgs/msg/PlanningRequest", "cdr"
        )
    )

    events = (
        (BASE_STAMP_NS, INTERNAL_TOPIC, make_empty_batch(BASE_STAMP_NS)),
        (
            BASE_STAMP_NS + 10_000_000,
            "/aurora/planning_request",
            make_request(BASE_STAMP_NS + 10_000_000, STATE_PRIMING_REQUEST_ID, False),
        ),
        (
            BASE_STAMP_NS + 50_000_000,
            POINTCLOUD_TOPIC,
            make_point_cloud(BASE_STAMP_NS + 50_000_000),
        ),
        (
            BASE_STAMP_NS + 100_000_000,
            "/aurora/planning_request",
            make_request(BASE_STAMP_NS + 100_000_000, PLANNING_REQUEST_ID, True),
        ),
        (
            BENIGN_DETECTION_STAMP_NS,
            INTERNAL_TOPIC,
            make_benign_detection(BENIGN_DETECTION_STAMP_NS),
        ),
        (
            CROSSING_DETECTION_STAMP_NS,
            INTERNAL_TOPIC,
            make_crossing_detection(CROSSING_DETECTION_STAMP_NS),
        ),
    )
    for stamp_ns, topic, message in events:
        writer.write(topic, serialize_message(message), stamp_ns)

    manifest = {
        "scenario": "internal_detection_crossing_v1",
        "storage_id": "sqlite3",
        "map_frame": MAP_FRAME,
        "dynamic_input_mode": "internal_detections",
        "base_stamp_ns": BASE_STAMP_NS,
        "bag_start_ns": events[0][0],
        "bag_end_ns": events[-1][0],
        "state_priming_request_id": STATE_PRIMING_REQUEST_ID,
        "expected_request_id": PLANNING_REQUEST_ID,
        "benign_track_id": BENIGN_TRACK_ID,
        "crossing_track_id": CROSSING_TRACK_ID,
        "topics": [POINTCLOUD_TOPIC, INTERNAL_TOPIC, "/aurora/planning_request"],
        "events": [
            {"stamp_ns": stamp_ns, "topic": topic}
            for stamp_ns, topic, _message in events
        ],
        "pointcloud_endpoint_count": 7,
        "detection_count": 2,
        "detections": [
            {
                "role": "benign_replan_trigger",
                "position_m": [100.0, 100.0, 100.0],
                "velocity_mps": [0.0, 0.0, 0.0],
            },
            {
                "role": "crossing_risk_trigger",
                "position_m": [0.0, -1.5, 1.0],
                "velocity_mps": [0.0, 1.0, 0.0],
            },
        ],
        "shape": "sphere",
        "radius_m": 0.25,
        "position_variance_m2": 0.0025,
        "velocity_variance_m2ps2": 0.01,
        "playback": {
            "clock_topics_all": True,
            "rate": 1.0,
            "use_sim_time": True,
        },
        "expected_behavior": {
            "initial_result": "SUCCESS",
            "internal_detection_replan": True,
            "crossing_result": "VALIDATION_FAILED",
            "crossing_risk_level": "HIGH",
            "emergency_stop_latched": True,
        },
    }
    manifest_path = output_uri / "scenario_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new bag directory")
    args = parser.parse_args()
    manifest_path = generate_internal_detection_crossing(args.output)
    print("bag_uri=%s" % args.output)
    print("manifest=%s" % manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
