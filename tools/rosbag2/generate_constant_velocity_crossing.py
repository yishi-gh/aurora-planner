#!/usr/bin/env python3
"""Generate the deterministic constant-velocity crossing rosbag2 scenario."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from aurora_msgs.msg import DynamicObstacleTrack, DynamicObstacleTrackArray
from geometry_msgs.msg import Point
from rclpy.serialization import serialize_message

from generate_static_baseline import (
    BASE_STAMP_NS,
    DYNAMIC_TOPIC,
    MAP_FRAME,
    POINTCLOUD_TOPIC,
    REQUEST_TOPIC,
    make_dynamic_heartbeat,
    make_point_cloud,
    make_request,
    stamp_message,
)


STATE_PRIMING_REQUEST_ID = 2700
PLANNING_REQUEST_ID = 2701
TRACK_ID = 7


def make_crossing_track(stamp_ns: int) -> DynamicObstacleTrackArray:
    track = DynamicObstacleTrack()
    track.header.stamp = stamp_message(stamp_ns)
    track.header.frame_id = MAP_FRAME
    track.track_id = TRACK_ID
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

    message = DynamicObstacleTrackArray()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    message.tracks.append(track)
    return message


def generate_constant_velocity_crossing(output_uri: Path) -> Path:
    """Write a deterministic SQLite3 bag and return its manifest path."""

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
            DYNAMIC_TOPIC,
            "aurora_msgs/msg/DynamicObstacleTrackArray",
            "cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            3, REQUEST_TOPIC, "aurora_msgs/msg/PlanningRequest", "cdr"
        )
    )

    events = (
        (BASE_STAMP_NS, DYNAMIC_TOPIC, make_dynamic_heartbeat(BASE_STAMP_NS)),
        (
            BASE_STAMP_NS + 10_000_000,
            REQUEST_TOPIC,
            make_request(BASE_STAMP_NS + 10_000_000, STATE_PRIMING_REQUEST_ID, False),
        ),
        (
            BASE_STAMP_NS + 50_000_000,
            POINTCLOUD_TOPIC,
            make_point_cloud(BASE_STAMP_NS + 50_000_000),
        ),
        (
            BASE_STAMP_NS + 100_000_000,
            REQUEST_TOPIC,
            make_request(BASE_STAMP_NS + 100_000_000, PLANNING_REQUEST_ID, True),
        ),
        (
            BASE_STAMP_NS + 300_000_000,
            DYNAMIC_TOPIC,
            make_dynamic_heartbeat(BASE_STAMP_NS + 300_000_000),
        ),
        (
            BASE_STAMP_NS + 500_000_000,
            DYNAMIC_TOPIC,
            make_crossing_track(BASE_STAMP_NS + 500_000_000),
        ),
    )
    for stamp_ns, topic, message in events:
        writer.write(topic, serialize_message(message), stamp_ns)

    manifest_path = output_uri / "scenario_manifest.json"
    manifest = {
        "scenario": "constant_velocity_crossing_v1",
        "storage_id": "sqlite3",
        "map_frame": MAP_FRAME,
        "base_stamp_ns": BASE_STAMP_NS,
        "bag_start_ns": events[0][0],
        "bag_end_ns": events[-1][0],
        "state_priming_request_id": STATE_PRIMING_REQUEST_ID,
        "expected_request_id": PLANNING_REQUEST_ID,
        "track_id": TRACK_ID,
        "topics": [POINTCLOUD_TOPIC, DYNAMIC_TOPIC, REQUEST_TOPIC],
        "events": [
            {"stamp_ns": stamp_ns, "topic": topic}
            for stamp_ns, topic, _message in events
        ],
        "pointcloud_endpoint_count": 7,
        "dynamic_track_count": 1,
        "dynamic_track": {
            "shape": "sphere",
            "radius_m": 0.25,
            "initial_position_m": [0.0, -1.5, 1.0],
            "velocity_mps": [0.0, 1.0, 0.0],
            "existence_probability": 1.0,
            "prediction_model": "CV",
            "position_variance_m2": 0.0025,
            "velocity_variance_m2ps2": 0.01,
        },
        "playback": {
            "clock_topics_all": True,
            "rate": 1.0,
            "use_sim_time": True,
        },
        "expected_behavior": {
            "initial_static_result": "SUCCESS",
            "dynamic_heartbeat_replan": True,
            "crossing_result": "VALIDATION_FAILED",
            "crossing_risk_level": "HIGH",
            "emergency_stop_latched": True,
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new bag directory")
    args = parser.parse_args()
    manifest_path = generate_constant_velocity_crossing(args.output)
    print("bag_uri=%s" % args.output)
    print("manifest=%s" % manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
