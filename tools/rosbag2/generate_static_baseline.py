#!/usr/bin/env python3
"""Generate the deterministic static 3D AURORA rosbag2 baseline.

The bag deliberately contains a state-priming request before the point cloud.
The planner uses the most recent vehicle state as the origin of PointCloud2
ray integration, so this ordering makes the map observation deterministic.
"""

import argparse
import json
import struct
from pathlib import Path

import rosbag2_py
from aurora_msgs.msg import (
    DynamicObstacleTrackArray,
    GlobalReferencePoint,
    PlanningRequest,
)
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField


BASE_STAMP_NS = 1_000_000_000_000
MAP_FRAME = "map"
STATE_PRIMING_REQUEST_ID = 2600
PLANNING_REQUEST_ID = 2601

POINTCLOUD_TOPIC = "/points"
DYNAMIC_TOPIC = "/aurora/dynamic_obstacle_tracks"
REQUEST_TOPIC = "/aurora/planning_request"


def stamp_message(stamp_ns: int) -> Time:
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    message = Time()
    message.sec = seconds
    message.nanosec = nanoseconds
    return message


def make_point_cloud(stamp_ns: int) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    message.height = 1

    # Endpoints are outside the local planning corridor. The first endpoint
    # extends the observed free ray past the local goal; the remaining points
    # make the input a small, deterministic 3D scene without blocking the
    # baseline flight corridor from x=-4 to x=2, y=0, z=1.
    endpoints = (
        (10.0, 0.0, 1.0),
        (10.0, 2.5, 1.0),
        (10.0, -2.5, 1.0),
        (10.0, 0.0, 3.0),
        (10.0, 0.0, -1.0),
        (0.0, 4.0, 1.0),
        (0.0, -4.0, 1.0),
    )
    message.width = len(endpoints)
    message.is_bigendian = False
    message.is_dense = True
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = b"".join(struct.pack("<fff", *endpoint) for endpoint in endpoints)
    return message


def make_dynamic_heartbeat(stamp_ns: int) -> DynamicObstacleTrackArray:
    message = DynamicObstacleTrackArray()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    return message


def make_request(stamp_ns: int, request_id: int, include_reference: bool) -> PlanningRequest:
    message = PlanningRequest()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    message.request_id = request_id
    message.vehicle_state.header.stamp = stamp_message(stamp_ns)
    message.vehicle_state.header.frame_id = MAP_FRAME
    message.vehicle_state.position = Point(x=-4.0, y=0.0, z=1.0)

    if include_reference:
        for x in (-4.0, 4.0):
            reference = GlobalReferencePoint()
            reference.position = Point(x=x, y=0.0, z=1.0)
            reference.has_time = False
            message.global_reference.append(reference)
    return message


def generate_static_baseline(output_uri: Path) -> Path:
    """Write a deterministic SQLite3 bag and return its manifest path."""

    output_uri = Path(output_uri)
    if output_uri.exists():
        raise FileExistsError("refusing to overwrite existing bag: %s" % output_uri)
    output_uri.parent.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(
        uri=str(output_uri), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

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
    )
    for stamp_ns, topic, message in events:
        writer.write(topic, serialize_message(message), stamp_ns)

    manifest_path = output_uri / "scenario_manifest.json"
    manifest = {
        "scenario": "static_3d_baseline_v1",
        "storage_id": "sqlite3",
        "map_frame": MAP_FRAME,
        "base_stamp_ns": BASE_STAMP_NS,
        "bag_start_ns": events[0][0],
        "bag_end_ns": events[-1][0],
        "state_priming_request_id": STATE_PRIMING_REQUEST_ID,
        "expected_request_id": PLANNING_REQUEST_ID,
        "topics": [POINTCLOUD_TOPIC, DYNAMIC_TOPIC, REQUEST_TOPIC],
        "events": [
            {"stamp_ns": stamp_ns, "topic": topic}
            for stamp_ns, topic, _message in events
        ],
        "pointcloud_endpoint_count": 7,
        "dynamic_track_count": 0,
        "playback": {
            "clock_topics_all": True,
            "rate": 1.0,
            "use_sim_time": True,
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new bag directory")
    args = parser.parse_args()
    manifest_path = generate_static_baseline(args.output)
    print("bag_uri=%s" % args.output)
    print("manifest=%s" % manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
