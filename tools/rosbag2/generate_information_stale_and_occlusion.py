#!/usr/bin/env python3
"""Generate deterministic stale-information and explicit-occlusion bags."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from aurora_msgs.msg import DynamicObstacleTrackArray
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


STALE_STATE_PRIMING_REQUEST_ID = 3000
STALE_REQUEST_ID = 3001
STALE_RECOVERY_REQUEST_ID = 3002
OCCLUSION_STATE_PRIMING_REQUEST_ID = 3200
OCCLUSION_REQUEST_ID = 3201
OCCLUSION_RECOVERY_REQUEST_ID = 3202


def make_occluded_batch(stamp_ns: int) -> DynamicObstacleTrackArray:
    message = DynamicObstacleTrackArray()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    message.occlusion_active = True
    message.occluded_track_ids = [77]
    return message


def _write_bag(output_uri: Path, scenario: str, events, manifest) -> Path:
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

    for stamp_ns, topic, message in events:
        writer.write(topic, serialize_message(message), stamp_ns)

    manifest.update(
        {
            "scenario": scenario,
            "storage_id": "sqlite3",
            "map_frame": MAP_FRAME,
            "base_stamp_ns": BASE_STAMP_NS,
            "bag_start_ns": events[0][0],
            "bag_end_ns": events[-1][0],
            "topics": [POINTCLOUD_TOPIC, DYNAMIC_TOPIC, REQUEST_TOPIC],
            "events": [
                {"stamp_ns": stamp_ns, "topic": topic}
                for stamp_ns, topic, _message in events
            ],
            "pointcloud_endpoint_count": 7,
            "playback": {
                "clock_topics_all": True,
                "rate": 1.0,
                "use_sim_time": True,
            },
        }
    )
    manifest_path = output_uri / "scenario_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest_path


def _common_events(priming_request_id: int, request_id: int):
    return [
        (BASE_STAMP_NS, DYNAMIC_TOPIC, make_dynamic_heartbeat(BASE_STAMP_NS)),
        (
            BASE_STAMP_NS + 10_000_000,
            REQUEST_TOPIC,
            make_request(BASE_STAMP_NS + 10_000_000, priming_request_id, False),
        ),
        (
            BASE_STAMP_NS + 50_000_000,
            POINTCLOUD_TOPIC,
            make_point_cloud(BASE_STAMP_NS + 50_000_000),
        ),
        (
            BASE_STAMP_NS + 100_000_000,
            REQUEST_TOPIC,
            make_request(BASE_STAMP_NS + 100_000_000, request_id, True),
        ),
    ]


def generate_prediction_stale(output_uri: Path) -> Path:
    """Stop dynamic updates, advance /clock, then leave room for recovery."""

    events = _common_events(STALE_STATE_PRIMING_REQUEST_ID, STALE_REQUEST_ID)
    # These map-only events advance simulated time after the last dynamic
    # snapshot. They do not create a new dynamic heartbeat.
    for offset_ns in (700_000_000, 1_100_000_000, 1_400_000_000):
        events.append(
            (
                BASE_STAMP_NS + offset_ns,
                POINTCLOUD_TOPIC,
                make_point_cloud(BASE_STAMP_NS + offset_ns),
            )
        )
    return _write_bag(
        output_uri,
        "prediction_information_stale_v1",
        events,
        {
            "state_priming_request_id": STALE_STATE_PRIMING_REQUEST_ID,
            "expected_request_id": STALE_REQUEST_ID,
            "recovery_request_id": STALE_RECOVERY_REQUEST_ID,
            "last_dynamic_snapshot_ns": BASE_STAMP_NS,
            "expected_behavior": {
                "initial_result": "SUCCESS",
                "stale_result": "VALIDATION_FAILED",
                "stale_safety_status": "INFORMATION_STALE",
                "trajectory_retained_during_hold": True,
                "emergency_stop_latched": True,
                "recovery_requires_dynamic_heartbeat_and_request": True,
            },
        },
    )


def generate_explicit_occlusion(output_uri: Path) -> Path:
    """Mark the dynamic set occluded, then advance /clock past the hold."""

    events = _common_events(
        OCCLUSION_STATE_PRIMING_REQUEST_ID, OCCLUSION_REQUEST_ID
    )
    events.extend(
        [
            (
                BASE_STAMP_NS + 300_000_000,
                DYNAMIC_TOPIC,
                make_occluded_batch(BASE_STAMP_NS + 300_000_000),
            ),
            (
                BASE_STAMP_NS + 900_000_000,
                POINTCLOUD_TOPIC,
                make_point_cloud(BASE_STAMP_NS + 900_000_000),
            ),
            (
                BASE_STAMP_NS + 1_200_000_000,
                POINTCLOUD_TOPIC,
                make_point_cloud(BASE_STAMP_NS + 1_200_000_000),
            ),
        ]
    )
    return _write_bag(
        output_uri,
        "explicit_dynamic_occlusion_v1",
        events,
        {
            "state_priming_request_id": OCCLUSION_STATE_PRIMING_REQUEST_ID,
            "expected_request_id": OCCLUSION_REQUEST_ID,
            "recovery_request_id": OCCLUSION_RECOVERY_REQUEST_ID,
            "occlusion_stamp_ns": BASE_STAMP_NS + 300_000_000,
            "occluded_track_ids": [77],
            "expected_behavior": {
                "initial_result": "SUCCESS",
                "occlusion_result": "VALIDATION_FAILED",
                "occlusion_safety_status": "INFORMATION_STALE",
                "emergency_stop_latched": True,
            },
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", choices=("stale", "occlusion"), required=True
    )
    parser.add_argument("--output", required=True, type=Path, help="new bag directory")
    args = parser.parse_args()
    generator = generate_prediction_stale if args.scenario == "stale" else generate_explicit_occlusion
    manifest_path = generator(args.output)
    print("bag_uri=%s" % args.output)
    print("manifest=%s" % manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
