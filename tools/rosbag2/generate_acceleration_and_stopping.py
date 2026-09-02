#!/usr/bin/env python3
"""Generate deterministic accelerating and braking/stopping rosbag2 scenarios."""

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


ACCELERATION_STATE_PRIMING_REQUEST_ID = 2800
ACCELERATION_REQUEST_ID = 2801
ACCELERATION_TRACK_ID = 8
ACCELERATION_STAMP_NS = BASE_STAMP_NS + 500_000_000

STOPPING_STATE_PRIMING_REQUEST_ID = 2900
STOPPING_REQUEST_ID = 2901
STOPPING_TRACK_ID = 9
BRAKING_STAMP_NS = BASE_STAMP_NS + 500_000_000
STOPPED_STAMP_NS = BASE_STAMP_NS + 1_500_000_000


def make_kinematic_track(
    stamp_ns: int,
    track_id: int,
    position,
    velocity,
    acceleration,
    position_variance: float,
    velocity_variance: float,
) -> DynamicObstacleTrackArray:
    track = DynamicObstacleTrack()
    track.header.stamp = stamp_message(stamp_ns)
    track.header.frame_id = MAP_FRAME
    track.track_id = track_id
    track.pose.position = Point(x=position[0], y=position[1], z=position[2])
    track.twist.linear.x = velocity[0]
    track.twist.linear.y = velocity[1]
    track.twist.linear.z = velocity[2]
    track.acceleration.linear.x = acceleration[0]
    track.acceleration.linear.y = acceleration[1]
    track.acceleration.linear.z = acceleration[2]
    track.shape_type = DynamicObstacleTrack.SPHERE
    track.radius = 0.25
    track.existence_probability = 1.0
    track.prediction_model = DynamicObstacleTrack.CA
    track.has_state_covariance = True
    track.state_covariance = [0.0] * 36
    for index in (0, 7, 14):
        track.state_covariance[index] = position_variance
    for index in (21, 28, 35):
        track.state_covariance[index] = velocity_variance

    message = DynamicObstacleTrackArray()
    message.header.stamp = stamp_message(stamp_ns)
    message.header.frame_id = MAP_FRAME
    message.tracks.append(track)
    return message


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
        (
            BASE_STAMP_NS + 300_000_000,
            DYNAMIC_TOPIC,
            make_dynamic_heartbeat(BASE_STAMP_NS + 300_000_000),
        ),
    ]


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


def generate_accelerating_crossing(output_uri: Path) -> Path:
    """Generate a CA target that accelerates into the aircraft corridor."""

    events = _common_events(
        ACCELERATION_STATE_PRIMING_REQUEST_ID, ACCELERATION_REQUEST_ID
    )
    events.append(
        (
            ACCELERATION_STAMP_NS,
            DYNAMIC_TOPIC,
            make_kinematic_track(
                ACCELERATION_STAMP_NS,
                ACCELERATION_TRACK_ID,
                (0.0, -1.5, 1.0),
                (0.0, 0.0, 0.0),
                (0.0, 4.0 / 3.0, 0.0),
                0.0025,
                0.01,
            ),
        )
    )
    return _write_bag(
        output_uri,
        "constant_acceleration_crossing_v1",
        events,
        {
            "state_priming_request_id": ACCELERATION_STATE_PRIMING_REQUEST_ID,
            "expected_request_id": ACCELERATION_REQUEST_ID,
            "track_id": ACCELERATION_TRACK_ID,
            "dynamic_track_update_count": 1,
            "dynamic_track": {
                "shape": "sphere",
                "radius_m": 0.25,
                "initial_position_m": [0.0, -1.5, 1.0],
                "initial_velocity_mps": [0.0, 0.0, 0.0],
                "constant_acceleration_mps2": [0.0, 4.0 / 3.0, 0.0],
                "existence_probability": 1.0,
                "prediction_model": "CA",
                "position_variance_m2": 0.0025,
                "velocity_variance_m2ps2": 0.01,
            },
            "expected_behavior": {
                "initial_static_result": "SUCCESS",
                "dynamic_update_replan": True,
                "motion_result": "VALIDATION_FAILED",
                "motion_risk_level": "HIGH",
                "emergency_stop_latched": True,
            },
        },
    )


def generate_stopping_before_crossing(output_uri: Path) -> Path:
    """Generate a CA target that brakes and remains outside the corridor."""

    events = _common_events(STOPPING_STATE_PRIMING_REQUEST_ID, STOPPING_REQUEST_ID)
    events.extend(
        [
            (
                BRAKING_STAMP_NS,
                DYNAMIC_TOPIC,
                make_kinematic_track(
                    BRAKING_STAMP_NS,
                    STOPPING_TRACK_ID,
                    (0.0, -3.0, 1.0),
                    (0.0, 1.0, 0.0),
                    (0.0, -1.0, 0.0),
                    1e-6,
                    1e-6,
                ),
            ),
            (
                STOPPED_STAMP_NS,
                DYNAMIC_TOPIC,
                make_kinematic_track(
                    STOPPED_STAMP_NS,
                    STOPPING_TRACK_ID,
                    (0.0, -2.5, 1.0),
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    1e-6,
                    1e-6,
                ),
            ),
        ]
    )
    return _write_bag(
        output_uri,
        "constant_acceleration_braking_stop_v1",
        events,
        {
            "state_priming_request_id": STOPPING_STATE_PRIMING_REQUEST_ID,
            "expected_request_id": STOPPING_REQUEST_ID,
            "track_id": STOPPING_TRACK_ID,
            "dynamic_track_update_count": 2,
            "dynamic_track": {
                "shape": "sphere",
                "radius_m": 0.25,
                "braking_position_m": [0.0, -3.0, 1.0],
                "braking_velocity_mps": [0.0, 1.0, 0.0],
                "braking_acceleration_mps2": [0.0, -1.0, 0.0],
                "stopped_position_m": [0.0, -2.5, 1.0],
                "stopped_velocity_mps": [0.0, 0.0, 0.0],
                "stopped_acceleration_mps2": [0.0, 0.0, 0.0],
                "existence_probability": 1.0,
                "prediction_model": "CA",
                "position_variance_m2": 1e-6,
                "velocity_variance_m2ps2": 1e-6,
            },
            "expected_behavior": {
                "initial_static_result": "SUCCESS",
                "braking_update_replan": True,
                "stopped_update_replan": True,
                "motion_results": "SUCCESS",
                "motion_risk_level": "LOW",
                "emergency_stop_latched": False,
            },
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", choices=("accelerating", "stopping"), required=True
    )
    parser.add_argument("--output", required=True, type=Path, help="new bag directory")
    args = parser.parse_args()
    generator = (
        generate_accelerating_crossing
        if args.scenario == "accelerating"
        else generate_stopping_before_crossing
    )
    manifest_path = generator(args.output)
    print("bag_uri=%s" % args.output)
    print("manifest=%s" % manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
