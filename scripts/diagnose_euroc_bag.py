#!/usr/bin/env python3
"""Print quick stereo/timestamp diagnostics for EuRoC-style ROS 2 bags."""

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosbags.highlevel import AnyReader
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Diagnose EuRoC stereo bag timing and CameraInfo.")
    parser.add_argument("bag", nargs="?", default="MH_01_easy_ros2_rectified")
    parser.add_argument("--left-image-topic", default="/cam0/image_rect")
    parser.add_argument("--right-image-topic", default="/cam1/image_rect")
    parser.add_argument("--left-info-topic", default="/cam0/camera_info")
    parser.add_argument("--right-info-topic", default="/cam1/camera_info")
    parser.add_argument("--limit", type=int, default=5000)
    return parser.parse_args()


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def describe_deltas(name: str, values: list[int]) -> None:
    if len(values) < 2:
        print(f"{name}: not enough samples")
        return
    deltas_ms = np.diff(np.sort(np.array(values, dtype=np.int64))) * 1e-6
    print(
        f"{name}: count={len(values)} rate_mean={1000.0 / np.mean(deltas_ms):.3f}Hz "
        f"dt_ms min/mean/max={np.min(deltas_ms):.3f}/{np.mean(deltas_ms):.3f}/{np.max(deltas_ms):.3f}"
    )


def main() -> None:
    args = parse_args()
    bag = Path(args.bag).expanduser()
    image_type = get_message("sensor_msgs/msg/Image")
    info_type = get_message("sensor_msgs/msg/CameraInfo")
    image_stamps = defaultdict(list)
    header_stamps = defaultdict(list)
    frame_ids = defaultdict(set)
    infos = {}

    with AnyReader([bag]) as reader:
        for connection, timestamp_ns, rawdata in reader.messages():
            if connection.topic in (args.left_image_topic, args.right_image_topic):
                if len(image_stamps[connection.topic]) >= args.limit:
                    continue
                msg = deserialize_message(bytes(rawdata), image_type)
                image_stamps[connection.topic].append(timestamp_ns)
                header_stamps[connection.topic].append(stamp_ns(msg))
                frame_ids[connection.topic].add(msg.header.frame_id)
            elif connection.topic in (args.left_info_topic, args.right_info_topic):
                if connection.topic in infos:
                    continue
                msg = deserialize_message(bytes(rawdata), info_type)
                infos[connection.topic] = msg
                frame_ids[connection.topic].add(msg.header.frame_id)

    for topic in (args.left_image_topic, args.right_image_topic):
        describe_deltas(topic, image_stamps[topic])
        if image_stamps[topic] and header_stamps[topic]:
            offsets_ms = (np.array(image_stamps[topic]) - np.array(header_stamps[topic])) * 1e-6
            print(
                f"{topic}: bag-header offset ms min/mean/max="
                f"{np.min(offsets_ms):.3f}/{np.mean(offsets_ms):.3f}/{np.max(offsets_ms):.3f}"
            )
        print(f"{topic}: frame_ids={sorted(frame_ids[topic])}")

    left = header_stamps[args.left_image_topic]
    right = header_stamps[args.right_image_topic]
    pairs = min(len(left), len(right))
    if pairs:
        stereo_offsets_us = (np.sort(np.array(left[:pairs])) - np.sort(np.array(right[:pairs]))) * 1e-3
        print(
            "left-right header offset us min/mean/max="
            f"{np.min(stereo_offsets_us):.3f}/{np.mean(stereo_offsets_us):.3f}/"
            f"{np.max(stereo_offsets_us):.3f}"
        )

    for topic, info in infos.items():
        baseline = None
        if abs(info.p[0]) > 1e-12 and abs(info.p[3]) > 1e-12:
            baseline = -info.p[3] / info.p[0]
        print(f"{topic}: frame_id={info.header.frame_id}")
        print(f"{topic}: size={info.width}x{info.height} distortion_model={info.distortion_model}")
        print(f"{topic}: D={list(info.d)}")
        print(f"{topic}: P={list(info.p)}")
        if baseline is not None:
            print(f"{topic}: baseline_from_P={baseline:.6f} m")


if __name__ == "__main__":
    main()
