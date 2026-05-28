#!/usr/bin/env python3
"""Export /leica/position from a ROS2 bag to CSV."""

import argparse
import csv
from pathlib import Path

from rosbags.highlevel import AnyReader


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export geometry_msgs/msg/PointStamped messages from /leica/position."
    )
    parser.add_argument(
        "bag",
        nargs="?",
        default="/home/hashizume/experiment/MH_01_easy_ros2",
        help="Path to a ROS2 bag directory.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="/home/hashizume/experiment/MH_01_easy_leica_position.csv",
        help="Output CSV path.",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default="/leica/position",
        help="PointStamped topic to export.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bag_path = Path(args.bag)
    output_path = Path(args.output)

    if not bag_path.exists():
        raise SystemExit(f"Bag path does not exist: {bag_path}")

    count = 0
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with AnyReader([bag_path]) as reader, output_path.open("w", newline="") as csvfile:
        connections = [conn for conn in reader.connections if conn.topic == args.topic]
        if not connections:
            available = "\n".join(sorted({conn.topic for conn in reader.connections}))
            raise SystemExit(
                f"Topic not found: {args.topic}\nAvailable topics:\n{available}"
            )

        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "timestamp_ns",
                "stamp_sec",
                "stamp_nanosec",
                "frame_id",
                "x_m",
                "y_m",
                "z_m",
            ]
        )

        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            writer.writerow(
                [
                    timestamp_ns,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                    msg.header.frame_id,
                    msg.point.x,
                    msg.point.y,
                    msg.point.z,
                ]
            )
            count += 1

    print(f"Exported {count} messages to {output_path}")


if __name__ == "__main__":
    main()
