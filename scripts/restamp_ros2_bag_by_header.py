#!/usr/bin/env python3
"""Rewrite ROS 2 bag message timestamps from message header stamps when available."""

import argparse
import shutil
import sqlite3
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Copy a ROS 2 bag and use msg.header.stamp as the bag timestamp."
    )
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--topics",
        nargs="*",
        default=[],
        help="Only restamp these topics. If omitted, restamp all messages with headers.",
    )
    return parser.parse_args()


def tf_static_qos_metadata() -> str:
    return "\n".join(
        [
            "- history: 3",
            "  depth: 0",
            "  reliability: 1",
            "  durability: 1",
            "  deadline:",
            "    sec: 9223372036",
            "    nsec: 854775807",
            "  lifespan:",
            "    sec: 9223372036",
            "    nsec: 854775807",
            "  liveliness: 1",
            "  liveliness_lease_duration:",
            "    sec: 9223372036",
            "    nsec: 854775807",
            "  avoid_ros_namespace_conventions: false",
        ]
    )


def normalize_qos_metadata(output_bag: Path) -> None:
    metadata_path = output_bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    bag_info = metadata["rosbag2_bagfile_information"]
    tf_static_qos = tf_static_qos_metadata()
    for item in bag_info["topics_with_message_count"]:
        topic_metadata = item["topic_metadata"]
        topic_metadata["offered_qos_profiles"] = (
            tf_static_qos if topic_metadata["name"] == "/tf_static" else ""
        )
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))

    db_path = output_bag / bag_info["relative_file_paths"][0]
    conn = sqlite3.connect(db_path)
    try:
        conn.execute("update topics set offered_qos_profiles = ''")
        conn.execute(
            "update topics set offered_qos_profiles = ? where name = ?",
            (tf_static_qos, "/tf_static"),
        )
        conn.commit()
    finally:
        conn.close()


def header_stamp_ns(msg) -> int | None:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main() -> None:
    args = parse_args()
    input_bag = Path(args.input_bag).expanduser()
    output_bag = Path(args.output_bag).expanduser()
    if not input_bag.exists():
        raise SystemExit(f"Input bag does not exist: {input_bag}")
    if output_bag.exists():
        if not args.overwrite:
            raise SystemExit(f"Output bag already exists: {output_bag}")
        shutil.rmtree(output_bag)

    topics = set(args.topics)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    connection_map = {}
    message_types = {}
    restamped = 0

    with AnyReader([input_bag]) as reader, Writer(output_bag, version=9) as writer:
        for connection in reader.connections:
            connection_map[connection.id] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=typestore,
                serialization_format=connection.ext.serialization_format,
                offered_qos_profiles=connection.ext.offered_qos_profiles,
            )
            message_types[connection.id] = get_message(connection.msgtype)

        for connection, timestamp_ns, rawdata in reader.messages():
            output_timestamp_ns = timestamp_ns
            should_try = not topics or connection.topic in topics
            if should_try:
                try:
                    msg = deserialize_message(bytes(rawdata), message_types[connection.id])
                    stamp_ns = header_stamp_ns(msg)
                except Exception:
                    stamp_ns = None
                if stamp_ns is not None:
                    output_timestamp_ns = stamp_ns
                    restamped += 1

            writer.write(connection_map[connection.id], output_timestamp_ns, rawdata)

    normalize_qos_metadata(output_bag)
    print(f"Created: {output_bag}")
    print(f"Restamped messages: {restamped}")


if __name__ == "__main__":
    main()
