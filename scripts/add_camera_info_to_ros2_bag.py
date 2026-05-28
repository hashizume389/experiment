#!/usr/bin/env python3
"""Add CameraInfo topics to a ROS 2 bag from EuRoC/Kalibr-style YAML files."""

import argparse
import math
import shutil
import sqlite3
from pathlib import Path

import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosbags.interfaces import (
    Qos,
    QosDurability,
    QosHistory,
    QosLiveliness,
    QosReliability,
    QosTime,
)
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage


DEFAULT_CAMERAS = (
    {
        "image_topic": "/cam0/image_raw",
        "camera_info_topic": "/cam0/camera_info",
        "yaml_path": "config/mh01_cam0_sensor.yaml",
        "frame_id": "cam0",
    },
    {
        "image_topic": "/cam1/image_raw",
        "camera_info_topic": "/cam1/camera_info",
        "yaml_path": "config/mh01_cam1_sensor.yaml",
        "frame_id": "cam1",
    },
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Copy a ROS 2 bag and insert CameraInfo messages for stereo images."
    )
    parser.add_argument(
        "input_bag",
        nargs="?",
        default="MH_01_easy_ros2",
        help="Input ROS 2 bag directory.",
    )
    parser.add_argument(
        "output_bag",
        nargs="?",
        default="MH_01_easy_ros2_with_camera_info",
        help="Output ROS 2 bag directory to create.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove the output bag directory first if it already exists.",
    )
    parser.add_argument(
        "--cam0-yaml",
        default=DEFAULT_CAMERAS[0]["yaml_path"],
        help="Calibration YAML for /cam0/image_raw.",
    )
    parser.add_argument(
        "--cam1-yaml",
        default=DEFAULT_CAMERAS[1]["yaml_path"],
        help="Calibration YAML for /cam1/image_raw.",
    )
    parser.add_argument(
        "--cam0-info-topic",
        default=DEFAULT_CAMERAS[0]["camera_info_topic"],
        help="CameraInfo topic paired with /cam0/image_raw.",
    )
    parser.add_argument(
        "--cam1-info-topic",
        default=DEFAULT_CAMERAS[1]["camera_info_topic"],
        help="CameraInfo topic paired with /cam1/image_raw.",
    )
    parser.add_argument(
        "--base-frame",
        default="imu0",
        help="Parent frame for EuRoC camera static transforms.",
    )
    parser.add_argument(
        "--tf-static-topic",
        default="/tf_static",
        help="Static TF topic to add.",
    )
    parser.add_argument(
        "--keep-image-frame-ids",
        action="store_true",
        help="Do not rewrite image header frame_id values.",
    )
    return parser.parse_args()


def load_camera_info_template(yaml_path: Path) -> CameraInfo:
    with yaml_path.open() as f:
        data = yaml.safe_load(f)

    width, height = data["resolution"]
    fx, fy, cx, cy = data["intrinsics"]
    distortion = list(data.get("distortion_coefficients", []))
    if len(distortion) == 4:
        distortion.append(0.0)

    msg = CameraInfo()
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.d = [float(v) for v in distortion]
    msg.k = [
        float(fx),
        0.0,
        float(cx),
        0.0,
        float(fy),
        float(cy),
        0.0,
        0.0,
        1.0,
    ]
    msg.r = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    msg.p = [
        float(fx),
        0.0,
        float(cx),
        0.0,
        0.0,
        float(fy),
        float(cy),
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return msg


def make_camera_info(template: CameraInfo, image_msg, frame_id: str) -> CameraInfo:
    msg = CameraInfo()
    msg.header = image_msg.header
    msg.header.frame_id = frame_id
    msg.height = template.height
    msg.width = template.width
    msg.distortion_model = template.distortion_model
    msg.d = list(template.d)
    msg.k = list(template.k)
    msg.r = list(template.r)
    msg.p = list(template.p)
    msg.binning_x = template.binning_x
    msg.binning_y = template.binning_y
    msg.roi = template.roi
    return msg


def matrix_to_quaternion(matrix: list[float]) -> tuple[float, float, float, float]:
    m00, m01, m02 = matrix[0], matrix[1], matrix[2]
    m10, m11, m12 = matrix[4], matrix[5], matrix[6]
    m20, m21, m22 = matrix[8], matrix[9], matrix[10]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)

    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def invert_transform(matrix: list[float]) -> list[float]:
    rotation = [
        [matrix[0], matrix[1], matrix[2]],
        [matrix[4], matrix[5], matrix[6]],
        [matrix[8], matrix[9], matrix[10]],
    ]
    translation = [matrix[3], matrix[7], matrix[11]]
    inv_rotation = [[rotation[j][i] for j in range(3)] for i in range(3)]
    inv_translation = [
        -sum(inv_rotation[row][col] * translation[col] for col in range(3))
        for row in range(3)
    ]

    return [
        inv_rotation[0][0],
        inv_rotation[0][1],
        inv_rotation[0][2],
        inv_translation[0],
        inv_rotation[1][0],
        inv_rotation[1][1],
        inv_rotation[1][2],
        inv_translation[1],
        inv_rotation[2][0],
        inv_rotation[2][1],
        inv_rotation[2][2],
        inv_translation[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def compose_transforms(left: list[float], right: list[float]) -> list[float]:
    out = [0.0] * 16
    for row in range(4):
        for col in range(4):
            out[row * 4 + col] = sum(
                left[row * 4 + idx] * right[idx * 4 + col] for idx in range(4)
            )
    return out


def load_static_transform(
    matrix: list[float],
    parent_frame: str,
    child_frame: str,
    stamp_sec: int,
    stamp_nanosec: int,
) -> TransformStamped:
    qx, qy, qz, qw = matrix_to_quaternion(matrix)

    transform = TransformStamped()
    transform.header.stamp.sec = stamp_sec
    transform.header.stamp.nanosec = stamp_nanosec
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(matrix[3])
    transform.transform.translation.y = float(matrix[7])
    transform.transform.translation.z = float(matrix[11])
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def load_transform_matrix(yaml_path: Path) -> list[float]:
    with yaml_path.open() as f:
        data = yaml.safe_load(f)
    return [float(value) for value in data["T_BS"]["data"]]


def static_tf_qos() -> Qos:
    return Qos(
        QosHistory.KEEP_LAST,
        1,
        QosReliability.RELIABLE,
        QosDurability.TRANSIENT_LOCAL,
        QosTime(0, 0),
        QosTime(0, 0),
        QosLiveliness.AUTOMATIC,
        QosTime(0, 0),
        False,
    )


def qos_metadata_string(qos_profiles: list[Qos]) -> str:
    if not qos_profiles:
        return ""
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


def normalize_qos_metadata(output_bag: Path, tf_static_topic: str) -> None:
    """Write empty QoS profiles in the string form expected by ROS 2 Humble CLI."""
    metadata_path = output_bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    bag_info = metadata["rosbag2_bagfile_information"]
    tf_static_qos = qos_metadata_string([static_tf_qos()])
    for item in bag_info["topics_with_message_count"]:
        topic_metadata = item["topic_metadata"]
        topic_metadata["offered_qos_profiles"] = (
            tf_static_qos if topic_metadata["name"] == tf_static_topic else ""
        )
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))

    db_path = output_bag / bag_info["relative_file_paths"][0]
    conn = sqlite3.connect(db_path)
    try:
        conn.execute("update topics set offered_qos_profiles = ''")
        conn.execute(
            "update topics set offered_qos_profiles = ? where name = ?",
            (tf_static_qos, tf_static_topic),
        )
        row = conn.execute("select id, metadata from metadata limit 1").fetchone()
        if row:
            db_metadata = yaml.safe_load(row[1])
            for item in db_metadata["topics_with_message_count"]:
                topic_metadata = item["topic_metadata"]
                topic_metadata["offered_qos_profiles"] = (
                    tf_static_qos if topic_metadata["name"] == tf_static_topic else ""
                )
            conn.execute(
                "update metadata set metadata = ? where id = ?",
                (yaml.safe_dump(db_metadata, sort_keys=False), row[0]),
            )
        conn.commit()
    finally:
        conn.close()


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    input_bag = Path(args.input_bag).expanduser()
    output_bag = Path(args.output_bag).expanduser()

    if not input_bag.exists():
        raise SystemExit(f"Input bag does not exist: {input_bag}")
    if output_bag.exists():
        if not args.overwrite:
            raise SystemExit(f"Output bag already exists: {output_bag}")
        shutil.rmtree(output_bag)

    cameras = {
        DEFAULT_CAMERAS[0]["image_topic"]: {
            "info_topic": args.cam0_info_topic,
            "yaml_path": (root / args.cam0_yaml).resolve(),
            "template": load_camera_info_template((root / args.cam0_yaml).resolve()),
            "frame_id": DEFAULT_CAMERAS[0]["frame_id"],
        },
        DEFAULT_CAMERAS[1]["image_topic"]: {
            "info_topic": args.cam1_info_topic,
            "yaml_path": (root / args.cam1_yaml).resolve(),
            "template": load_camera_info_template((root / args.cam1_yaml).resolve()),
            "frame_id": DEFAULT_CAMERAS[1]["frame_id"],
        },
    }

    image_type = get_message("sensor_msgs/msg/Image")
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    connection_map = {}
    added_count = {camera["info_topic"]: 0 for camera in cameras.values()}

    with AnyReader([input_bag]) as reader, Writer(output_bag, version=9) as writer:
        for connection in reader.connections:
            connection_map[connection.id] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=typestore,
                serialization_format=connection.ext.serialization_format,
                offered_qos_profiles=connection.ext.offered_qos_profiles,
            )

        camera_info_connections = {
            image_topic: writer.add_connection(
                camera["info_topic"],
                "sensor_msgs/msg/CameraInfo",
                typestore=typestore,
            )
            for image_topic, camera in cameras.items()
        }
        tf_static_connection = writer.add_connection(
            args.tf_static_topic,
            "tf2_msgs/msg/TFMessage",
            typestore=typestore,
            offered_qos_profiles=[static_tf_qos()],
        )

        static_tf = TFMessage()
        stamp_sec = reader.start_time // 1_000_000_000
        stamp_nanosec = reader.start_time % 1_000_000_000
        cam0_matrix = load_transform_matrix(cameras["/cam0/image_raw"]["yaml_path"])
        cam1_matrix = load_transform_matrix(cameras["/cam1/image_raw"]["yaml_path"])
        cam0_to_cam1 = compose_transforms(invert_transform(cam0_matrix), cam1_matrix)
        static_tf.transforms = [
            load_static_transform(
                cam0_matrix,
                args.base_frame,
                "cam0",
                stamp_sec,
                stamp_nanosec,
            ),
            load_static_transform(
                cam0_to_cam1,
                "cam0",
                "cam1",
                stamp_sec,
                stamp_nanosec,
            )
        ]
        writer.write(
            tf_static_connection,
            reader.start_time,
            serialize_message(static_tf),
        )

        for connection, timestamp_ns, rawdata in reader.messages():
            if connection.topic in cameras and not args.keep_image_frame_ids:
                image_msg = deserialize_message(bytes(rawdata), image_type)
                image_msg.header.frame_id = cameras[connection.topic]["frame_id"]
                output_rawdata = serialize_message(image_msg)
            else:
                output_rawdata = rawdata

            writer.write(connection_map[connection.id], timestamp_ns, output_rawdata)

            if connection.topic not in cameras:
                continue

            camera = cameras[connection.topic]
            image_msg = deserialize_message(bytes(output_rawdata), image_type)
            camera_info = make_camera_info(
                camera["template"],
                image_msg,
                camera["frame_id"],
            )
            writer.write(
                camera_info_connections[connection.topic],
                timestamp_ns,
                serialize_message(camera_info),
            )
            added_count[camera["info_topic"]] += 1

    normalize_qos_metadata(output_bag, args.tf_static_topic)

    print(f"Created: {output_bag}")
    for topic, count in added_count.items():
        print(f"Added {count} messages to {topic}")
    print(f"Added 1 message to {args.tf_static_topic}")


if __name__ == "__main__":
    main()
