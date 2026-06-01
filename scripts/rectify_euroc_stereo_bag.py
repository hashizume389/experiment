#!/usr/bin/env python3
"""Create a rectified EuRoC stereo ROS 2 bag for Isaac ROS Visual SLAM."""

import argparse
import math
import shutil
import sqlite3
from pathlib import Path

import cv2
import numpy as np
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
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


CAM0_TOPIC = "/cam0/image_raw"
CAM1_TOPIC = "/cam1/image_raw"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rectify EuRoC stereo images and add rectified CameraInfo topics."
    )
    parser.add_argument("input_bag", nargs="?", default="MH_01_easy_ros2")
    parser.add_argument(
        "output_bag",
        nargs="?",
        default="MH_01_easy_ros2_rectified",
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--cam0-yaml", default="config/mh01_cam0_sensor.yaml")
    parser.add_argument("--cam1-yaml", default="config/mh01_cam1_sensor.yaml")
    parser.add_argument("--left-image-topic", default="/cam0/image_rect")
    parser.add_argument("--right-image-topic", default="/cam1/image_rect")
    parser.add_argument("--left-info-topic", default="/cam0/camera_info")
    parser.add_argument("--right-info-topic", default="/cam1/camera_info")
    parser.add_argument("--left-frame-id", default="cam0_rect")
    parser.add_argument("--right-frame-id", default="cam1_rect")
    parser.add_argument("--base-frame", default="imu0")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument(
        "--copy-raw",
        action="store_true",
        help="Also copy original raw image topics into the output bag.",
    )
    return parser.parse_args()


def load_calibration(path: Path) -> dict:
    with path.open() as f:
        data = yaml.safe_load(f)
    width, height = data["resolution"]
    fx, fy, cx, cy = data["intrinsics"]
    distortion = list(data.get("distortion_coefficients", []))
    if len(distortion) == 4:
        distortion.append(0.0)
    return {
        "width": int(width),
        "height": int(height),
        "k": np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64),
        "d": np.array(distortion, dtype=np.float64),
        "t_bs": np.array(data["T_BS"]["data"], dtype=np.float64).reshape(4, 4),
    }


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = matrix[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ matrix[:3, 3]
    return out


def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        )
    if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return (
            0.25 * s,
            (rotation[0, 1] + rotation[1, 0]) / s,
            (rotation[0, 2] + rotation[2, 0]) / s,
            (rotation[2, 1] - rotation[1, 2]) / s,
        )
    if rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return (
            (rotation[0, 1] + rotation[1, 0]) / s,
            0.25 * s,
            (rotation[1, 2] + rotation[2, 1]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
        )
    s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return (
        (rotation[0, 2] + rotation[2, 0]) / s,
        (rotation[1, 2] + rotation[2, 1]) / s,
        0.25 * s,
        (rotation[1, 0] - rotation[0, 1]) / s,
    )


def make_tf(
    parent: str,
    child: str,
    transform: np.ndarray,
    stamp_sec: int,
    stamp_nanosec: int,
) -> TransformStamped:
    qx, qy, qz, qw = matrix_to_quaternion(transform[:3, :3])
    msg = TransformStamped()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nanosec
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(transform[0, 3])
    msg.transform.translation.y = float(transform[1, 3])
    msg.transform.translation.z = float(transform[2, 3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def make_camera_info(
    image_msg: Image,
    frame_id: str,
    projection: np.ndarray,
    rectification: np.ndarray,
) -> CameraInfo:
    info = CameraInfo()
    info.header = image_msg.header
    info.header.frame_id = frame_id
    info.height = image_msg.height
    info.width = image_msg.width
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [
        float(projection[0, 0]),
        0.0,
        float(projection[0, 2]),
        0.0,
        float(projection[1, 1]),
        float(projection[1, 2]),
        0.0,
        0.0,
        1.0,
    ]
    info.r = [float(v) for v in rectification.reshape(-1)]
    info.p = [float(v) for v in projection.reshape(-1)]
    return info


def image_to_array(msg: Image) -> np.ndarray:
    if msg.encoding not in ("mono8", "8UC1"):
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return data.reshape(msg.height, msg.step)[:, : msg.width]


def array_to_image(array: np.ndarray, source: Image, frame_id: str) -> Image:
    msg = Image()
    msg.header = source.header
    msg.header.frame_id = frame_id
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = "mono8"
    msg.is_bigendian = source.is_bigendian
    msg.step = int(array.shape[1])
    msg.data = array.astype(np.uint8, copy=False).reshape(-1).tobytes()
    return msg


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


def qos_metadata_string() -> str:
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
    metadata_path = output_bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    bag_info = metadata["rosbag2_bagfile_information"]
    tf_static_qos = qos_metadata_string()
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

    cam0 = load_calibration((root / args.cam0_yaml).resolve())
    cam1 = load_calibration((root / args.cam1_yaml).resolve())
    image_size = (cam0["width"], cam0["height"])

    t_c0_c1 = invert_transform(cam0["t_bs"]) @ cam1["t_bs"]
    t_c1_c0 = invert_transform(t_c0_c1)
    r_c1_c0 = t_c1_c0[:3, :3]
    trans_c1_c0 = t_c1_c0[:3, 3]

    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
        cam0["k"],
        cam0["d"],
        cam1["k"],
        cam1["d"],
        image_size,
        r_c1_c0,
        trans_c1_c0,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0.0,
    )
    map0_x, map0_y = cv2.initUndistortRectifyMap(
        cam0["k"], cam0["d"], r1, p1[:3, :3], image_size, cv2.CV_32FC1
    )
    map1_x, map1_y = cv2.initUndistortRectifyMap(
        cam1["k"], cam1["d"], r2, p2[:3, :3], image_size, cv2.CV_32FC1
    )

    image_type = get_message("sensor_msgs/msg/Image")
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    connection_map = {}
    counts = {args.left_image_topic: 0, args.right_image_topic: 0}

    with AnyReader([input_bag]) as reader, Writer(output_bag, version=9) as writer:
        for connection in reader.connections:
            if not args.copy_raw and connection.topic in (CAM0_TOPIC, CAM1_TOPIC):
                continue
            connection_map[connection.id] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=typestore,
                serialization_format=connection.ext.serialization_format,
                offered_qos_profiles=connection.ext.offered_qos_profiles,
            )

        left_image_conn = writer.add_connection(args.left_image_topic, "sensor_msgs/msg/Image", typestore=typestore)
        right_image_conn = writer.add_connection(args.right_image_topic, "sensor_msgs/msg/Image", typestore=typestore)
        left_info_conn = writer.add_connection(args.left_info_topic, "sensor_msgs/msg/CameraInfo", typestore=typestore)
        right_info_conn = writer.add_connection(args.right_info_topic, "sensor_msgs/msg/CameraInfo", typestore=typestore)
        tf_conn = writer.add_connection(
            args.tf_static_topic,
            "tf2_msgs/msg/TFMessage",
            typestore=typestore,
            offered_qos_profiles=[static_tf_qos()],
        )

        stamp_sec = reader.start_time // 1_000_000_000
        stamp_nanosec = reader.start_time % 1_000_000_000
        t_b_cam0_rect = cam0["t_bs"].copy()
        t_b_cam0_rect[:3, :3] = cam0["t_bs"][:3, :3] @ r1.T
        t_b_cam1_rect = cam1["t_bs"].copy()
        t_b_cam1_rect[:3, :3] = cam1["t_bs"][:3, :3] @ r2.T
        t_cam0rect_cam1rect = invert_transform(t_b_cam0_rect) @ t_b_cam1_rect

        static_tf = TFMessage()
        static_tf.transforms = [
            make_tf(args.base_frame, args.left_frame_id, t_b_cam0_rect, stamp_sec, stamp_nanosec),
            make_tf(args.left_frame_id, args.right_frame_id, t_cam0rect_cam1rect, stamp_sec, stamp_nanosec),
        ]
        writer.write(tf_conn, reader.start_time, serialize_message(static_tf))

        for connection, timestamp_ns, rawdata in reader.messages():
            if connection.id in connection_map:
                writer.write(connection_map[connection.id], timestamp_ns, rawdata)

            if connection.topic not in (CAM0_TOPIC, CAM1_TOPIC):
                continue

            source = deserialize_message(bytes(rawdata), image_type)
            source_array = image_to_array(source)
            if connection.topic == CAM0_TOPIC:
                rectified = cv2.remap(source_array, map0_x, map0_y, cv2.INTER_LINEAR)
                image_msg = array_to_image(rectified, source, args.left_frame_id)
                info_msg = make_camera_info(image_msg, args.left_frame_id, p1, r1)
                writer.write(left_image_conn, timestamp_ns, serialize_message(image_msg))
                writer.write(left_info_conn, timestamp_ns, serialize_message(info_msg))
                counts[args.left_image_topic] += 1
            else:
                rectified = cv2.remap(source_array, map1_x, map1_y, cv2.INTER_LINEAR)
                image_msg = array_to_image(rectified, source, args.right_frame_id)
                info_msg = make_camera_info(image_msg, args.right_frame_id, p2, r2)
                writer.write(right_image_conn, timestamp_ns, serialize_message(image_msg))
                writer.write(right_info_conn, timestamp_ns, serialize_message(info_msg))
                counts[args.right_image_topic] += 1

    normalize_qos_metadata(output_bag, args.tf_static_topic)
    baseline_m = abs(float(p2[0, 3] / p2[0, 0]))
    print(f"Created: {output_bag}")
    print(f"Rectified {counts[args.left_image_topic]} left images to {args.left_image_topic}")
    print(f"Rectified {counts[args.right_image_topic]} right images to {args.right_image_topic}")
    print(f"Rectified stereo baseline from P2: {baseline_m:.6f} m")
    print(f"Left P: {p1.reshape(-1).tolist()}")
    print(f"Right P: {p2.reshape(-1).tolist()}")


if __name__ == "__main__":
    main()
