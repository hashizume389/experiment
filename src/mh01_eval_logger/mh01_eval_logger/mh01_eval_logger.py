#!/usr/bin/env python3
from __future__ import annotations

import bisect
import csv
import datetime
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class GroundTruthSample:
    timestamp_ns: int
    x: float
    y: float
    z: float
    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    qw: Optional[float] = None


@dataclass(frozen=True)
class GroundTruthMatch:
    timestamp_ns: int
    x: float
    y: float
    z: float
    dt_sec: float
    mode: str
    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    qw: Optional[float] = None

    @property
    def has_orientation(self) -> bool:
        return None not in (self.qx, self.qy, self.qz, self.qw)


class GroundTruthTrack:
    def __init__(self, samples: list[GroundTruthSample]):
        if not samples:
            raise ValueError('ground truth CSV has no samples')

        self.samples = sorted(samples, key=lambda sample: sample.timestamp_ns)
        self.timestamps_ns = [sample.timestamp_ns for sample in self.samples]

    @classmethod
    def from_csv(cls, csv_path: Path) -> 'GroundTruthTrack':
        samples: list[GroundTruthSample] = []

        with csv_path.open(newline='', encoding='utf-8') as csv_file:
            reader = csv.DictReader(csv_file)
            fieldnames = reader.fieldnames or []
            columns = {name.strip(): name for name in fieldnames}
            is_simple_position_csv = {'timestamp_ns', 'x_m', 'y_m', 'z_m'} <= set(columns)
            euroc_timestamp_column = (
                columns.get('#timestamp [ns]')
                or columns.get('#timestamp')
            )
            is_euroc_pose_csv = (
                euroc_timestamp_column is not None
                and 'p_RS_R_x [m]' in columns
                and 'p_RS_R_y [m]' in columns
                and 'p_RS_R_z [m]' in columns
                and 'q_RS_w []' in columns
                and 'q_RS_x []' in columns
                and 'q_RS_y []' in columns
                and 'q_RS_z []' in columns
            )
            if not is_simple_position_csv and not is_euroc_pose_csv:
                raise ValueError(
                    'ground truth CSV must be either the exported position format '
                    '(timestamp_ns,x_m,y_m,z_m) or EuRoC state_groundtruth_estimate0/data.csv'
                )

            for row in reader:
                if is_euroc_pose_csv:
                    samples.append(
                        GroundTruthSample(
                            timestamp_ns=int(row[euroc_timestamp_column]),
                            x=float(row[columns['p_RS_R_x [m]']]),
                            y=float(row[columns['p_RS_R_y [m]']]),
                            z=float(row[columns['p_RS_R_z [m]']]),
                            qx=float(row[columns['q_RS_x []']]),
                            qy=float(row[columns['q_RS_y []']]),
                            qz=float(row[columns['q_RS_z []']]),
                            qw=float(row[columns['q_RS_w []']]),
                        )
                    )
                    continue

                samples.append(
                    GroundTruthSample(
                        timestamp_ns=int(row[columns['timestamp_ns']]),
                        x=float(row[columns['x_m']]),
                        y=float(row[columns['y_m']]),
                        z=float(row[columns['z_m']]),
                    )
                )

        return cls(samples)

    def match(self, timestamp_ns: int) -> GroundTruthMatch:
        index = bisect.bisect_left(self.timestamps_ns, timestamp_ns)

        if index <= 0:
            sample = self.samples[0]
            return self._nearest_match(sample, timestamp_ns)

        if index >= len(self.samples):
            sample = self.samples[-1]
            return self._nearest_match(sample, timestamp_ns)

        before = self.samples[index - 1]
        after = self.samples[index]
        span_ns = after.timestamp_ns - before.timestamp_ns

        if span_ns <= 0:
            nearest = min(
                (before, after),
                key=lambda sample: abs(sample.timestamp_ns - timestamp_ns),
            )
            return self._nearest_match(nearest, timestamp_ns)

        ratio = (timestamp_ns - before.timestamp_ns) / span_ns
        x = before.x + (after.x - before.x) * ratio
        y = before.y + (after.y - before.y) * ratio
        z = before.z + (after.z - before.z) * ratio
        qx = qy = qz = qw = None
        if self._has_orientation(before) and self._has_orientation(after):
            qx, qy, qz, qw = self.slerp(
                (before.qx, before.qy, before.qz, before.qw),
                (after.qx, after.qy, after.qz, after.qw),
                ratio,
            )
        nearest_dt_ns = min(
            abs(timestamp_ns - before.timestamp_ns),
            abs(after.timestamp_ns - timestamp_ns),
        )

        return GroundTruthMatch(
            timestamp_ns=timestamp_ns,
            x=x,
            y=y,
            z=z,
            dt_sec=nearest_dt_ns * 1e-9,
            mode='interpolated',
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    @staticmethod
    def _nearest_match(sample: GroundTruthSample, timestamp_ns: int) -> GroundTruthMatch:
        return GroundTruthMatch(
            timestamp_ns=sample.timestamp_ns,
            x=sample.x,
            y=sample.y,
            z=sample.z,
            dt_sec=(timestamp_ns - sample.timestamp_ns) * 1e-9,
            mode='nearest',
            qx=sample.qx,
            qy=sample.qy,
            qz=sample.qz,
            qw=sample.qw,
        )

    @staticmethod
    def _has_orientation(sample: GroundTruthSample) -> bool:
        return None not in (sample.qx, sample.qy, sample.qz, sample.qw)

    @staticmethod
    def normalize_quaternion(
        quat: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        x, y, z, w = quat
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 0.0:
            return 0.0, 0.0, 0.0, 1.0
        return x / norm, y / norm, z / norm, w / norm

    @classmethod
    def slerp(
        cls,
        q0: tuple[float, float, float, float],
        q1: tuple[float, float, float, float],
        ratio: float,
    ) -> tuple[float, float, float, float]:
        x0, y0, z0, w0 = cls.normalize_quaternion(q0)
        x1, y1, z1, w1 = cls.normalize_quaternion(q1)
        dot = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1
        if dot < 0.0:
            x1, y1, z1, w1 = -x1, -y1, -z1, -w1
            dot = -dot
        if dot > 0.9995:
            return cls.normalize_quaternion((
                x0 + ratio * (x1 - x0),
                y0 + ratio * (y1 - y0),
                z0 + ratio * (z1 - z0),
                w0 + ratio * (w1 - w0),
            ))

        theta_0 = math.acos(max(-1.0, min(1.0, dot)))
        theta = theta_0 * ratio
        sin_theta = math.sin(theta)
        sin_theta_0 = math.sin(theta_0)
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return (
            s0 * x0 + s1 * x1,
            s0 * y0 + s1 * y1,
            s0 * z0 + s1 * z1,
            s0 * w0 + s1 * w1,
        )


class Mh01EvalLogger(Node):
    def __init__(self):
        super().__init__('mh01_eval_logger')

        self.declare_parameter(
            'ground_truth_csv',
            '/home/hashizume/experiment/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv',
        )
        self.declare_parameter('odom_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('output_dir', '/home/hashizume/experiment/results')
        self.declare_parameter('output_filename', '')
        self.declare_parameter('max_match_dt_sec', 0.1)
        self.declare_parameter('qos_reliability', 'reliable')

        ground_truth_csv = Path(self.get_parameter('ground_truth_csv').value).expanduser()
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        output_filename = str(self.get_parameter('output_filename').value)
        self.max_match_dt_sec = float(self.get_parameter('max_match_dt_sec').value)
        qos_reliability = str(self.get_parameter('qos_reliability').value)

        self.ground_truth = GroundTruthTrack.from_csv(ground_truth_csv)
        self.csv_file = None
        self.csv_writer: Optional[csv.writer] = None
        self.message_count = 0
        self.warned_time_range = False
        self.first_odom_position = None
        self.first_gt_position = None

        output_dir.mkdir(parents=True, exist_ok=True)
        if not output_filename:
            now = datetime.datetime.now()
            output_filename = f'mh01_eval_log_{now.strftime("%Y%m%d_%H%M%S")}.csv'
        self.output_path = output_dir / output_filename

        self.open_output_csv()
        qos_profile = self.make_qos_profile(qos_reliability)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile,
        )
        self.create_timer(5.0, self.status_timer_callback)

        first = self.ground_truth.samples[0].timestamp_ns
        last = self.ground_truth.samples[-1].timestamp_ns
        self.get_logger().info(f'ground truth CSV: {ground_truth_csv}')
        self.get_logger().info(f'ground truth samples: {len(self.ground_truth.samples)}')
        self.get_logger().info(f'ground truth time range [ns]: {first} - {last}')
        self.get_logger().info(f'odometry topic: {self.odom_topic}')
        self.get_logger().info(f'odometry QoS reliability: {qos_reliability}')
        self.get_logger().info(f'output CSV: {self.output_path}')

    def open_output_csv(self) -> None:
        self.csv_file = self.output_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'odom_timestamp_ns',
            'odom_stamp_sec',
            'odom_stamp_nanosec',
            'odom_frame_id',
            'odom_child_frame_id',
            'odom_x_m',
            'odom_y_m',
            'odom_z_m',
            'odom_qx',
            'odom_qy',
            'odom_qz',
            'odom_qw',
            'odom_yaw_rad',
            'gt_timestamp_ns',
            'gt_match_mode',
            'gt_dt_sec',
            'gt_x_m',
            'gt_y_m',
            'gt_z_m',
            'gt_qx',
            'gt_qy',
            'gt_qz',
            'gt_qw',
            'orientation_error_deg',
            'error_x_m',
            'error_y_m',
            'error_z_m',
            'error_2d_m',
            'error_3d_m',
            'odom_rel_x_m',
            'odom_rel_y_m',
            'odom_rel_z_m',
            'gt_rel_x_m',
            'gt_rel_y_m',
            'gt_rel_z_m',
            'rel_error_x_m',
            'rel_error_y_m',
            'rel_error_z_m',
            'rel_error_2d_m',
            'rel_error_3d_m',
        ])

    def status_timer_callback(self) -> None:
        publisher_count = self.count_publishers(self.odom_topic)
        if self.message_count > 0:
            self.get_logger().info(
                f'logging {self.message_count} odometry messages from {self.odom_topic}',
                throttle_duration_sec=30.0,
            )
            return

        topics = dict(self.get_topic_names_and_types())
        if self.odom_topic in topics:
            self.get_logger().warn(
                (
                    f'no odometry received yet from {self.odom_topic}; '
                    f'publishers={publisher_count}. If publishers > 0, try '
                    'qos_reliability:=best_effort.'
                ),
                throttle_duration_sec=10.0,
            )
        else:
            odom_like_topics = sorted(
                topic for topic in topics
                if 'odom' in topic.lower() or 'visual_slam' in topic.lower()
            )
            suffix = f' candidates={odom_like_topics}' if odom_like_topics else ''
            self.get_logger().warn(
                f'waiting for odometry topic {self.odom_topic}.{suffix}',
                throttle_duration_sec=10.0,
            )

    def odom_callback(self, msg: Odometry) -> None:
        timestamp_ns = self.stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        gt = self.ground_truth.match(timestamp_ns)

        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        yaw = self.yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        error_x = position.x - gt.x
        error_y = position.y - gt.y
        error_z = position.z - gt.z
        error_2d = math.hypot(error_x, error_y)
        error_3d = math.sqrt(error_x * error_x + error_y * error_y + error_z * error_z)
        orientation_error_deg = ''
        if gt.has_orientation:
            orientation_error_deg = self.quaternion_angle_deg((
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ), (
                gt.qx,
                gt.qy,
                gt.qz,
                gt.qw,
            ))

        if self.first_odom_position is None:
            self.first_odom_position = (position.x, position.y, position.z)
            self.first_gt_position = (gt.x, gt.y, gt.z)

        odom_rel_x = position.x - self.first_odom_position[0]
        odom_rel_y = position.y - self.first_odom_position[1]
        odom_rel_z = position.z - self.first_odom_position[2]
        gt_rel_x = gt.x - self.first_gt_position[0]
        gt_rel_y = gt.y - self.first_gt_position[1]
        gt_rel_z = gt.z - self.first_gt_position[2]
        rel_error_x = odom_rel_x - gt_rel_x
        rel_error_y = odom_rel_y - gt_rel_y
        rel_error_z = odom_rel_z - gt_rel_z
        rel_error_2d = math.hypot(rel_error_x, rel_error_y)
        rel_error_3d = math.sqrt(
            rel_error_x * rel_error_x
            + rel_error_y * rel_error_y
            + rel_error_z * rel_error_z
        )

        if self.csv_writer is None:
            return

        self.csv_writer.writerow([
            timestamp_ns,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.header.frame_id,
            msg.child_frame_id,
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            yaw,
            gt.timestamp_ns,
            gt.mode,
            gt.dt_sec,
            gt.x,
            gt.y,
            gt.z,
            gt.qx if gt.qx is not None else '',
            gt.qy if gt.qy is not None else '',
            gt.qz if gt.qz is not None else '',
            gt.qw if gt.qw is not None else '',
            orientation_error_deg,
            error_x,
            error_y,
            error_z,
            error_2d,
            error_3d,
            odom_rel_x,
            odom_rel_y,
            odom_rel_z,
            gt_rel_x,
            gt_rel_y,
            gt_rel_z,
            rel_error_x,
            rel_error_y,
            rel_error_z,
            rel_error_2d,
            rel_error_3d,
        ])
        self.message_count += 1

        if abs(gt.dt_sec) > self.max_match_dt_sec:
            self.get_logger().warn(
                f'ground truth time gap is {gt.dt_sec:.3f}s at odom timestamp {timestamp_ns}',
                throttle_duration_sec=5.0,
            )

        if self.message_count % 200 == 0:
            self.csv_file.flush()
            self.get_logger().info(f'logged {self.message_count} odometry messages')

    def close(self) -> None:
        if self.csv_file:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()
            self.get_logger().info(
                f'closed {self.output_path} ({self.message_count} odometry messages)'
            )
            self.csv_file = None

    @staticmethod
    def stamp_to_ns(sec: int, nanosec: int) -> int:
        return int(sec) * 1_000_000_000 + int(nanosec)

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def quaternion_angle_deg(
        q0: tuple[float, float, float, float],
        q1: tuple[float, float, float, float],
    ) -> float:
        x0, y0, z0, w0 = GroundTruthTrack.normalize_quaternion(q0)
        x1, y1, z1, w1 = GroundTruthTrack.normalize_quaternion(q1)
        dot = abs(x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1)
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(2.0 * math.acos(dot))

    @staticmethod
    def make_qos_profile(qos_reliability: str) -> QoSProfile:
        normalized = qos_reliability.strip().lower()
        if normalized in ('best_effort', 'besteffort', 'best-effort'):
            reliability = ReliabilityPolicy.BEST_EFFORT
        elif normalized == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
        else:
            raise ValueError(
                "qos_reliability must be 'reliable' or 'best_effort'"
            )
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Mh01EvalLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('keyboard interrupt, shutting down')
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
