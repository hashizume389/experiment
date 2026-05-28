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


@dataclass(frozen=True)
class GroundTruthSample:
    timestamp_ns: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class GroundTruthMatch:
    timestamp_ns: int
    x: float
    y: float
    z: float
    dt_sec: float
    mode: str


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
            required_columns = {'timestamp_ns', 'x_m', 'y_m', 'z_m'}
            missing = required_columns - set(reader.fieldnames or [])
            if missing:
                missing_text = ', '.join(sorted(missing))
                raise ValueError(f'ground truth CSV is missing columns: {missing_text}')

            for row in reader:
                samples.append(
                    GroundTruthSample(
                        timestamp_ns=int(row['timestamp_ns']),
                        x=float(row['x_m']),
                        y=float(row['y_m']),
                        z=float(row['z_m']),
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
        )


class Mh01EvalLogger(Node):
    def __init__(self):
        super().__init__('mh01_eval_logger')

        self.declare_parameter(
            'ground_truth_csv',
            '/home/hashizume/experiment/MH_01_easy_leica_position.csv',
        )
        self.declare_parameter('odom_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('output_dir', '/home/hashizume/experiment/results')
        self.declare_parameter('output_filename', '')
        self.declare_parameter('max_match_dt_sec', 0.1)

        ground_truth_csv = Path(self.get_parameter('ground_truth_csv').value).expanduser()
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        output_filename = str(self.get_parameter('output_filename').value)
        self.max_match_dt_sec = float(self.get_parameter('max_match_dt_sec').value)

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
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        first = self.ground_truth.samples[0].timestamp_ns
        last = self.ground_truth.samples[-1].timestamp_ns
        self.get_logger().info(f'ground truth CSV: {ground_truth_csv}')
        self.get_logger().info(f'ground truth samples: {len(self.ground_truth.samples)}')
        self.get_logger().info(f'ground truth time range [ns]: {first} - {last}')
        self.get_logger().info(f'odometry topic: {self.odom_topic}')
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
