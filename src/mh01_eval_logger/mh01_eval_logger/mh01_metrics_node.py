#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from mh01_eval_logger.mh01_eval_logger import GroundTruthTrack


@dataclass(frozen=True)
class RelativeError:
    rte_percent: float
    re_deg: float


@dataclass(frozen=True)
class EvaluationResult:
    label: str
    sample_count: int
    relative_pair_count: int
    avg_rte_percent: float
    avg_re_deg: float
    rmse_ape_m: float
    mean_ape_m: float
    median_ape_m: float
    max_ape_m: float
    rmse_x_m: float
    rmse_y_m: float
    rmse_z_m: float
    mean_abs_x_m: float
    mean_abs_y_m: float
    mean_abs_z_m: float
    max_abs_x_m: float
    max_abs_y_m: float
    max_abs_z_m: float
    trajectory_length_m: float
    alignment_scale: float
    uses_orientation: bool


@dataclass(frozen=True)
class MetricsOutputs:
    results: list[EvaluationResult]
    output_summary_csv: Path
    output_plot_png: Optional[Path]
    jump_count: int
    first_jump_sec: Optional[float]


class Mh01MetricsNode(Node):
    def __init__(self):
        super().__init__('mh01_metrics_node')

        self.declare_parameter(
            'ground_truth_csv',
            '/home/hashizume/experiment/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv',
        )
        self.declare_parameter(
            'eval_log_csv',
            '/home/hashizume/experiment/results/mh01_eval_log_20260528_150755.csv',
        )
        self.declare_parameter('output_dir', '/home/hashizume/experiment/results')
        self.declare_parameter('relative_delta', 1.0)
        self.declare_parameter('relative_delta_unit', 'm')
        self.declare_parameter('relative_window_sec', 1.0)
        self.declare_parameter('alignment', 'sim3')
        self.declare_parameter('min_relative_distance_m', 0.1)
        self.declare_parameter('max_match_dt_sec', 0.05)
        self.declare_parameter('segment_end_sec', 50.0)
        self.declare_parameter('jump_threshold_m', 0.5)
        self.declare_parameter('plot', True)

        ground_truth_csv = Path(
            str(self.get_parameter('ground_truth_csv').value)
        ).expanduser()
        eval_log_csv = Path(str(self.get_parameter('eval_log_csv').value)).expanduser()
        output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        relative_delta = float(self.get_parameter('relative_delta').value)
        relative_delta_unit = str(self.get_parameter('relative_delta_unit').value).strip().lower()
        relative_window_sec = float(self.get_parameter('relative_window_sec').value)
        alignment = str(self.get_parameter('alignment').value).strip().lower()
        min_relative_distance_m = float(self.get_parameter('min_relative_distance_m').value)
        max_match_dt_sec = float(self.get_parameter('max_match_dt_sec').value)
        segment_end_sec = float(self.get_parameter('segment_end_sec').value)
        jump_threshold_m = float(self.get_parameter('jump_threshold_m').value)
        plot = bool(self.get_parameter('plot').value)

        outputs = self.evaluate(
            ground_truth_csv=ground_truth_csv,
            eval_log_csv=eval_log_csv,
            output_dir=output_dir,
            relative_delta=relative_delta,
            relative_delta_unit=relative_delta_unit,
            relative_window_sec=relative_window_sec,
            alignment=alignment,
            min_relative_distance_m=min_relative_distance_m,
            max_match_dt_sec=max_match_dt_sec,
            segment_end_sec=segment_end_sec,
            jump_threshold_m=jump_threshold_m,
            plot=plot,
        )
        self.log_result(outputs)

    def evaluate(
        self,
        ground_truth_csv: Path,
        eval_log_csv: Path,
        output_dir: Path,
        relative_delta: float,
        relative_delta_unit: str,
        relative_window_sec: float,
        alignment: str,
        min_relative_distance_m: float,
        max_match_dt_sec: float,
        segment_end_sec: float,
        jump_threshold_m: float,
        plot: bool,
    ) -> MetricsOutputs:
        ground_truth = GroundTruthTrack.from_csv(ground_truth_csv)
        timestamps_ns, odom_points, odom_quats, gt_points, gt_quats = self.load_trajectories(
            eval_log_csv,
            ground_truth,
            max_match_dt_sec,
        )

        full_result, aligned_odom_points, ape_errors = self.compute_segment_result(
            'full',
            timestamps_ns,
            odom_points,
            odom_quats,
            gt_points,
            gt_quats,
            relative_delta,
            relative_delta_unit,
            relative_window_sec,
            min_relative_distance_m,
            alignment,
        )
        results = [full_result]

        segment_mask = self.segment_mask(timestamps_ns, segment_end_sec)
        if int(np.count_nonzero(segment_mask)) >= 3:
            segment_result, _, _ = self.compute_segment_result(
                f'first_{segment_end_sec:g}s',
                timestamps_ns[segment_mask],
                odom_points[segment_mask],
                odom_quats[segment_mask] if odom_quats is not None else None,
                gt_points[segment_mask],
                gt_quats[segment_mask] if gt_quats is not None else None,
                relative_delta,
                relative_delta_unit,
                relative_window_sec,
                min_relative_distance_m,
                alignment,
            )
            results.append(segment_result)
        else:
            self.get_logger().warn(
                f'segment_end_sec={segment_end_sec:g}s has fewer than 3 samples'
            )

        jumps = self.detect_odometry_jumps(timestamps_ns, odom_points, gt_points, jump_threshold_m)
        first_jump_sec = jumps[0][0] if jumps else None

        output_dir.mkdir(parents=True, exist_ok=True)
        summary_csv = output_dir / f'{eval_log_csv.stem}_metrics_summary.csv'
        plot_png = output_dir / f'{eval_log_csv.stem}_metrics_plot.png' if plot else None

        outputs = MetricsOutputs(
            results=results,
            output_summary_csv=summary_csv,
            output_plot_png=plot_png,
            jump_count=len(jumps),
            first_jump_sec=first_jump_sec,
        )
        self.write_summary(
            outputs,
            eval_log_csv,
            ground_truth_csv,
            relative_delta,
            relative_delta_unit,
            relative_window_sec,
            alignment,
            min_relative_distance_m,
            segment_end_sec,
            jump_threshold_m,
        )

        if plot_png is not None:
            self.write_plot(
                plot_png,
                timestamps_ns,
                odom_points,
                aligned_odom_points,
                gt_points,
                ape_errors,
                outputs,
                segment_end_sec,
            )

        return outputs

    def compute_segment_result(
        self,
        label: str,
        timestamps_ns: np.ndarray,
        odom_points: np.ndarray,
        odom_quats: Optional[np.ndarray],
        gt_points: np.ndarray,
        gt_quats: Optional[np.ndarray],
        relative_delta: float,
        relative_delta_unit: str,
        relative_window_sec: float,
        min_relative_distance_m: float,
        alignment: str,
    ) -> tuple[EvaluationResult, np.ndarray, np.ndarray]:
        aligned_odom_points, rotation, _, scale = self.align_points(
            odom_points,
            gt_points,
            alignment,
        )
        aligned_odom_quats = self.rotate_quaternions(odom_quats, rotation)
        component_errors = aligned_odom_points - gt_points
        ape_errors = np.linalg.norm(component_errors, axis=1)
        squared_component_errors = component_errors * component_errors
        abs_component_errors = np.abs(component_errors)
        relative_errors = self.compute_relative_errors(
            timestamps_ns,
            aligned_odom_points,
            aligned_odom_quats,
            gt_points,
            gt_quats,
            relative_delta,
            relative_delta_unit,
            relative_window_sec,
            min_relative_distance_m,
        )
        uses_orientation = aligned_odom_quats is not None and gt_quats is not None

        result = EvaluationResult(
            label=label,
            sample_count=len(odom_points),
            relative_pair_count=len(relative_errors),
            avg_rte_percent=float(np.mean([err.rte_percent for err in relative_errors])),
            avg_re_deg=float(np.mean([err.re_deg for err in relative_errors])),
            rmse_ape_m=float(math.sqrt(np.mean(ape_errors * ape_errors))),
            mean_ape_m=float(np.mean(ape_errors)),
            median_ape_m=float(np.median(ape_errors)),
            max_ape_m=float(np.max(ape_errors)),
            rmse_x_m=float(math.sqrt(np.mean(squared_component_errors[:, 0]))),
            rmse_y_m=float(math.sqrt(np.mean(squared_component_errors[:, 1]))),
            rmse_z_m=float(math.sqrt(np.mean(squared_component_errors[:, 2]))),
            mean_abs_x_m=float(np.mean(abs_component_errors[:, 0])),
            mean_abs_y_m=float(np.mean(abs_component_errors[:, 1])),
            mean_abs_z_m=float(np.mean(abs_component_errors[:, 2])),
            max_abs_x_m=float(np.max(abs_component_errors[:, 0])),
            max_abs_y_m=float(np.max(abs_component_errors[:, 1])),
            max_abs_z_m=float(np.max(abs_component_errors[:, 2])),
            trajectory_length_m=self.path_length(gt_points),
            alignment_scale=scale,
            uses_orientation=uses_orientation,
        )
        return result, aligned_odom_points, ape_errors

    def load_trajectories(
        self,
        eval_log_csv: Path,
        ground_truth: GroundTruthTrack,
        max_match_dt_sec: float,
    ) -> tuple[
        np.ndarray,
        np.ndarray,
        Optional[np.ndarray],
        np.ndarray,
        Optional[np.ndarray],
    ]:
        timestamps_ns: list[int] = []
        odom_points: list[list[float]] = []
        odom_quats: list[list[float]] = []
        gt_points: list[list[float]] = []
        gt_quats: list[list[float]] = []
        skipped = 0

        with eval_log_csv.open(newline='', encoding='utf-8') as csv_file:
            reader = csv.DictReader(csv_file)
            required_columns = {
                'odom_timestamp_ns',
                'odom_x_m',
                'odom_y_m',
                'odom_z_m',
                'odom_qx',
                'odom_qy',
                'odom_qz',
                'odom_qw',
            }
            missing = required_columns - set(reader.fieldnames or [])
            if missing:
                missing_text = ', '.join(sorted(missing))
                raise ValueError(f'eval log CSV is missing columns: {missing_text}')

            for row in reader:
                timestamp_ns = int(row['odom_timestamp_ns'])
                gt = ground_truth.match(timestamp_ns)
                if abs(gt.dt_sec) > max_match_dt_sec:
                    skipped += 1
                    continue
                if not gt.has_orientation:
                    raise ValueError(
                        'ground truth match has no orientation; use EuRoC '
                        'state_groundtruth_estimate0/data.csv for avgRE'
                    )

                timestamps_ns.append(timestamp_ns)
                odom_points.append([
                    float(row['odom_x_m']),
                    float(row['odom_y_m']),
                    float(row['odom_z_m']),
                ])
                odom_quats.append([
                    float(row['odom_qx']),
                    float(row['odom_qy']),
                    float(row['odom_qz']),
                    float(row['odom_qw']),
                ])
                gt_points.append([gt.x, gt.y, gt.z])
                if gt.has_orientation:
                    gt_quats.append([gt.qx, gt.qy, gt.qz, gt.qw])

        if skipped:
            self.get_logger().warn(
                f'skipped {skipped} rows with |ground truth dt| > {max_match_dt_sec:.3f}s'
            )
        if len(odom_points) < 3:
            raise ValueError('need at least 3 matched trajectory samples')

        return (
            np.asarray(timestamps_ns, dtype=np.int64),
            np.asarray(odom_points, dtype=np.float64),
            np.asarray(
                [self.normalize_quat(quat) for quat in odom_quats],
                dtype=np.float64,
            ) if len(odom_quats) == len(odom_points) else None,
            np.asarray(gt_points, dtype=np.float64),
            np.asarray(
                [self.normalize_quat(quat) for quat in gt_quats],
                dtype=np.float64,
            ) if len(gt_quats) == len(gt_points) else None,
        )

    @staticmethod
    def align_points(
        source_points: np.ndarray,
        target_points: np.ndarray,
        alignment: str,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        if alignment not in ('se3', 'sim3'):
            raise ValueError("alignment must be 'se3' or 'sim3'")

        source_center = np.mean(source_points, axis=0)
        target_center = np.mean(target_points, axis=0)
        source_zero = source_points - source_center
        target_zero = target_points - target_center

        covariance = source_zero.T @ target_zero
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T

        scale = 1.0
        if alignment == 'sim3':
            denominator = float(np.sum(source_zero * source_zero))
            if denominator > 0.0:
                scale = float(np.sum((rotation @ source_zero.T).T * target_zero) / denominator)

        translation = target_center - scale * rotation @ source_center
        aligned = scale * (rotation @ source_points.T).T + translation
        return aligned, rotation, translation, scale

    @staticmethod
    def compute_relative_errors(
        timestamps_ns: np.ndarray,
        odom_points: np.ndarray,
        odom_quats: Optional[np.ndarray],
        gt_points: np.ndarray,
        gt_quats: Optional[np.ndarray],
        relative_delta: float,
        relative_delta_unit: str,
        relative_window_sec: float,
        min_relative_distance_m: float,
    ) -> list[RelativeError]:
        if odom_quats is None or gt_quats is None:
            raise ValueError(
                'avgRE requires odometry and ground truth quaternions; use EuRoC '
                'state_groundtruth_estimate0/data.csv and an eval log with odom_qx/qy/qz/qw'
            )
        pairs = Mh01MetricsNode.relative_pair_indices(
            timestamps_ns,
            gt_points,
            relative_delta,
            relative_delta_unit,
            relative_window_sec,
        )
        errors: list[RelativeError] = []

        for index, pair_index in pairs:
            odom_delta = odom_points[pair_index] - odom_points[index]
            gt_delta = gt_points[pair_index] - gt_points[index]
            gt_distance = float(np.linalg.norm(gt_delta))
            if gt_distance < min_relative_distance_m:
                continue

            translation_error = float(np.linalg.norm(odom_delta - gt_delta))
            rte_percent = translation_error / gt_distance * 100.0
            odom_delta_rot = Mh01MetricsNode.relative_rotation(
                odom_quats[index],
                odom_quats[pair_index],
            )
            gt_delta_rot = Mh01MetricsNode.relative_rotation(
                gt_quats[index],
                gt_quats[pair_index],
            )
            rot_error = Mh01MetricsNode.quat_multiply(
                Mh01MetricsNode.quat_inverse(gt_delta_rot),
                odom_delta_rot,
            )
            re_deg = Mh01MetricsNode.quaternion_angle_deg(rot_error)
            errors.append(RelativeError(rte_percent=rte_percent, re_deg=re_deg))

        if not errors:
            raise ValueError(
                'no valid relative error pairs; adjust relative_delta, '
                'relative_delta_unit, or min_relative_distance_m'
            )
        return errors

    @staticmethod
    def relative_pair_indices(
        timestamps_ns: np.ndarray,
        gt_points: np.ndarray,
        relative_delta: float,
        relative_delta_unit: str,
        relative_window_sec: float,
    ) -> list[tuple[int, int]]:
        unit = relative_delta_unit.strip().lower()
        pairs: list[tuple[int, int]] = []

        if unit in ('m', 'meter', 'meters', 'distance'):
            path = np.concatenate(
                ([0.0], np.cumsum(np.linalg.norm(np.diff(gt_points, axis=0), axis=1)))
            )
            for index, distance in enumerate(path):
                pair_index = int(np.searchsorted(path, distance + relative_delta))
                if pair_index >= len(path):
                    break
                pairs.append((index, pair_index))
            return pairs

        if unit in ('s', 'sec', 'second', 'seconds', 'time'):
            window_ns = int(relative_window_sec * 1_000_000_000)
            for index, timestamp_ns in enumerate(timestamps_ns):
                pair_index = int(np.searchsorted(timestamps_ns, timestamp_ns + window_ns))
                if pair_index >= len(timestamps_ns):
                    break
                pairs.append((index, pair_index))
            return pairs

        if unit in ('frame', 'frames'):
            frame_delta = int(round(relative_delta))
            if frame_delta <= 0:
                raise ValueError('relative_delta must be positive when relative_delta_unit=frames')
            for index in range(0, len(timestamps_ns) - frame_delta):
                pairs.append((index, index + frame_delta))
            return pairs

        raise ValueError("relative_delta_unit must be 'm', 's', or 'frames'")

    @staticmethod
    def segment_mask(timestamps_ns: np.ndarray, segment_end_sec: float) -> np.ndarray:
        elapsed_sec = (timestamps_ns - timestamps_ns[0]) * 1e-9
        return elapsed_sec <= segment_end_sec

    @staticmethod
    def detect_odometry_jumps(
        timestamps_ns: np.ndarray,
        odom_points: np.ndarray,
        gt_points: np.ndarray,
        threshold_m: float,
    ) -> list[tuple[float, float, float]]:
        jumps: list[tuple[float, float, float]] = []
        odom_steps = np.linalg.norm(np.diff(odom_points, axis=0), axis=1)
        gt_steps = np.linalg.norm(np.diff(gt_points, axis=0), axis=1)
        elapsed_sec = (timestamps_ns[1:] - timestamps_ns[0]) * 1e-9
        for time_sec, odom_step, gt_step in zip(elapsed_sec, odom_steps, gt_steps):
            if odom_step > threshold_m or abs(odom_step - gt_step) > threshold_m:
                jumps.append((float(time_sec), float(odom_step), float(gt_step)))
        return jumps

    @staticmethod
    def path_length(points: np.ndarray) -> float:
        deltas = np.diff(points, axis=0)
        return float(np.sum(np.linalg.norm(deltas, axis=1)))

    @staticmethod
    def rotate_quaternions(
        quats: Optional[np.ndarray],
        rotation: np.ndarray,
    ) -> Optional[np.ndarray]:
        if quats is None:
            return None
        rotation_quat = Mh01MetricsNode.rotation_matrix_to_quat(rotation)
        return np.asarray([
            Mh01MetricsNode.quat_multiply(rotation_quat, quat)
            for quat in quats
        ], dtype=np.float64)

    @staticmethod
    def normalize_quat(quat: np.ndarray | list[float] | tuple[float, float, float, float]) -> np.ndarray:
        q = np.asarray(quat, dtype=np.float64)
        norm = float(np.linalg.norm(q))
        if norm <= 0.0:
            return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return q / norm

    @staticmethod
    def quat_inverse(quat: np.ndarray) -> np.ndarray:
        q = Mh01MetricsNode.normalize_quat(quat)
        return np.asarray([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)

    @staticmethod
    def quat_multiply(q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
        x0, y0, z0, w0 = Mh01MetricsNode.normalize_quat(q0)
        x1, y1, z1, w1 = Mh01MetricsNode.normalize_quat(q1)
        return Mh01MetricsNode.normalize_quat(np.asarray([
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        ], dtype=np.float64))

    @staticmethod
    def relative_rotation(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        return Mh01MetricsNode.quat_multiply(
            Mh01MetricsNode.quat_inverse(q_from),
            q_to,
        )

    @staticmethod
    def quaternion_angle_deg(quat: np.ndarray) -> float:
        q = Mh01MetricsNode.normalize_quat(quat)
        w = max(-1.0, min(1.0, abs(float(q[3]))))
        return math.degrees(2.0 * math.acos(w))

    @staticmethod
    def rotation_matrix_to_quat(rotation: np.ndarray) -> np.ndarray:
        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (rotation[2, 1] - rotation[1, 2]) / s
            qy = (rotation[0, 2] - rotation[2, 0]) / s
            qz = (rotation[1, 0] - rotation[0, 1]) / s
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation[0, 1] + rotation[1, 0]) / s
            qz = (rotation[0, 2] + rotation[2, 0]) / s
        elif rotation[1, 1] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / s
            qx = (rotation[0, 1] + rotation[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / s
            qx = (rotation[0, 2] + rotation[2, 0]) / s
            qy = (rotation[1, 2] + rotation[2, 1]) / s
            qz = 0.25 * s
        return Mh01MetricsNode.normalize_quat(np.asarray([qx, qy, qz, qw], dtype=np.float64))

    @staticmethod
    def write_summary(
        outputs: MetricsOutputs,
        eval_log_csv: Path,
        ground_truth_csv: Path,
        relative_delta: float,
        relative_delta_unit: str,
        relative_window_sec: float,
        alignment: str,
        min_relative_distance_m: float,
        segment_end_sec: float,
        jump_threshold_m: float,
    ) -> None:
        with outputs.output_summary_csv.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['label', 'metric', 'value', 'unit'])
            writer.writerow(['input', 'ground_truth_csv', str(ground_truth_csv), 'path'])
            writer.writerow(['input', 'eval_log_csv', str(eval_log_csv), 'path'])
            writer.writerow(['config', 'pose_direction', 'T_odom_imu0 / T_world_body; no inverse', 'text'])
            writer.writerow(['config', 'alignment', alignment, 'se3_or_sim3'])
            writer.writerow(['config', 'relative_delta', relative_delta, relative_delta_unit])
            writer.writerow(['config', 'relative_delta_unit', relative_delta_unit, 'unit'])
            writer.writerow(['config', 'relative_window_sec', relative_window_sec, 's'])
            writer.writerow(['config', 'min_relative_distance_m', min_relative_distance_m, 'm'])
            writer.writerow(['config', 'segment_end_sec', segment_end_sec, 's'])
            writer.writerow(['config', 'jump_threshold_m', jump_threshold_m, 'm'])
            writer.writerow(['diagnostic', 'odometry_jump_count', outputs.jump_count, 'count'])
            if outputs.first_jump_sec is not None:
                writer.writerow(['diagnostic', 'first_odometry_jump_sec', outputs.first_jump_sec, 's'])

            for result in outputs.results:
                writer.writerow([result.label, 'sample_count', result.sample_count, 'count'])
                writer.writerow([
                    result.label,
                    'relative_pair_count',
                    result.relative_pair_count,
                    'count',
                ])
                writer.writerow([result.label, 'avgRTE', result.avg_rte_percent, '%'])
                writer.writerow([result.label, 'avgRE', result.avg_re_deg, 'deg'])
                writer.writerow([
                    result.label,
                    'avgRE_definition',
                    'relative rotation from odom and ground truth quaternions',
                    'text',
                ])
                writer.writerow([result.label, 'RMSE_APE', result.rmse_ape_m, 'm'])
                writer.writerow([result.label, 'alignment_scale', result.alignment_scale, 'ratio'])
                writer.writerow([result.label, 'mean_APE', result.mean_ape_m, 'm'])
                writer.writerow([result.label, 'median_APE', result.median_ape_m, 'm'])
                writer.writerow([result.label, 'max_APE', result.max_ape_m, 'm'])
                writer.writerow([result.label, 'RMSE_X', result.rmse_x_m, 'm'])
                writer.writerow([result.label, 'RMSE_Y', result.rmse_y_m, 'm'])
                writer.writerow([result.label, 'RMSE_Z', result.rmse_z_m, 'm'])
                writer.writerow([result.label, 'mean_abs_X', result.mean_abs_x_m, 'm'])
                writer.writerow([result.label, 'mean_abs_Y', result.mean_abs_y_m, 'm'])
                writer.writerow([result.label, 'mean_abs_Z', result.mean_abs_z_m, 'm'])
                writer.writerow([result.label, 'max_abs_X', result.max_abs_x_m, 'm'])
                writer.writerow([result.label, 'max_abs_Y', result.max_abs_y_m, 'm'])
                writer.writerow([result.label, 'max_abs_Z', result.max_abs_z_m, 'm'])
                writer.writerow([
                    result.label,
                    'gt_trajectory_length',
                    result.trajectory_length_m,
                    'm',
                ])

    @staticmethod
    def write_plot(
        output_png: Path,
        timestamps_ns: np.ndarray,
        raw_odom_points: np.ndarray,
        odom_points: np.ndarray,
        gt_points: np.ndarray,
        ape_errors: np.ndarray,
        outputs: MetricsOutputs,
        segment_end_sec: float,
    ) -> None:
        import matplotlib

        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        time_sec = (timestamps_ns - timestamps_ns[0]) * 1e-9
        fig = plt.figure(figsize=(13, 11), constrained_layout=True)
        grid = fig.add_gridspec(2, 2)
        ax_xy = fig.add_subplot(grid[0, 0])
        ax_xz = fig.add_subplot(grid[0, 1])
        ax_yz = fig.add_subplot(grid[1, 0])
        ax_3d = fig.add_subplot(grid[1, 1], projection='3d')
        full_result = outputs.results[0]

        ax_xy.plot(gt_points[:, 0], gt_points[:, 1], label='Ground truth', linewidth=2.0)
        ax_xy.plot(
            odom_points[:, 0],
            odom_points[:, 1],
            label='Odometry aligned',
            linewidth=1.5,
        )
        ax_xy.set_title('XY trajectory')
        ax_xy.set_xlabel('x [m]')
        ax_xy.set_ylabel('y [m]')
        ax_xy.axis('equal')
        ax_xy.grid(True, alpha=0.3)
        ax_xy.legend()
        ax_xy.text(
            0.02,
            0.98,
            (
                f'RMSE APE: {full_result.rmse_ape_m:.3f} m\n'
                f'RMSE XYZ: {full_result.rmse_x_m:.3f}, '
                f'{full_result.rmse_y_m:.3f}, {full_result.rmse_z_m:.3f} m'
            ),
            transform=ax_xy.transAxes,
            va='top',
            bbox={'facecolor': 'white', 'alpha': 0.8, 'edgecolor': 'none'},
        )

        ax_xz.plot(gt_points[:, 0], gt_points[:, 2], label='Ground truth', linewidth=2.0)
        ax_xz.plot(
            odom_points[:, 0],
            odom_points[:, 2],
            label='Odometry aligned',
            linewidth=1.5,
        )
        ax_xz.set_title('XZ trajectory')
        ax_xz.set_xlabel('x [m]')
        ax_xz.set_ylabel('z [m]')
        ax_xz.axis('equal')
        ax_xz.grid(True, alpha=0.3)
        ax_xz.legend()

        ax_yz.plot(gt_points[:, 1], gt_points[:, 2], label='Ground truth', linewidth=2.0)
        ax_yz.plot(
            odom_points[:, 1],
            odom_points[:, 2],
            label='Odometry aligned',
            linewidth=1.5,
        )
        ax_yz.set_title('YZ trajectory')
        ax_yz.set_xlabel('y [m]')
        ax_yz.set_ylabel('z [m]')
        ax_yz.axis('equal')
        ax_yz.grid(True, alpha=0.3)
        ax_yz.legend()

        ax_3d.plot(
            gt_points[:, 0],
            gt_points[:, 1],
            gt_points[:, 2],
            label='Ground truth',
            linewidth=2.0,
        )
        ax_3d.plot(
            odom_points[:, 0],
            odom_points[:, 1],
            odom_points[:, 2],
            label='Odometry aligned',
            linewidth=1.5,
        )
        ax_3d.set_title('3D trajectory')
        ax_3d.set_xlabel('x [m]')
        ax_3d.set_ylabel('y [m]')
        ax_3d.set_zlabel('z [m]')
        ax_3d.legend()
        Mh01MetricsNode.set_axes_equal_3d(ax_3d, gt_points, odom_points)

        fig.savefig(output_png, dpi=160)
        plt.close(fig)

    @staticmethod
    def set_axes_equal_3d(ax, *point_sets: np.ndarray) -> None:
        points = np.vstack(point_sets)
        mins = np.min(points, axis=0)
        maxs = np.max(points, axis=0)
        centers = (mins + maxs) * 0.5
        radius = float(np.max(maxs - mins) * 0.5)
        if radius <= 0.0:
            radius = 1.0
        ax.set_xlim(centers[0] - radius, centers[0] + radius)
        ax.set_ylim(centers[1] - radius, centers[1] + radius)
        ax.set_zlim(centers[2] - radius, centers[2] + radius)

    def log_result(self, outputs: MetricsOutputs) -> None:
        for result in outputs.results:
            self.get_logger().info(f'[{result.label}] samples: {result.sample_count}')
            self.get_logger().info(f'[{result.label}] relative pairs: {result.relative_pair_count}')
            self.get_logger().info(f'[{result.label}] avgRTE: {result.avg_rte_percent:.6f} %')
            self.get_logger().info(f'[{result.label}] avgRE: {result.avg_re_deg:.6f} deg')
            self.get_logger().info(f'[{result.label}] RMSE APE: {result.rmse_ape_m:.6f} m')
            self.get_logger().info(f'[{result.label}] alignment scale: {result.alignment_scale:.9f}')
            self.get_logger().info(
                f'[{result.label}] avgRE source: '
                f'{"quaternion RPE" if result.uses_orientation else "translation-vector fallback"}'
            )
            self.get_logger().info(
                f'[{result.label}] RMSE XYZ: '
                f'{result.rmse_x_m:.6f}, {result.rmse_y_m:.6f}, {result.rmse_z_m:.6f} m'
            )
        self.get_logger().info(f'odometry jump count: {outputs.jump_count}')
        if outputs.first_jump_sec is not None:
            self.get_logger().info(f'first odometry jump: {outputs.first_jump_sec:.3f} s')
        self.get_logger().info(f'summary CSV: {outputs.output_summary_csv}')
        if outputs.output_plot_png is not None:
            self.get_logger().info(f'plot PNG: {outputs.output_plot_png}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mh01MetricsNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
