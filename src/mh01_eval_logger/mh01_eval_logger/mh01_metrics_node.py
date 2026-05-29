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
    sample_count: int
    relative_pair_count: int
    avg_rte_percent: float
    avg_re_deg: float
    rmse_ape_m: float
    mean_ape_m: float
    median_ape_m: float
    max_ape_m: float
    trajectory_length_m: float
    output_summary_csv: Path
    output_plot_png: Optional[Path]


class Mh01MetricsNode(Node):
    def __init__(self):
        super().__init__('mh01_metrics_node')

        self.declare_parameter(
            'ground_truth_csv',
            '/home/hashizume/experiment/MH_01_easy_leica_position.csv',
        )
        self.declare_parameter(
            'eval_log_csv',
            '/home/hashizume/experiment/results/mh01_eval_log_20260528_150755.csv',
        )
        self.declare_parameter('output_dir', '/home/hashizume/experiment/results')
        self.declare_parameter('relative_window_sec', 1.0)
        self.declare_parameter('max_match_dt_sec', 0.5)
        self.declare_parameter('plot', True)

        ground_truth_csv = Path(
            str(self.get_parameter('ground_truth_csv').value)
        ).expanduser()
        eval_log_csv = Path(str(self.get_parameter('eval_log_csv').value)).expanduser()
        output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        relative_window_sec = float(self.get_parameter('relative_window_sec').value)
        max_match_dt_sec = float(self.get_parameter('max_match_dt_sec').value)
        plot = bool(self.get_parameter('plot').value)

        result = self.evaluate(
            ground_truth_csv=ground_truth_csv,
            eval_log_csv=eval_log_csv,
            output_dir=output_dir,
            relative_window_sec=relative_window_sec,
            max_match_dt_sec=max_match_dt_sec,
            plot=plot,
        )
        self.log_result(result)

    def evaluate(
        self,
        ground_truth_csv: Path,
        eval_log_csv: Path,
        output_dir: Path,
        relative_window_sec: float,
        max_match_dt_sec: float,
        plot: bool,
    ) -> EvaluationResult:
        ground_truth = GroundTruthTrack.from_csv(ground_truth_csv)
        timestamps_ns, odom_points, gt_points = self.load_trajectories(
            eval_log_csv,
            ground_truth,
            max_match_dt_sec,
        )

        aligned_odom_points, _, _ = self.align_rigid(odom_points, gt_points)
        ape_errors = np.linalg.norm(aligned_odom_points - gt_points, axis=1)
        rmse_ape = float(math.sqrt(np.mean(ape_errors * ape_errors)))
        relative_errors = self.compute_relative_errors(
            timestamps_ns,
            aligned_odom_points,
            gt_points,
            relative_window_sec,
        )

        avg_rte = float(np.mean([err.rte_percent for err in relative_errors]))
        avg_re = float(np.mean([err.re_deg for err in relative_errors]))
        trajectory_length = self.path_length(gt_points)

        output_dir.mkdir(parents=True, exist_ok=True)
        summary_csv = output_dir / f'{eval_log_csv.stem}_metrics_summary.csv'
        plot_png = output_dir / f'{eval_log_csv.stem}_metrics_plot.png' if plot else None

        result = EvaluationResult(
            sample_count=len(odom_points),
            relative_pair_count=len(relative_errors),
            avg_rte_percent=avg_rte,
            avg_re_deg=avg_re,
            rmse_ape_m=rmse_ape,
            mean_ape_m=float(np.mean(ape_errors)),
            median_ape_m=float(np.median(ape_errors)),
            max_ape_m=float(np.max(ape_errors)),
            trajectory_length_m=trajectory_length,
            output_summary_csv=summary_csv,
            output_plot_png=plot_png,
        )
        self.write_summary(result, eval_log_csv, ground_truth_csv, relative_window_sec)

        if plot_png is not None:
            self.write_plot(
                plot_png,
                timestamps_ns,
                aligned_odom_points,
                gt_points,
                ape_errors,
                result,
            )

        return result

    def load_trajectories(
        self,
        eval_log_csv: Path,
        ground_truth: GroundTruthTrack,
        max_match_dt_sec: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        timestamps_ns: list[int] = []
        odom_points: list[list[float]] = []
        gt_points: list[list[float]] = []
        skipped = 0

        with eval_log_csv.open(newline='', encoding='utf-8') as csv_file:
            reader = csv.DictReader(csv_file)
            required_columns = {
                'odom_timestamp_ns',
                'odom_x_m',
                'odom_y_m',
                'odom_z_m',
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

                timestamps_ns.append(timestamp_ns)
                odom_points.append([
                    float(row['odom_x_m']),
                    float(row['odom_y_m']),
                    float(row['odom_z_m']),
                ])
                gt_points.append([gt.x, gt.y, gt.z])

        if skipped:
            self.get_logger().warn(
                f'skipped {skipped} rows with |ground truth dt| > {max_match_dt_sec:.3f}s'
            )
        if len(odom_points) < 3:
            raise ValueError('need at least 3 matched trajectory samples')

        return (
            np.asarray(timestamps_ns, dtype=np.int64),
            np.asarray(odom_points, dtype=np.float64),
            np.asarray(gt_points, dtype=np.float64),
        )

    @staticmethod
    def align_rigid(
        source_points: np.ndarray,
        target_points: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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

        translation = target_center - rotation @ source_center
        aligned = (rotation @ source_points.T).T + translation
        return aligned, rotation, translation

    @staticmethod
    def compute_relative_errors(
        timestamps_ns: np.ndarray,
        odom_points: np.ndarray,
        gt_points: np.ndarray,
        relative_window_sec: float,
    ) -> list[RelativeError]:
        window_ns = int(relative_window_sec * 1_000_000_000)
        errors: list[RelativeError] = []

        for index, timestamp_ns in enumerate(timestamps_ns):
            target_timestamp_ns = timestamp_ns + window_ns
            pair_index = int(np.searchsorted(timestamps_ns, target_timestamp_ns))
            if pair_index >= len(timestamps_ns):
                break

            odom_delta = odom_points[pair_index] - odom_points[index]
            gt_delta = gt_points[pair_index] - gt_points[index]
            gt_distance = float(np.linalg.norm(gt_delta))
            odom_distance = float(np.linalg.norm(odom_delta))
            if gt_distance <= 1e-9 or odom_distance <= 1e-9:
                continue

            translation_error = float(np.linalg.norm(odom_delta - gt_delta))
            rte_percent = translation_error / gt_distance * 100.0
            cosine = float(np.dot(odom_delta, gt_delta) / (odom_distance * gt_distance))
            cosine = max(-1.0, min(1.0, cosine))
            re_deg = math.degrees(math.acos(cosine))
            errors.append(RelativeError(rte_percent=rte_percent, re_deg=re_deg))

        if not errors:
            raise ValueError('no valid relative error pairs; lower relative_window_sec')
        return errors

    @staticmethod
    def path_length(points: np.ndarray) -> float:
        deltas = np.diff(points, axis=0)
        return float(np.sum(np.linalg.norm(deltas, axis=1)))

    @staticmethod
    def write_summary(
        result: EvaluationResult,
        eval_log_csv: Path,
        ground_truth_csv: Path,
        relative_window_sec: float,
    ) -> None:
        with result.output_summary_csv.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['metric', 'value', 'unit'])
            writer.writerow(['ground_truth_csv', str(ground_truth_csv), 'path'])
            writer.writerow(['eval_log_csv', str(eval_log_csv), 'path'])
            writer.writerow(['sample_count', result.sample_count, 'count'])
            writer.writerow(['relative_window_sec', relative_window_sec, 's'])
            writer.writerow(['relative_pair_count', result.relative_pair_count, 'count'])
            writer.writerow(['avgRTE', result.avg_rte_percent, '%'])
            writer.writerow(['avgRE', result.avg_re_deg, 'deg'])
            writer.writerow(['RMSE_APE', result.rmse_ape_m, 'm'])
            writer.writerow(['mean_APE', result.mean_ape_m, 'm'])
            writer.writerow(['median_APE', result.median_ape_m, 'm'])
            writer.writerow(['max_APE', result.max_ape_m, 'm'])
            writer.writerow(['gt_trajectory_length', result.trajectory_length_m, 'm'])

    @staticmethod
    def write_plot(
        output_png: Path,
        timestamps_ns: np.ndarray,
        odom_points: np.ndarray,
        gt_points: np.ndarray,
        ape_errors: np.ndarray,
        result: EvaluationResult,
    ) -> None:
        import matplotlib

        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        time_sec = (timestamps_ns - timestamps_ns[0]) * 1e-9
        fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)

        axes[0].plot(gt_points[:, 0], gt_points[:, 1], label='Ground truth', linewidth=2.0)
        axes[0].plot(
            odom_points[:, 0],
            odom_points[:, 1],
            label='Odometry aligned',
            linewidth=1.5,
        )
        axes[0].set_title('XY trajectory')
        axes[0].set_xlabel('x [m]')
        axes[0].set_ylabel('y [m]')
        axes[0].axis('equal')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(time_sec, ape_errors, color='tab:red', linewidth=1.4)
        axes[1].axhline(result.rmse_ape_m, color='black', linestyle='--', linewidth=1.0)
        axes[1].set_title('APE over time')
        axes[1].set_xlabel('time [s]')
        axes[1].set_ylabel('APE [m]')
        axes[1].grid(True, alpha=0.3)
        axes[1].text(
            0.02,
            0.98,
            (
                f'avgRTE: {result.avg_rte_percent:.3f}%\n'
                f'avgRE: {result.avg_re_deg:.3f} deg\n'
                f'RMSE APE: {result.rmse_ape_m:.3f} m'
            ),
            transform=axes[1].transAxes,
            va='top',
            bbox={'facecolor': 'white', 'alpha': 0.8, 'edgecolor': 'none'},
        )

        fig.savefig(output_png, dpi=160)
        plt.close(fig)

    def log_result(self, result: EvaluationResult) -> None:
        self.get_logger().info(f'samples: {result.sample_count}')
        self.get_logger().info(f'relative pairs: {result.relative_pair_count}')
        self.get_logger().info(f'avgRTE: {result.avg_rte_percent:.6f} %')
        self.get_logger().info(f'avgRE: {result.avg_re_deg:.6f} deg')
        self.get_logger().info(f'RMSE APE: {result.rmse_ape_m:.6f} m')
        self.get_logger().info(f'summary CSV: {result.output_summary_csv}')
        if result.output_plot_png is not None:
            self.get_logger().info(f'plot PNG: {result.output_plot_png}')


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
