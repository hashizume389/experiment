"""SelectiveEnhancerNode - lightweight task-aware preprocessing for VSLAM.

This node is intended for EuRoC/Isaac ROS Visual SLAM experiments where
CLAHE-based preprocessing reduced trajectory accuracy, while custom
feature-guided preprocessing became expensive due to edge scoring and
ORB/FAST-style detector stages.  It uses only image statistics and small OpenCV
filters, then applies enhancement only when the frame looks likely to benefit.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from .image_utils import QOS_IMAGE, mono8_to_msg, to_mono8


class SelectiveEnhancerNode(Node):
    """Adaptive low-cost image enhancer for visual SLAM front-ends."""

    def __init__(self):
        super().__init__('selective_enhancer_node')

        self.declare_parameter('metric_scale', 0.5)
        self.declare_parameter('dark_mean', 70.0)
        self.declare_parameter('bright_mean', 185.0)
        self.declare_parameter('low_contrast', 42.0)
        self.declare_parameter('target_contrast', 70.0)
        self.declare_parameter('saturation_percent', 2.0)
        self.declare_parameter('gradient_low', 18.0)
        self.declare_parameter('gamma_dark', 0.72)
        self.declare_parameter('gamma_bright', 1.15)
        self.declare_parameter('max_blend', 0.85)
        self.declare_parameter('stretch_percentile_low', 1.0)
        self.declare_parameter('stretch_percentile_high', 99.0)
        self.declare_parameter('unsharp_amount', 0.25)
        self.declare_parameter('unsharp_sigma', 1.0)
        self.declare_parameter('enable_unsharp', True)
        self.declare_parameter('debug_log_every', 0)

        self.metric_scale = self._clamp_float('metric_scale', 0.05, 1.0)
        self.dark_mean = float(self.get_parameter('dark_mean').value)
        self.bright_mean = float(self.get_parameter('bright_mean').value)
        self.low_contrast = float(self.get_parameter('low_contrast').value)
        self.target_contrast = float(self.get_parameter('target_contrast').value)
        self.saturation_percent = float(self.get_parameter('saturation_percent').value)
        self.gradient_low = float(self.get_parameter('gradient_low').value)
        self.gamma_dark = self._clamp_float('gamma_dark', 0.2, 3.0)
        self.gamma_bright = self._clamp_float('gamma_bright', 0.2, 3.0)
        self.max_blend = self._clamp_float('max_blend', 0.0, 1.0)
        self.p_low = self._clamp_float('stretch_percentile_low', 0.0, 20.0)
        self.p_high = self._clamp_float('stretch_percentile_high', 80.0, 100.0)
        self.unsharp_amount = max(0.0, float(self.get_parameter('unsharp_amount').value))
        self.unsharp_sigma = max(0.1, float(self.get_parameter('unsharp_sigma').value))
        self.enable_unsharp = bool(self.get_parameter('enable_unsharp').value)
        self.debug_log_every = int(self.get_parameter('debug_log_every').value)

        self._frame_count = 0
        self._gamma_tables: dict[float, np.ndarray] = {}
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            'SelectiveEnhancerNode started '
            f'(metric_scale={self.metric_scale}, max_blend={self.max_blend})'
        )

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as exc:
            self.get_logger().error(f'Conversion error: {exc}')
            return

        metrics_img = self._metrics_image(img)
        mean = float(np.mean(metrics_img))
        p_low, p_high = np.percentile(metrics_img, [self.p_low, self.p_high])
        contrast = float(p_high - p_low)
        saturated = self._saturation_percent(metrics_img)
        gradient = self._mean_gradient(metrics_img)

        need_lift = mean < self.dark_mean
        need_tame = mean > self.bright_mean or saturated > self.saturation_percent
        need_contrast = contrast < self.low_contrast
        low_texture = gradient < self.gradient_low

        out = img
        if need_lift or need_tame:
            gamma = self._adaptive_gamma(mean, saturated)
            out = cv2.LUT(out, self._gamma_table(gamma))

        if need_contrast:
            out = self._percentile_stretch(out, contrast)

        if self.enable_unsharp and (need_contrast or low_texture) and self.unsharp_amount > 0.0:
            out = self._unsharp(out, low_texture)

        if out is not img:
            strength = self._blend_strength(mean, contrast, saturated)
            if strength < 1.0:
                out = cv2.addWeighted(img, 1.0 - strength, out, strength, 0.0)

        self._maybe_log(mean, contrast, saturated, gradient, out is not img)
        self.pub.publish(mono8_to_msg(out, msg.header, self.bridge))

    def _metrics_image(self, img: np.ndarray) -> np.ndarray:
        if self.metric_scale >= 0.99:
            return img
        return cv2.resize(
            img,
            None,
            fx=self.metric_scale,
            fy=self.metric_scale,
            interpolation=cv2.INTER_AREA,
        )

    def _adaptive_gamma(self, mean: float, saturated: float) -> float:
        if mean < self.dark_mean:
            alpha = np.clip((self.dark_mean - mean) / max(self.dark_mean, 1.0), 0.0, 1.0)
            return float(1.0 + alpha * (self.gamma_dark - 1.0))
        bright_alpha = np.clip((mean - self.bright_mean) / max(255.0 - self.bright_mean, 1.0), 0.0, 1.0)
        sat_alpha = np.clip(saturated / max(self.saturation_percent * 4.0, 1.0), 0.0, 1.0)
        alpha = max(float(bright_alpha), float(sat_alpha))
        return float(1.0 + alpha * (self.gamma_bright - 1.0))

    def _gamma_table(self, gamma: float) -> np.ndarray:
        key = round(gamma, 3)
        table = self._gamma_tables.get(key)
        if table is None:
            values = ((np.arange(256, dtype=np.float32) / 255.0) ** key) * 255.0
            table = np.clip(values, 0, 255).astype(np.uint8)
            self._gamma_tables[key] = table
        return table

    def _percentile_stretch(self, img: np.ndarray, contrast: float) -> np.ndarray:
        lo, hi = np.percentile(img, [self.p_low, self.p_high])
        if hi <= lo + 1.0:
            return img
        gain = min(self.target_contrast / max(contrast, 1.0), 2.2)
        mid = 0.5 * (lo + hi)
        stretched = (img.astype(np.float32) - mid) * gain + mid
        return np.clip(stretched, 0, 255).astype(np.uint8)

    def _unsharp(self, img: np.ndarray, low_texture: bool) -> np.ndarray:
        amount = self.unsharp_amount * (1.4 if low_texture else 1.0)
        blurred = cv2.GaussianBlur(img, (0, 0), self.unsharp_sigma)
        sharpened = img.astype(np.float32) + amount * (img.astype(np.float32) - blurred.astype(np.float32))
        return np.clip(sharpened, 0, 255).astype(np.uint8)

    def _blend_strength(self, mean: float, contrast: float, saturated: float) -> float:
        dark_score = np.clip((self.dark_mean - mean) / max(self.dark_mean, 1.0), 0.0, 1.0)
        bright_score = np.clip((mean - self.bright_mean) / max(255.0 - self.bright_mean, 1.0), 0.0, 1.0)
        contrast_score = np.clip((self.low_contrast - contrast) / max(self.low_contrast, 1.0), 0.0, 1.0)
        sat_score = np.clip(saturated / max(self.saturation_percent * 4.0, 1.0), 0.0, 1.0)
        score = max(float(dark_score), float(bright_score), float(contrast_score), float(sat_score))
        return max(0.15, min(self.max_blend, score * self.max_blend))

    @staticmethod
    def _saturation_percent(img: np.ndarray) -> float:
        saturated = np.count_nonzero((img <= 3) | (img >= 252))
        return 100.0 * float(saturated) / float(img.size)

    @staticmethod
    def _mean_gradient(img: np.ndarray) -> float:
        gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)
        return float(np.mean(cv2.magnitude(gx, gy)))

    def _maybe_log(self, mean: float, contrast: float, saturated: float, gradient: float, enhanced: bool) -> None:
        if self.debug_log_every <= 0:
            return
        self._frame_count += 1
        if self._frame_count % self.debug_log_every == 0:
            self.get_logger().info(
                f'mean={mean:.1f} contrast={contrast:.1f} sat={saturated:.2f}% '
                f'grad={gradient:.1f} enhanced={enhanced}'
            )

    def _clamp_float(self, name: str, lower: float, upper: float) -> float:
        value = float(self.get_parameter(name).value)
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = SelectiveEnhancerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
