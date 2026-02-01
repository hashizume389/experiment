"""InfraEnhancer node

- Subscribes to:
  /camera/infra1/image_rect_raw  (sensor_msgs/msg/Image)
  /camera/infra2/image_raw       (sensor_msgs/msg/Image)
- Uses message_filters.ApproximateTimeSynchronizer to sync images
- Applies: denoising (Gaussian or bilateral), CLAHE, normalization
- Publishes:
  /proc/infra1/image_enhanced
  /proc/infra2/image_enhanced

Efficient, parameterized, written for ROS 2 (Humble/Iron/Jazzy)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import message_filters
import numpy as np
import cv2


class InfraEnhancer(Node):
    def __init__(self):
        super().__init__('infra_enhancer')

        # Parameters (can be overridden by ros2 param set)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.02)  # seconds for approx sync
        self.declare_parameter('clahe_clip', 2.0)
        self.declare_parameter('clahe_tile', 8)
        self.declare_parameter('denoise', 'gaussian')  # 'gaussian' | 'bilateral' | 'none'
        self.declare_parameter('gaussian_ksize', 3)
        self.declare_parameter('bilateral_d', 5)
        self.declare_parameter('bilateral_sigmaColor', 75)
        self.declare_parameter('bilateral_sigmaSpace', 75)

        self.queue_size = self.get_parameter('queue_size').value
        self.slop = float(self.get_parameter('slop').value)
        self.bridge = CvBridge()

        # Create CLAHE instance
        clahe_clip = float(self.get_parameter('clahe_clip').value)
        clahe_tile = int(self.get_parameter('clahe_tile').value)
        self.clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))

        # Denoise params
        self.denoise = self.get_parameter('denoise').value
        self.gaussian_ksize = int(self.get_parameter('gaussian_ksize').value)
        self.bilateral_d = int(self.get_parameter('bilateral_d').value)
        self.bilateral_sigmaColor = float(self.get_parameter('bilateral_sigmaColor').value)
        self.bilateral_sigmaSpace = float(self.get_parameter('bilateral_sigmaSpace').value)

        # Publishers
        self.pub1 = self.create_publisher(Image, '/proc/infra1/image_enhanced', qos_profile_sensor_data)
        self.pub2 = self.create_publisher(Image, '/proc/infra2/image_enhanced', qos_profile_sensor_data)

        # Subscribers via message_filters
        self.sub1 = message_filters.Subscriber(self, Image, '/camera/infra1/image_rect_raw')
        self.sub2 = message_filters.Subscriber(self, Image, '/camera/infra2/image_raw')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2], self.queue_size, self.slop)
        self.sync.registerCallback(self.callback)

        self.get_logger().info('InfraEnhancer node started')

    def callback(self, img1_msg: Image, img2_msg: Image) -> None:
        # Process both images and publish
        try:
            proc1 = self.process_image_msg(img1_msg)
            proc2 = self.process_image_msg(img2_msg)
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            return

        # Convert to Image messages preserving original headers
        try:
            out1 = self.bridge.cv2_to_imgmsg(proc1, encoding='mono8')
            out1.header = img1_msg.header
            out2 = self.bridge.cv2_to_imgmsg(proc2, encoding='mono8')
            out2.header = img2_msg.header
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')
            return

        self.pub1.publish(out1)
        self.pub2.publish(out2)

    def process_image_msg(self, img_msg: Image) -> np.ndarray:
        # Convert to OpenCV image (grayscale 8-bit)
        try:
            # Try to get a mono8 image directly
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        except CvBridgeError:
            # Fallback: convert passthrough and then to 8-bit
            raw = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            if raw.ndim == 3:
                # If somehow RGB, convert to gray
                raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
            # Normalize/scale to 0..255 and convert to uint8
            cv_img = self._to_uint8(raw)

        # Small denoise (light to keep features)
        if self.denoise == 'gaussian':
            k = self.gaussian_ksize if self.gaussian_ksize % 2 == 1 else self.gaussian_ksize + 1
            if k > 1:
                cv_img = cv2.GaussianBlur(cv_img, (k, k), 0)
        elif self.denoise == 'bilateral':
            cv_img = cv2.bilateralFilter(cv_img, self.bilateral_d,
                                         self.bilateral_sigmaColor, self.bilateral_sigmaSpace)

        # CLAHE
        cv_img = self.clahe.apply(cv_img)

        # Normalize to reduce brightness variation - preserve contrast
        cv_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)

        # Ensure dtype
        if cv_img.dtype != np.uint8:
            cv_img = cv_img.astype(np.uint8)

        return cv_img

    def _to_uint8(self, arr: np.ndarray) -> np.ndarray:
        # Map arbitrary image types (float, uint16, etc.) to uint8 efficiently
        if arr.dtype == np.uint8:
            return arr
        arr = arr.astype(np.float32)
        mn = float(np.nanmin(arr))
        mx = float(np.nanmax(arr))
        if mx <= mn:
            return np.zeros(arr.shape, dtype=np.uint8)
        # scale
        scaled = (arr - mn) * (255.0 / (mx - mn))
        return np.clip(scaled, 0, 255).astype(np.uint8)


def main(args=None):
    rclpy.init(args=args)
    node = InfraEnhancer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
