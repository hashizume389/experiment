"""DenoiseNode — ノイズ除去ノード

image_in を購読し、ノイズ除去後の画像を image_out に配信する。
方式は ROS パラメータ `method` で選択: 'gaussian' | 'bilateral' | 'none'

ROS 2 (Humble/Iron/Jazzy) 対応
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .image_utils import QOS_IMAGE, to_mono8, mono8_to_msg


class DenoiseNode(Node):
    def __init__(self):
        super().__init__('denoise_node')

        # パラメータ
        self.declare_parameter('method', 'gaussian')       # 'gaussian' | 'bilateral' | 'none'
        self.declare_parameter('gaussian_ksize', 3)
        self.declare_parameter('bilateral_d', 5)
        self.declare_parameter('bilateral_sigma_color', 75.0)
        self.declare_parameter('bilateral_sigma_space', 75.0)

        self.method = self.get_parameter('method').value
        self.gaussian_ksize = int(self.get_parameter('gaussian_ksize').value)
        self.bilateral_d = int(self.get_parameter('bilateral_d').value)
        self.bilateral_sigma_color = float(self.get_parameter('bilateral_sigma_color').value)
        self.bilateral_sigma_space = float(self.get_parameter('bilateral_sigma_space').value)

        self.bridge = CvBridge()

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            f'DenoiseNode started  (method={self.method})')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        # ノイズ除去
        if self.method == 'gaussian':
            k = self.gaussian_ksize if self.gaussian_ksize % 2 == 1 else self.gaussian_ksize + 1
            if k > 1:
                img = cv2.GaussianBlur(img, (k, k), 0)
        elif self.method == 'bilateral':
            img = cv2.bilateralFilter(
                img, self.bilateral_d,
                self.bilateral_sigma_color, self.bilateral_sigma_space)

        self.pub.publish(mono8_to_msg(img, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = DenoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
