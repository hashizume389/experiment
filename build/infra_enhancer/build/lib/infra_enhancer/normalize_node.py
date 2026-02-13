"""NormalizeNode — 画像正規化ノード

image_in を購読し、正規化後の画像を image_out に配信する。

ROS 2 (Humble/Iron/Jazzy) 対応
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .image_utils import QOS_IMAGE, to_mono8, mono8_to_msg


class NormalizeNode(Node):
    def __init__(self):
        super().__init__('normalize_node')

        # パラメータ
        self.declare_parameter('min_val', 0)
        self.declare_parameter('max_val', 255)

        self.min_val = int(self.get_parameter('min_val').value)
        self.max_val = int(self.get_parameter('max_val').value)
        self.bridge = CvBridge()

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            f'NormalizeNode started  (range=[{self.min_val}, {self.max_val}])')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        img = cv2.normalize(img, None, self.min_val, self.max_val, cv2.NORM_MINMAX)

        if img.dtype != np.uint8:
            img = img.astype(np.uint8)

        self.pub.publish(mono8_to_msg(img, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = NormalizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
