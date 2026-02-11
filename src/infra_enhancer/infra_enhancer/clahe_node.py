"""ClaheNode — CLAHE コントラスト強調ノード

image_in を購読し、CLAHE 適用後の画像を image_out に配信する。

ROS 2 (Humble/Iron/Jazzy) 対応
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .image_utils import QOS_IMAGE, to_mono8, mono8_to_msg


class ClaheNode(Node):
    def __init__(self):
        super().__init__('clahe_node')

        # パラメータ
        self.declare_parameter('clip_limit', 2.0)
        self.declare_parameter('tile_size', 8)

        clip = float(self.get_parameter('clip_limit').value)
        tile = int(self.get_parameter('tile_size').value)

        self.clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
        self.bridge = CvBridge()

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            f'ClaheNode started  (clip={clip}, tile={tile})')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        img = self.clahe.apply(img)

        self.pub.publish(mono8_to_msg(img, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = ClaheNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
