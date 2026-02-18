"""EdgeNode — エッジ検出フィルタノード

image_in を購読し、エッジ検出後の画像を image_out に配信する。
方式は ROS パラメータ `method` で選択: 'laplacian' | 'sobel' | 'canny'

各方式の概要:
- laplacian: ラプラシアンフィルタ（2次微分）。全方向のエッジを検出。
- sobel: Sobelフィルタ（1次微分）。X/Y方向の勾配の大きさを計算。
- canny: Cannyエッジ検出。エッジの細線化・ヒステリシス閾値処理を含む。

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


class EdgeNode(Node):
    def __init__(self):
        super().__init__('edge_node')

        # ─── パラメータ ────────────────────────────────────────
        self.declare_parameter('method', 'laplacian')       # 'laplacian' | 'sobel' | 'canny'
        self.declare_parameter('ksize', 3)                   # カーネルサイズ (laplacian, sobel)
        self.declare_parameter('canny_low', 50)              # Canny 低閾値
        self.declare_parameter('canny_high', 150)            # Canny 高閾値

        self.method = self.get_parameter('method').value
        self.ksize = int(self.get_parameter('ksize').value)
        self.canny_low = int(self.get_parameter('canny_low').value)
        self.canny_high = int(self.get_parameter('canny_high').value)

        self.bridge = CvBridge()

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            f'EdgeNode started  (method={self.method}, ksize={self.ksize})')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        if self.method == 'laplacian':
            # ラプラシアンフィルタ (2次微分)
            # 全方向のエッジを一度に検出する
            edge = cv2.Laplacian(img, cv2.CV_16S, ksize=self.ksize)
            edge = np.abs(edge)
            edge = np.clip(edge, 0, 255).astype(np.uint8)

        elif self.method == 'sobel':
            # Sobelフィルタ (1次微分)
            # X方向・Y方向の勾配を個別に計算し、大きさを合成
            grad_x = cv2.Sobel(img, cv2.CV_16S, 1, 0, ksize=self.ksize)
            grad_y = cv2.Sobel(img, cv2.CV_16S, 0, 1, ksize=self.ksize)
            edge = np.sqrt(grad_x.astype(np.float32) ** 2 + grad_y.astype(np.float32) ** 2)
            edge = np.clip(edge, 0, 255).astype(np.uint8)

        elif self.method == 'canny':
            # Cannyエッジ検出
            # 非極大値抑制 + ヒステリシス閾値により、細くて途切れにくいエッジを出力
            edge = cv2.Canny(img, self.canny_low, self.canny_high)

        else:
            self.get_logger().warn(f'Unknown method: {self.method}, passing through')
            edge = img

        self.pub.publish(mono8_to_msg(edge, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = EdgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
