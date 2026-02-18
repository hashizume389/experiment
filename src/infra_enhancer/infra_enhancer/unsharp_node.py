"""UnsharpNode — アンシャープマスク（鮮鋭化）ノード

image_in を購読し、アンシャープマスク適用後の画像を image_out に配信する。

アンシャープマスクの原理:
  sharpened = original + amount * (original - blurred)
  ぼかした画像との差分（高周波成分）を原画像に加算することで、
  エッジやテクスチャを強調する。

パラメータ:
  ksize  : ガウシアンぼかしのカーネルサイズ（奇数, default=5）
  sigma  : ガウシアンぼかしの標準偏差（default=1.0）
  amount : 鮮鋭化の強度（default=0.5, 0.3〜1.0 推奨）

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


class UnsharpNode(Node):
    def __init__(self):
        super().__init__('unsharp_node')

        # ─── パラメータ ────────────────────────────────────────
        self.declare_parameter('ksize', 5)       # ぼかしカーネルサイズ (奇数)
        self.declare_parameter('sigma', 1.0)     # ガウシアンぼかしのσ
        self.declare_parameter('amount', 0.5)    # 鮮鋭化の強度

        self.ksize = int(self.get_parameter('ksize').value)
        self.sigma = float(self.get_parameter('sigma').value)
        self.amount = float(self.get_parameter('amount').value)

        # カーネルサイズは奇数である必要がある
        if self.ksize % 2 == 0:
            self.ksize += 1

        self.bridge = CvBridge()

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            f'UnsharpNode started  (ksize={self.ksize}, sigma={self.sigma}, amount={self.amount})')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        # アンシャープマスク処理
        # 1. ガウシアンぼかしで低周波成分を取得
        blurred = cv2.GaussianBlur(img, (self.ksize, self.ksize), self.sigma)

        # 2. sharpened = original + amount * (original - blurred)
        #    float32 で計算してクリッピング
        sharpened = img.astype(np.float32) + self.amount * (img.astype(np.float32) - blurred.astype(np.float32))
        sharpened = np.clip(sharpened, 0, 255).astype(np.uint8)

        self.pub.publish(mono8_to_msg(sharpened, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = UnsharpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
