"""SpikeNode — イベントベース スパイクフィルタノード

フレーム間の輝度変化をスパイク列（3値画像）に変換する。
イベントカメラ（DVS）の動作をソフトウェアでエミュレートし、
動きのあるエッジを強調する前処理を提供する。

出力値:
  255 (白) : 明るくなった (ON event)  — diff >  threshold
  128 (灰) : 変化なし                — |diff| <= threshold
    0 (黒) : 暗くなった (OFF event)  — diff < -threshold

ROS 2 (Humble/Iron/Jazzy) 対応
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .image_utils import QOS_IMAGE, to_mono8, mono8_to_msg


class SpikeNode(Node):
    def __init__(self):
        super().__init__('spike_node')

        # ─── パラメータ ────────────────────────────────────────
        # threshold: スパイク検出の輝度差閾値 (0-255)
        # 値が小さいほど微小な変化もスパイクとして検出する（ノイズも拾いやすい）
        # 値が大きいほど大きな変化のみ検出する（ロバストだが感度が低い）
        self.declare_parameter('threshold', 10)

        self.threshold = int(self.get_parameter('threshold').value)
        self.bridge = CvBridge()
        self.prev_frame: np.ndarray | None = None

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info(
            f'SpikeNode started  (threshold={self.threshold})')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        if self.prev_frame is None:
            # 初回フレーム: 前フレームがないため、全ピクセル 128（変化なし）を出力
            self.prev_frame = img.copy()
            spike_img = np.full_like(img, 128, dtype=np.uint8)
            self.pub.publish(mono8_to_msg(spike_img, msg.header, self.bridge))
            return

        # フレーム間差分を int16 で計算（符号付きにするため）
        diff = img.astype(np.int16) - self.prev_frame.astype(np.int16)

        # 3値スパイク画像の生成
        spike_img = np.full_like(img, 128, dtype=np.uint8)  # デフォルト: 変化なし
        spike_img[diff > self.threshold] = 255               # ON event (明るくなった)
        spike_img[diff < -self.threshold] = 0                # OFF event (暗くなった)

        # 前フレームを更新
        self.prev_frame = img.copy()

        self.pub.publish(mono8_to_msg(spike_img, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = SpikeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
