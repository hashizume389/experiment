"""InfraEnhancer ノード（オールインワン版 — 後方互換）

- 購読トピック:
  /camera/infra1/image_rect_raw  (sensor_msgs/msg/Image)
  /camera/infra2/image_raw       (sensor_msgs/msg/Image)
- message_filters.ApproximateTimeSynchronizerで画像を時刻同期
- 処理内容: ノイズ除去（Gaussian or Bilateral）、CLAHE、正規化
- 配信トピック:
  /proc/infra1/image_enhanced
  /proc/infra2/image_enhanced

分割ノード版（denoise_node / clahe_node / normalize_node）を推奨。
ROS 2 (Humble/Iron/Jazzy) 対応
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import message_filters
import numpy as np
import cv2

from .image_utils import QOS_IMAGE, to_mono8, mono8_to_msg


class InfraEnhancer(Node):
    def __init__(self):
        super().__init__('infra_enhancer')

        # パラメータ（ros2 param setで変更可能）
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

        # CLAHEインスタンスを作成
        clahe_clip = float(self.get_parameter('clahe_clip').value)
        clahe_tile = int(self.get_parameter('clahe_tile').value)
        self.clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))

        # ノイズ除去パラメータ
        self.denoise = self.get_parameter('denoise').value
        self.gaussian_ksize = int(self.get_parameter('gaussian_ksize').value)
        self.bilateral_d = int(self.get_parameter('bilateral_d').value)
        self.bilateral_sigmaColor = float(self.get_parameter('bilateral_sigmaColor').value)
        self.bilateral_sigmaSpace = float(self.get_parameter('bilateral_sigmaSpace').value)

        # パブリッシャー
        self.pub1 = self.create_publisher(Image, '/proc/infra1/image_enhanced', QOS_IMAGE)
        self.pub2 = self.create_publisher(Image, '/proc/infra2/image_enhanced', QOS_IMAGE)

        # message_filtersによるサブスクライバー
        self.sub1 = message_filters.Subscriber(self, Image, '/camera/infra1/image_rect_raw', qos_profile=QOS_IMAGE)
        self.sub2 = message_filters.Subscriber(self, Image, '/camera/infra2/image_rect_raw', qos_profile=QOS_IMAGE)

        #左右画像のマッチング，タイムスタンプの比較でペアを形成
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2], self.queue_size, self.slop)
        self.sync.registerCallback(self.callback)

        self.get_logger().info('InfraEnhancer node started')

    def callback(self, img1_msg: Image, img2_msg: Image) -> None:
        # 両方の画像を処理して配信
        try:
            proc1 = self.process_image_msg(img1_msg)
            proc2 = self.process_image_msg(img2_msg)
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            return

        try:
            out1 = mono8_to_msg(proc1, img1_msg.header, self.bridge)
            out2 = mono8_to_msg(proc2, img2_msg.header, self.bridge)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')
            return

        self.pub1.publish(out1)
        self.pub2.publish(out2)

    #ROS固有の画像フォーマットをOpenCV形式に変換し，画像処理を行う
    def process_image_msg(self, img_msg: Image) -> np.ndarray:
        # OpenCV画像に変換（8bitグレースケール）
        cv_img = to_mono8(img_msg, self.bridge)

        # ノイズ除去（Gaussian：全体的にぼかして砂嵐ノイズを除去 or Bilateral：エッジを維持したまま平滑化する）
        if self.denoise == 'gaussian':
            k = self.gaussian_ksize if self.gaussian_ksize % 2 == 1 else self.gaussian_ksize + 1
            if k > 1:
                cv_img = cv2.GaussianBlur(cv_img, (k, k), 0)
        elif self.denoise == 'bilateral':
            cv_img = cv2.bilateralFilter(cv_img, self.bilateral_d,
                                         self.bilateral_sigmaColor, self.bilateral_sigmaSpace)

        # CLAHE（コントラスト強調）
        cv_img = self.clahe.apply(cv_img)

        # 正規化（画像全体の明るさのバラつきを抑える）
        cv_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX)

        # データ型を確認
        if cv_img.dtype != np.uint8:
            cv_img = cv_img.astype(np.uint8)

        return cv_img



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


"""
処理後の画像をIsaac ROS Visual SLAMに入力するには

remappings=[
    ('visual_slam/image_0', '/proc/infra1/image_enhanced'),  # 変更
    ('visual_slam/camera_info_0', 'camera/infra1/camera_info'),
    ('visual_slam/image_1', '/proc/infra2/image_enhanced'),  # 変更
    ('visual_slam/camera_info_1', 'camera/infra2/camera_info'),
    ('visual_slam/imu', 'camera/imu'),
],
"""