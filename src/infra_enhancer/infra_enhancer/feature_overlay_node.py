"""FeatureOverlayNode — Visual SLAM 特徴点を画像上に投影・可視化するノード

Isaac ROS Visual SLAM が配信する特徴点 (PointCloud2) を、
2系統のカメラ画像 (raw / enhanced) 上に投影して描画した画像を配信します。

入力:
  - image_in      (sensor_msgs/Image)       : raw カメラ画像 (mono8)
  - image_in_2    (sensor_msgs/Image)       : 処理後カメラ画像 (mono8)
  - cloud_in      (sensor_msgs/PointCloud2)  : Visual SLAM の特徴点群
  - camera_info_in (sensor_msgs/CameraInfo)  : カメラ内部パラメータ

出力:
  - image_out     (sensor_msgs/Image)        : raw に特徴点を描画した画像 (bgr8)
  - image_out_2   (sensor_msgs/Image)        : 処理後に特徴点を描画した画像 (bgr8)
  - feature_count (std_msgs/Int32)           : 投影された特徴点数

処理フロー:
  1. PointCloud2 の 3D 座標を TF でカメラ光学フレームに変換
  2. カメラの内部パラメータ (K 行列) で 2D 画像平面に投影
  3. 画像範囲内の点をマーカーとして両方の画像に描画
  4. 結果を配信
"""

from __future__ import annotations

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

import tf2_ros
import message_filters

from .image_utils import QOS_IMAGE


class FeatureOverlayNode(Node):
    def __init__(self):
        super().__init__('feature_overlay_node')

        # ─── パラメータ ──────────────────────────────────────────
        self.declare_parameter('marker_color_r', 0)
        self.declare_parameter('marker_color_g', 255)
        self.declare_parameter('marker_color_b', 0)
        self.declare_parameter('marker_radius', 3)
        self.declare_parameter('sync_slop', 0.1)  # 時刻同期の許容誤差 (秒)

        self.marker_color = (
            self.get_parameter('marker_color_b').value,
            self.get_parameter('marker_color_g').value,
            self.get_parameter('marker_color_r').value,
        )
        self.marker_radius = self.get_parameter('marker_radius').value
        sync_slop = self.get_parameter('sync_slop').value

        self.bridge = CvBridge()

        # ─── TF2 Buffer ─────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ─── カメラ内部パラメータ (最新値をキャッシュ) ─────────────
        self.camera_matrix = None  # 3x3 K行列
        self.dist_coeffs = None
        self.img_width = 0
        self.img_height = 0

        # CameraInfo は単独で購読 (パラメータは滅多に変化しないため)
        self.sub_info = self.create_subscription(
            CameraInfo, 'camera_info_in', self._cb_camera_info, 10
        )

        # ─── 画像 (2系統) + PointCloud2 の同期購読 ──────────────────
        self.sub_image_raw = message_filters.Subscriber(self, Image, 'image_in')
        self.sub_image_enh = message_filters.Subscriber(self, Image, 'image_in_2')
        self.sub_cloud = message_filters.Subscriber(self, PointCloud2, 'cloud_in')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_image_raw, self.sub_image_enh, self.sub_cloud],
            queue_size=10,
            slop=sync_slop,
        )
        self.sync.registerCallback(self._cb_sync)

        # ─── Publisher ───────────────────────────────────────────
        self.pub_image_raw = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.pub_image_enh = self.create_publisher(Image, 'image_out_2', QOS_IMAGE)
        self.pub_count = self.create_publisher(Int32, 'feature_count', 10)

        self.get_logger().info(
            f'FeatureOverlayNode started (dual image) — color=BGR{self.marker_color}, '
            f'radius={self.marker_radius}, sync_slop={sync_slop}s'
        )

    # ─── CameraInfo コールバック ──────────────────────────────────
    def _cb_camera_info(self, msg: CameraInfo):
        """カメラ内部パラメータをキャッシュする。"""
        # K行列 (3x3) — [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else None
        self.img_width = msg.width
        self.img_height = msg.height

    # ─── 同期コールバック ─────────────────────────────────────────
    def _cb_sync(self, img_raw_msg: Image, img_enh_msg: Image, cloud_msg: PointCloud2):
        if self.camera_matrix is None:
            self.get_logger().warn('CameraInfo not yet received, skipping', throttle_duration_sec=2.0)
            return

        # 1. 両方の画像を取得
        try:
            gray_raw = self.bridge.imgmsg_to_cv2(img_raw_msg, desired_encoding='mono8')
            gray_enh = self.bridge.imgmsg_to_cv2(img_enh_msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        # 2. PointCloud2 から 3D 座標を抽出
        points_3d = self._read_pointcloud2(cloud_msg)
        if points_3d is None or len(points_3d) == 0:
            # 特徴点なしでもそのまま画像を配信
            canvas_raw = cv2.cvtColor(gray_raw, cv2.COLOR_GRAY2BGR)
            canvas_enh = cv2.cvtColor(gray_enh, cv2.COLOR_GRAY2BGR)
            self._put_text(canvas_raw, 0)
            self._put_text(canvas_enh, 0)
            self._publish(canvas_raw, canvas_enh, img_raw_msg.header, img_enh_msg.header, 0)
            return

        # 3. TF でカメラ光学フレームに変換
        camera_frame = img_raw_msg.header.frame_id
        cloud_frame = cloud_msg.header.frame_id

        if camera_frame and cloud_frame and camera_frame != cloud_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    camera_frame, cloud_frame,
                    rclpy.time.Time(),  # 最新の TF を使用
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                points_3d = self._transform_points(points_3d, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF lookup failed ({cloud_frame} -> {camera_frame}): {e}',
                    throttle_duration_sec=2.0,
                )
                # TF が取れない場合はそのまま投影を試みる
                pass

        # 4. 3D → 2D 投影
        points_2d = self._project_to_image(points_3d)

        # 5. 画像範囲内の点だけをフィルタ
        h, w = gray_raw.shape[:2]
        valid_mask = (
            (points_2d[:, 0] >= 0) & (points_2d[:, 0] < w) &
            (points_2d[:, 1] >= 0) & (points_2d[:, 1] < h)
        )
        visible_points = points_2d[valid_mask]
        count = len(visible_points)

        # 6. 両方の画像に同じ特徴点を描画
        canvas_raw = cv2.cvtColor(gray_raw, cv2.COLOR_GRAY2BGR)
        canvas_enh = cv2.cvtColor(gray_enh, cv2.COLOR_GRAY2BGR)
        for pt in visible_points:
            x, y = int(pt[0]), int(pt[1])
            cv2.circle(canvas_raw, (x, y), self.marker_radius, self.marker_color, -1)
            cv2.circle(canvas_enh, (x, y), self.marker_radius, self.marker_color, -1)

        self._put_text(canvas_raw, count)
        self._put_text(canvas_enh, count)
        self._publish(canvas_raw, canvas_enh, img_raw_msg.header, img_enh_msg.header, count)

    # ─── PointCloud2 パーサー ─────────────────────────────────────
    def _read_pointcloud2(self, msg: PointCloud2) -> np.ndarray | None:
        """PointCloud2 メッセージから (N, 3) の float32 配列を取得する。"""
        # フィールドから x, y, z のオフセットを取得
        field_map = {f.name: f for f in msg.fields}
        if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
            self.get_logger().warn('PointCloud2 missing x/y/z fields', throttle_duration_sec=5.0)
            return None

        x_off = field_map['x'].offset
        y_off = field_map['y'].offset
        z_off = field_map['z'].offset

        n_points = msg.width * msg.height
        if n_points == 0:
            return np.empty((0, 3), dtype=np.float32)

        points = np.empty((n_points, 3), dtype=np.float32)
        data = msg.data
        step = msg.point_step

        for i in range(n_points):
            base = i * step
            points[i, 0] = struct.unpack_from('f', data, base + x_off)[0]
            points[i, 1] = struct.unpack_from('f', data, base + y_off)[0]
            points[i, 2] = struct.unpack_from('f', data, base + z_off)[0]

        # NaN / Inf を除去
        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    # ─── TF 変換 ──────────────────────────────────────────────────
    def _transform_points(self, points: np.ndarray,
                          transform) -> np.ndarray:
        """TransformStamped を使って点群を変換する。"""
        t = transform.transform.translation
        q = transform.transform.rotation

        # クォータニオン → 回転行列
        R = self._quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        tvec = np.array([t.x, t.y, t.z], dtype=np.float64)

        # 変換適用: p' = R * p + t
        return (R @ points.T).T + tvec

    @staticmethod
    def _quat_to_rotation_matrix(x, y, z, w) -> np.ndarray:
        """クォータニオン (x, y, z, w) → 3x3 回転行列。"""
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ], dtype=np.float64)
        return R

    # ─── 3D → 2D 投影 ────────────────────────────────────────────
    def _project_to_image(self, points_3d: np.ndarray) -> np.ndarray:
        """カメラ内部パラメータで 3D 点を 2D 画像座標に投影する。

        pinhole モデル:
            u = fx * X/Z + cx
            v = fy * Y/Z + cy
        """
        if len(points_3d) == 0:
            return np.empty((0, 2), dtype=np.float64)

        # Z > 0 (カメラ前方) の点のみを対象
        z = points_3d[:, 2]
        valid = z > 0.01  # 1cm 以上前方
        pts = points_3d[valid]

        if len(pts) == 0:
            return np.empty((0, 2), dtype=np.float64)

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        u = fx * pts[:, 0] / pts[:, 2] + cx
        v = fy * pts[:, 1] / pts[:, 2] + cy

        return np.column_stack([u, v])

    # ─── ヘルパー ─────────────────────────────────────────────────
    def _put_text(self, canvas: np.ndarray, count: int):
        """特徴点数テキストを画像左上に描画する。"""
        text = f'SLAM Features: {count}'
        cv2.putText(canvas, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)

    def _publish(self, canvas_raw: np.ndarray, canvas_enh: np.ndarray,
                header_raw, header_enh, count: int):
        """描画済み画像 (raw / enhanced) と特徴点数を配信する。"""
        out_raw = self.bridge.cv2_to_imgmsg(canvas_raw, encoding='bgr8')
        out_raw.header = header_raw
        self.pub_image_raw.publish(out_raw)

        out_enh = self.bridge.cv2_to_imgmsg(canvas_enh, encoding='bgr8')
        out_enh.header = header_enh
        self.pub_image_enh.publish(out_enh)

        count_msg = Int32()
        count_msg.data = count
        self.pub_count.publish(count_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FeatureOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
