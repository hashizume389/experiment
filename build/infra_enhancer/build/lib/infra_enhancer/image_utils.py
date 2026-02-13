"""共通ユーティリティ

各画像処理ノードで共有するヘルパー関数と QoS プロファイル定数。
"""

from __future__ import annotations

import numpy as np
import cv2

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# ─── 共通 QoS プロファイル ──────────────────────────────────────
QOS_IMAGE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


# ─── 画像変換ヘルパー ─────────────────────────────────────────
def to_mono8(img_msg: Image, bridge: CvBridge) -> np.ndarray:
    """ROS Image メッセージを uint8 グレースケール numpy 配列に変換する。"""
    try:
        return bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
    except CvBridgeError:
        raw = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if raw.ndim == 3:
            raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        return _to_uint8(raw)


def mono8_to_msg(cv_img: np.ndarray, header, bridge: CvBridge) -> Image:
    """uint8 グレースケール numpy 配列を ROS Image メッセージに変換する。"""
    if cv_img.dtype != np.uint8:
        cv_img = cv_img.astype(np.uint8)
    out = bridge.cv2_to_imgmsg(cv_img, encoding='mono8')
    out.header = header
    return out


def _to_uint8(arr: np.ndarray) -> np.ndarray:
    """様々な画像型（float, uint16 等）を uint8 に変換する。"""
    if arr.dtype == np.uint8:
        return arr
    arr = arr.astype(np.float32)
    mn, mx = float(np.nanmin(arr)), float(np.nanmax(arr))
    if mx <= mn:
        return np.zeros(arr.shape, dtype=np.uint8)
    scaled = (arr - mn) * (255.0 / (mx - mn))
    return np.clip(scaled, 0, 255).astype(np.uint8)
