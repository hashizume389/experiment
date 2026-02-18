"""LDFENode — LDFE-SLAM 原則に基づく照明適応型前処理ノード

Illumination-Adaptive Enhancement（照明適応型強調）を実装しています。
画像の平均輝度に応じて、CLAHE（Contrast Limited Adaptive Histogram Equalization）の
クリップ制限とガンマ補正値を動的に調整します。

背景と目的 (LDFE-SLAM Principles):
Visual SLAM（特に特徴点ベースの手法）は、照明条件が悪い（暗すぎる、明暗差が激しい）環境では
特徴点の抽出や追跡が困難になります。
LDFE-SLAM (Light-Aware Deep Front-End SLAM) の研究では、以下の原則を提案しています：

1. Illumination-Adaptive (照明適応性):
   一律のパラメータで強調するのではなく、暗い場所ではコントラスト強調を強め、
   十分明るい場所ではノイズ増幅を避けるために強調を弱めるべきです。

2. Structure Preservation (構造の維持):
   単に明るくするだけでなく、幾何学的な構造（エッジやコーナー）を復元・維持することが重要です。
   これにより、SuperPointなどのDeep特徴点抽出器の性能が向上します。

実装の詳細:
- 入力画像: 赤外線カメラ画像 (mono8)
- 処理フロー:
  1. 輝度計測: 画像全体の平均輝度を計算し、現在の「暗さ」を評価します。
  2. パラメータ適応:
     - CLAHE Clip Limit: 暗いほど値を大きくし、局所的なコントラストを強調します。
     - Gamma Correction: 暗いほど値を小さく (gamma < 1.0) し、暗部の階調を持ち上げます。
  3. 処理適用: 計算されたパラメータを用いて画像変換を行います。

ROS 2 (Humble/Iron/Jazzy) 対応
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from .image_utils import QOS_IMAGE, to_mono8, mono8_to_msg


class LDFENode(Node):
    def __init__(self):
        super().__init__('ldfe_node')

        # ─── パラメータ設定 ───────────────────────────────────────────
        
        # CLAHE Clip Limit の範囲設定
        # Clip Limit はコントラスト制限の閾値です。
        # 値が大きいほどコントラストが強調されますが、同時にノイズも増幅されます。
        # 暗い環境 (Low Light): 構造が見えにくいため、高い値 (例: 5.0) にして強調します。
        # 明るい環境 (High Light): 既に十分見えるため、低い値 (例: 1.0) にしてノイズを抑えます。
        self.declare_parameter('clahe_clip_min', 1.0)
        self.declare_parameter('clahe_clip_max', 5.0)
        
        # Gamma 補正値の範囲設定
        # 出力 = 入力 ^ (1 / gamma) の関係になります (OpenCVの実装や一般的定義による)。
        # gamma < 1.0: 暗部が持ち上がり、全体が明るくなります (暗所向け)。
        # gamma > 1.0: 全体が暗くなり、コントラストが強まります (明所向け)。
        # ここでは暗所対策のため、暗いときに 0.5 程度まで下げる設定にします。
        self.declare_parameter('gamma_min', 0.5)
        self.declare_parameter('gamma_max', 1.2)
        
        # 輝度基準値 (0-255)
        # 画像の平均輝度がこの範囲のどこにあるかで、パラメータを線形補間します。
        self.declare_parameter('brightness_dark', 50.0)   # この輝度以下なら「完全に暗い」と判断し、最大補正を適用
        self.declare_parameter('brightness_bright', 180.0) # この輝度以上なら「十分に明るい」と判断し、最小補正を適用

        # CLAHE タイルサイズ
        # 画像を分割するグリッドのサイズ。8x8 が一般的です。
        self.declare_parameter('clahe_tile_grid_size', 8)

        # パラメータ読み込み
        self.clip_min = self.get_parameter('clahe_clip_min').value
        self.clip_max = self.get_parameter('clahe_clip_max').value
        self.gamma_min = self.get_parameter('gamma_min').value
        self.gamma_max = self.get_parameter('gamma_max').value
        self.br_dark = self.get_parameter('brightness_dark').value
        self.br_bright = self.get_parameter('brightness_bright').value
        self.tile_size = self.get_parameter('clahe_tile_grid_size').value

        self.bridge = CvBridge()

        # Pub / Sub
        self.pub = self.create_publisher(Image, 'image_out', QOS_IMAGE)
        self.sub = self.create_subscription(Image, 'image_in', self.callback, QOS_IMAGE)

        self.get_logger().info('LDFENode started (Adaptive Enhancement)')
        self.get_logger().info(f'Config: Dark<{self.br_dark} | Bright>{self.br_bright}')

    def callback(self, msg: Image) -> None:
        try:
            img = to_mono8(msg, self.bridge)
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')
            return

        # 1. 輝度計測 (Illumination Scoring)
        # 画像全体の平均輝度を計算します。
        # NOTE: 処理速度を優先するため単純平均を用いていますが、
        # 中央重点平均など、ロボットの注目領域に重みを置くことも有効です。
        mean_brightness = np.mean(img)

        # 2. パラメータ適応 (Adaptive Parameter Calculation)
        # 現在の平均輝度 (mean_brightness) に基づいてパラメータを動的に決定します。
        # np.interp は線形補間を行います:
        #   x <= br_dark   -> param_max (最大補正)
        #   x >= br_bright -> param_min (最小補正)
        #   その中間       -> 線形に変化
        
        # CLAHE Clip Limit: 暗いほど値を大きく (Maxへ)、明るいほど小さく (Minへ)
        current_clip = np.interp(
            mean_brightness,
            [self.br_dark, self.br_bright],
            [self.clip_max, self.clip_min]
        )
        
        # Gamma: 暗いほど値を小さく (Minへ)、明るいほど大きく (Maxへ)
        # gamma が小さいほど (e.g. 0.5)、暗いピクセルが明るく変換されます。
        current_gamma = np.interp(
            mean_brightness,
            [self.br_dark, self.br_bright],
            [self.gamma_min, self.gamma_max]
        )

        # 3. CLAHE (局所コントラスト強調) の適用
        # adaptive clip limit を使用してオブジェクトを生成・適用します。
        clahe = cv2.createCLAHE(clipLimit=current_clip, tileGridSize=(self.tile_size, self.tile_size))
        img_clahe = clahe.apply(img)

        # 4. Gamma 補正 (全体輝度調整) の適用
        # ガンマ補正は計算コスト削減のため、Look Up Table (LUT) を使用します。
        # 式: output = ((input / 255) ^ (1/gamma)) * 255
        inv_gamma = 1.0 / current_gamma
        table = np.array([((i / 255.0) ** current_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        img_enhanced = cv2.LUT(img_clahe, table)
        
        # デバッグ用: 補正状況を確認したい場合は以下のコメントを解除してください
        # self.get_logger().info(f'Br: {mean_brightness:.1f}, Clip: {current_clip:.2f}, Gamma: {current_gamma:.2f}')

        # 5. 配信
        self.pub.publish(mono8_to_msg(img_enhanced, msg.header, self.bridge))


def main(args=None):
    rclpy.init(args=args)
    node = LDFENode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
