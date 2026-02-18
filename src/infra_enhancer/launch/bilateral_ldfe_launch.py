"""パイプライン1: バイラテラルフィルタ → LDFE

柿農園でのVisual SLAM向け前処理パイプライン。
エッジ保存型ノイズ除去 + 照明適応型強調の2段構成。

処理フロー:
  /camera/infra{1,2}/image_rect_raw
    → denoise_node (bilateral)
    → ldfe_node (CLAHE + Gamma)
    → /proc/infra{1,2}/image_enhanced

使用例:
  ros2 launch infra_enhancer bilateral_ldfe_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ─── 共通パラメータ ─────────────────────────────────────────
    bilateral_params = {
        'method': 'bilateral',
        'bilateral_d': 5,
        'bilateral_sigma_color': 50.0,
        'bilateral_sigma_space': 50.0,
    }

    ldfe_params = {
        'brightness_dark': 50.0,
        'brightness_bright': 180.0,
        'clahe_clip_min': 1.0,
        'clahe_clip_max': 5.0,
        'gamma_min': 0.5,
        'gamma_max': 1.2,
    }

    return LaunchDescription([
        # ════════════════════════════════════════════════════════════
        #  左画像系統 (infra1)
        # ════════════════════════════════════════════════════════════

        # ① バイラテラルフィルタ
        Node(
            package='infra_enhancer',
            executable='denoise_node',
            name='denoise_infra1',
            remappings=[
                ('image_in',  '/camera/infra1/image_rect_raw'),
                ('image_out', '/proc/infra1/image_denoised'),
            ],
            parameters=[bilateral_params],
        ),

        # ② LDFE (CLAHE + Gamma)
        Node(
            package='infra_enhancer',
            executable='ldfe_node',
            name='ldfe_infra1',
            remappings=[
                ('image_in',  '/proc/infra1/image_denoised'),
                ('image_out', '/proc/infra1/image_enhanced'),
            ],
            parameters=[ldfe_params],
        ),

        # ════════════════════════════════════════════════════════════
        #  右画像系統 (infra2)
        # ════════════════════════════════════════════════════════════

        # ① バイラテラルフィルタ
        Node(
            package='infra_enhancer',
            executable='denoise_node',
            name='denoise_infra2',
            remappings=[
                ('image_in',  '/camera/infra2/image_rect_raw'),
                ('image_out', '/proc/infra2/image_denoised'),
            ],
            parameters=[bilateral_params],
        ),

        # ② LDFE (CLAHE + Gamma)
        Node(
            package='infra_enhancer',
            executable='ldfe_node',
            name='ldfe_infra2',
            remappings=[
                ('image_in',  '/proc/infra2/image_denoised'),
                ('image_out', '/proc/infra2/image_enhanced'),
            ],
            parameters=[ldfe_params],
        ),
    ])
