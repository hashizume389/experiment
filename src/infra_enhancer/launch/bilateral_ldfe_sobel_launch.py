"""パイプライン3: バイラテラルフィルタ → LDFE → Sobel

柿農園でのVisual SLAM向け前処理パイプライン。
エッジ保存型ノイズ除去 + 照明適応型強調 + エッジ検出の3段構成。

注意: Sobelフィルタの出力はエッジ画像（勾配の大きさ）であり、
      元の輝度情報が失われます。SLAMの特徴点マッチングには
      不利になる可能性があるため、実験的な構成です。

処理フロー:
  /camera/infra{1,2}/image_rect_raw
    → denoise_node (bilateral)
    → ldfe_node (CLAHE + Gamma)
    → edge_node (sobel)
    → /proc/infra{1,2}/image_enhanced

使用例:
  ros2 launch infra_enhancer bilateral_ldfe_sobel_launch.py
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

    sobel_params = {
        'method': 'sobel',
        'ksize': 3,
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
                ('image_out', '/proc/infra1/image_ldfe'),
            ],
            parameters=[ldfe_params],
        ),

        # ③ Sobel エッジ検出
        Node(
            package='infra_enhancer',
            executable='edge_node',
            name='sobel_infra1',
            remappings=[
                ('image_in',  '/proc/infra1/image_ldfe'),
                ('image_out', '/proc/infra1/image_enhanced'),
            ],
            parameters=[sobel_params],
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
                ('image_out', '/proc/infra2/image_ldfe'),
            ],
            parameters=[ldfe_params],
        ),

        # ③ Sobel エッジ検出
        Node(
            package='infra_enhancer',
            executable='edge_node',
            name='sobel_infra2',
            remappings=[
                ('image_in',  '/proc/infra2/image_ldfe'),
                ('image_out', '/proc/infra2/image_enhanced'),
            ],
            parameters=[sobel_params],
        ),
    ])
