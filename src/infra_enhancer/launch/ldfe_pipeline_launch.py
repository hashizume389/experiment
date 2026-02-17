"""InfraEnhancer LDFE パイプライン Launch ファイル

LDFE-SLAM の原則に基づいた照明適応型前処理ノード (ldfe_node) を起動する。
左右画像 (infra1, infra2) それぞれに対してノードを立ち上げる。

使用例:
  ros2 launch infra_enhancer ldfe_pipeline_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Launch Arguments ───────────────────────────────────────
    # 必要に応じてパラメータを外部から変更できるようにする
    # 例: ros2 launch ... clahe_clip_max:=6.0
    
    return LaunchDescription([
        # ─── 左画像系統 (infra1) ────────────────────────────────────
        Node(
            package='infra_enhancer',
            executable='ldfe_node',  # setup.py で設定するエントリポイント名
            name='ldfe_infra1',
            remappings=[
                ('image_in',  '/camera/infra1/image_rect_raw'),
                ('image_out', '/proc/infra1/image_enhanced'),
            ],
            parameters=[{
                'brightness_dark': 50.0,
                'brightness_bright': 180.0,
                'clahe_clip_min': 1.0,
                'clahe_clip_max': 5.0,
                'gamma_min': 0.5,
                'gamma_max': 1.0,
            }],
        ),

        # ─── 右画像系統 (infra2) ────────────────────────────────────
        Node(
            package='infra_enhancer',
            executable='ldfe_node',
            name='ldfe_infra2',
            remappings=[
                ('image_in',  '/camera/infra2/image_rect_raw'),
                ('image_out', '/proc/infra2/image_enhanced'),
            ],
            parameters=[{
                'brightness_dark': 50.0,
                'brightness_bright': 180.0,
                'clahe_clip_min': 1.0,
                'clahe_clip_max': 5.0,
                'gamma_min': 0.5,
                'gamma_max': 1.0,
            }],
        ),
    ])
