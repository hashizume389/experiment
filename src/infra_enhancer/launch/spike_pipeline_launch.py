"""InfraEnhancer スパイクフィルタ パイプライン Launch ファイル

イベントベースのスパイクフィルタノード (spike_node) を起動する。
左右画像 (infra1, infra2) それぞれに対してノードを立ち上げる。

使用例:
  ros2 launch infra_enhancer spike_pipeline_launch.py
  ros2 launch infra_enhancer spike_pipeline_launch.py threshold:=15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Launch Arguments ───────────────────────────────────────
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='10',
        description='Spike detection threshold (0-255)')

    return LaunchDescription([
        threshold_arg,

        # ─── 左画像系統 (infra1) ────────────────────────────────────
        Node(
            package='infra_enhancer',
            executable='spike_node',
            name='spike_infra1',
            remappings=[
                ('image_in',  '/camera/infra1/image_rect_raw'),
                ('image_out', '/proc/infra1/image_spike'),
            ],
            parameters=[{
                'threshold': LaunchConfiguration('threshold'),
            }],
        ),

        # ─── 右画像系統 (infra2) ────────────────────────────────────
        Node(
            package='infra_enhancer',
            executable='spike_node',
            name='spike_infra2',
            remappings=[
                ('image_in',  '/camera/infra2/image_rect_raw'),
                ('image_out', '/proc/infra2/image_spike'),
            ],
            parameters=[{
                'threshold': LaunchConfiguration('threshold'),
            }],
        ),
    ])
