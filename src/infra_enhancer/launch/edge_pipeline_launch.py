"""InfraEnhancer エッジ検出パイプライン Launch ファイル

エッジ検出フィルタノード (edge_node) を起動する。
左右画像 (infra1, infra2) それぞれに対してノードを立ち上げる。

使用例:
  ros2 launch infra_enhancer edge_pipeline_launch.py
  ros2 launch infra_enhancer edge_pipeline_launch.py method:=sobel
  ros2 launch infra_enhancer edge_pipeline_launch.py method:=canny canny_low:=30 canny_high:=100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Launch Arguments ───────────────────────────────────────
    method_arg = DeclareLaunchArgument(
        'method', default_value='laplacian',
        description='Edge detection method: laplacian | sobel | canny')
    ksize_arg = DeclareLaunchArgument(
        'ksize', default_value='3',
        description='Kernel size for Laplacian/Sobel')
    canny_low_arg = DeclareLaunchArgument(
        'canny_low', default_value='50',
        description='Canny low threshold')
    canny_high_arg = DeclareLaunchArgument(
        'canny_high', default_value='150',
        description='Canny high threshold')

    return LaunchDescription([
        method_arg,
        ksize_arg,
        canny_low_arg,
        canny_high_arg,

        # ─── 左画像系統 (infra1) ────────────────────────────────────
        Node(
            package='infra_enhancer',
            executable='edge_node',
            name='edge_infra1',
            remappings=[
                ('image_in',  '/camera/infra1/image_rect_raw'),
                ('image_out', '/proc/infra1/image_edge'),
            ],
            parameters=[{
                'method': LaunchConfiguration('method'),
                'ksize': LaunchConfiguration('ksize'),
                'canny_low': LaunchConfiguration('canny_low'),
                'canny_high': LaunchConfiguration('canny_high'),
            }],
        ),

        # ─── 右画像系統 (infra2) ────────────────────────────────────
        Node(
            package='infra_enhancer',
            executable='edge_node',
            name='edge_infra2',
            remappings=[
                ('image_in',  '/camera/infra2/image_rect_raw'),
                ('image_out', '/proc/infra2/image_edge'),
            ],
            parameters=[{
                'method': LaunchConfiguration('method'),
                'ksize': LaunchConfiguration('ksize'),
                'canny_low': LaunchConfiguration('canny_low'),
                'canny_high': LaunchConfiguration('canny_high'),
            }],
        ),
    ])
