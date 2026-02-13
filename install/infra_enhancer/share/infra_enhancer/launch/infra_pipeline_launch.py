"""InfraEnhancer パイプライン Launch ファイル

Denoise → CLAHE → Normalize の3段パイプラインを左右画像それぞれに構築する。
各ノードのトピックを remappings でチェーン接続する。

使用例:
  ros2 launch infra_enhancer infra_pipeline_launch.py
  ros2 launch infra_enhancer infra_pipeline_launch.py denoise_method:=bilateral
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Launch Arguments ───────────────────────────────────────
    denoise_method_arg = DeclareLaunchArgument(
        'denoise_method', default_value='gaussian',
        description='Denoise method: gaussian | bilateral | none')
    clahe_clip_arg = DeclareLaunchArgument(
        'clahe_clip', default_value='2.0',
        description='CLAHE clip limit')
    clahe_tile_arg = DeclareLaunchArgument(
        'clahe_tile', default_value='8',
        description='CLAHE tile grid size')

    # ─── 左画像系統 (infra1) ────────────────────────────────────
    denoise_infra1 = Node(
        package='infra_enhancer',
        executable='denoise_node',
        name='denoise_infra1',
        remappings=[
            ('image_in',  '/camera/infra1/image_rect_raw'),
            ('image_out', '/proc/infra1/image_denoised'),
        ],
        parameters=[{
            'method': LaunchConfiguration('denoise_method'),
        }],
    )

    clahe_infra1 = Node(
        package='infra_enhancer',
        executable='clahe_node',
        name='clahe_infra1',
        remappings=[
            ('image_in',  '/proc/infra1/image_denoised'),
            ('image_out', '/proc/infra1/image_clahe'),
        ],
        parameters=[{
            'clip_limit': LaunchConfiguration('clahe_clip'),
            'tile_size':  LaunchConfiguration('clahe_tile'),
        }],
    )

    normalize_infra1 = Node(
        package='infra_enhancer',
        executable='normalize_node',
        name='normalize_infra1',
        remappings=[
            ('image_in',  '/proc/infra1/image_clahe'),
            ('image_out', '/proc/infra1/image_enhanced'),
        ],
    )

    # ─── 右画像系統 (infra2) ────────────────────────────────────
    denoise_infra2 = Node(
        package='infra_enhancer',
        executable='denoise_node',
        name='denoise_infra2',
        remappings=[
            ('image_in',  '/camera/infra2/image_rect_raw'),
            ('image_out', '/proc/infra2/image_denoised'),
        ],
        parameters=[{
            'method': LaunchConfiguration('denoise_method'),
        }],
    )

    clahe_infra2 = Node(
        package='infra_enhancer',
        executable='clahe_node',
        name='clahe_infra2',
        remappings=[
            ('image_in',  '/proc/infra2/image_denoised'),
            ('image_out', '/proc/infra2/image_clahe'),
        ],
        parameters=[{
            'clip_limit': LaunchConfiguration('clahe_clip'),
            'tile_size':  LaunchConfiguration('clahe_tile'),
        }],
    )

    normalize_infra2 = Node(
        package='infra_enhancer',
        executable='normalize_node',
        name='normalize_infra2',
        remappings=[
            ('image_in',  '/proc/infra2/image_clahe'),
            ('image_out', '/proc/infra2/image_enhanced'),
        ],
    )

    return LaunchDescription([
        # Arguments
        denoise_method_arg,
        clahe_clip_arg,
        clahe_tile_arg,
        # Infra1 pipeline
        denoise_infra1,
        clahe_infra1,
        normalize_infra1,
        # Infra2 pipeline
        denoise_infra2,
        clahe_infra2,
        normalize_infra2,
    ])
