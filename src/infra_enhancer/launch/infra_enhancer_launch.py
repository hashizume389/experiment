"""Launch file for infra_enhancer node"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='infra_enhancer',
            executable='infra_enhancer_node',
            name='infra_enhancer',
            output='screen',
            parameters=[
                {'queue_size': 10},
                {'slop': 0.02},
                {'clahe_clip': 2.0},
                {'clahe_tile': 8},
                {'denoise': 'gaussian'},
                {'gaussian_ksize': 3},
            ],
        )
    ])
