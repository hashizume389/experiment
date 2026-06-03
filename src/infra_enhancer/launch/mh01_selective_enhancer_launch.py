"""MH_01 EuRoC selective lightweight preprocessing launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def selective_node(name, image_in, image_out):
    return Node(
        package='infra_enhancer',
        executable='selective_enhancer_node',
        name=name,
        remappings=[
            ('image_in', image_in),
            ('image_out', image_out),
        ],
        parameters=[{
            'metric_scale': ParameterValue(LaunchConfiguration('metric_scale'), value_type=float),
            'dark_mean': ParameterValue(LaunchConfiguration('dark_mean'), value_type=float),
            'bright_mean': ParameterValue(LaunchConfiguration('bright_mean'), value_type=float),
            'low_contrast': ParameterValue(LaunchConfiguration('low_contrast'), value_type=float),
            'target_contrast': ParameterValue(LaunchConfiguration('target_contrast'), value_type=float),
            'saturation_percent': ParameterValue(LaunchConfiguration('saturation_percent'), value_type=float),
            'gradient_low': ParameterValue(LaunchConfiguration('gradient_low'), value_type=float),
            'gamma_dark': ParameterValue(LaunchConfiguration('gamma_dark'), value_type=float),
            'gamma_bright': ParameterValue(LaunchConfiguration('gamma_bright'), value_type=float),
            'max_blend': ParameterValue(LaunchConfiguration('max_blend'), value_type=float),
            'unsharp_amount': ParameterValue(LaunchConfiguration('unsharp_amount'), value_type=float),
            'unsharp_sigma': ParameterValue(LaunchConfiguration('unsharp_sigma'), value_type=float),
            'enable_unsharp': ParameterValue(LaunchConfiguration('enable_unsharp'), value_type=bool),
            'debug_log_every': ParameterValue(LaunchConfiguration('debug_log_every'), value_type=int),
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('left_image_in', default_value='/cam0/image_rect'),
        DeclareLaunchArgument('right_image_in', default_value='/cam1/image_rect'),
        DeclareLaunchArgument('left_image_out', default_value='/mh01/cam0/image_selective'),
        DeclareLaunchArgument('right_image_out', default_value='/mh01/cam1/image_selective'),
        DeclareLaunchArgument('metric_scale', default_value='0.5'),
        DeclareLaunchArgument('dark_mean', default_value='70.0'),
        DeclareLaunchArgument('bright_mean', default_value='185.0'),
        DeclareLaunchArgument('low_contrast', default_value='42.0'),
        DeclareLaunchArgument('target_contrast', default_value='70.0'),
        DeclareLaunchArgument('saturation_percent', default_value='2.0'),
        DeclareLaunchArgument('gradient_low', default_value='18.0'),
        DeclareLaunchArgument('gamma_dark', default_value='0.72'),
        DeclareLaunchArgument('gamma_bright', default_value='1.15'),
        DeclareLaunchArgument('max_blend', default_value='0.85'),
        DeclareLaunchArgument('unsharp_amount', default_value='0.25'),
        DeclareLaunchArgument('unsharp_sigma', default_value='1.0'),
        DeclareLaunchArgument('enable_unsharp', default_value='true'),
        DeclareLaunchArgument('debug_log_every', default_value='0'),
        selective_node(
            'mh01_selective_cam0',
            LaunchConfiguration('left_image_in'),
            LaunchConfiguration('left_image_out'),
        ),
        selective_node(
            'mh01_selective_cam1',
            LaunchConfiguration('right_image_in'),
            LaunchConfiguration('right_image_out'),
        ),
    ])
