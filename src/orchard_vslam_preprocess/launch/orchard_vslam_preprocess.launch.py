from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("orchard_vslam_preprocess")
    params_file = os.path.join(package_share, "config", "orchard_vslam_preprocess.yaml")
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Path to the orchard_vslam_preprocess parameter file",
    )

    return LaunchDescription([
        params_arg,
        Node(
            package="orchard_vslam_preprocess",
            executable="orchard_vslam_preprocess_node",
            name="orchard_vslam_preprocess_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        )
    ])
