"""MH_01 EuRoC orchard VSLAM preprocessing launch file.

This launch starts only the preprocessing outputs needed by Visual SLAM:
preprocessed stereo images and copied CameraInfo. Debug keypoint and reliability
map publication is disabled.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "left_image_topic",
            default_value="/cam0/image_rect",
            description="Rectified MH_01 left image topic.",
        ),
        DeclareLaunchArgument(
            "right_image_topic",
            default_value="/cam1/image_rect",
            description="Rectified MH_01 right image topic.",
        ),
        DeclareLaunchArgument(
            "left_camera_info_topic",
            default_value="/cam0/camera_info",
            description="Rectified MH_01 left CameraInfo topic.",
        ),
        DeclareLaunchArgument(
            "right_camera_info_topic",
            default_value="/cam1/camera_info",
            description="Rectified MH_01 right CameraInfo topic.",
        ),
        DeclareLaunchArgument(
            "left_output_image_topic",
            default_value="/mh01/cam0/image_orchard_preprocessed",
            description="Preprocessed left image topic for Visual SLAM.",
        ),
        DeclareLaunchArgument(
            "right_output_image_topic",
            default_value="/mh01/cam1/image_orchard_preprocessed",
            description="Preprocessed right image topic for Visual SLAM.",
        ),
        DeclareLaunchArgument(
            "publish_every_n_frames",
            default_value="2",
            description="Publish one preprocessed frame for every N input frames.",
        ),
        Node(
            package="orchard_vslam_preprocess",
            executable="orchard_vslam_preprocess_node",
            name="orchard_vslam_preprocess_node",
            output="screen",
            parameters=[{
                "left_image_topic": LaunchConfiguration("left_image_topic"),
                "right_image_topic": LaunchConfiguration("right_image_topic"),
                "left_camera_info_topic": LaunchConfiguration("left_camera_info_topic"),
                "right_camera_info_topic": LaunchConfiguration("right_camera_info_topic"),
                "left_output_image_topic": LaunchConfiguration("left_output_image_topic"),
                "right_output_image_topic": LaunchConfiguration("right_output_image_topic"),
                "left_output_camera_info_topic": "/mh01/cam0/camera_info",
                "right_output_camera_info_topic": "/mh01/cam1/camera_info",
                "use_gamma": True,
                "gamma": 0.8,
                "use_clahe": True,
                "clahe_clip_limit": 2.0,
                "clahe_tile_grid_size": 8,
                "feature_detector_type": "FAST",
                "orb_nfeatures": 600,
                "fast_threshold": 24,
                "w_sobel": 0.30,
                "w_laplacian": 0.20,
                "w_variance": 0.20,
                "w_temporal": 0.20,
                "w_top": 0.10,
                "w_density": 0.10,
                "top_penalty_ratio": 0.30,
                "local_variance_kernel_size": 5,
                "reliability_blur_kernel_size": 0,
                "grid_rows": 4,
                "grid_cols": 6,
                "max_keypoints_per_cell": 12,
                "gaussian_sigma": 7.0,
                "keypoint_boost_strength": 0.28,
                "min_modulation_weight": 0.60,
                "max_modulation_weight": 1.20,
                "publish_reliability": False,
                "publish_debug": False,
                "use_stereo_sync": False,
                "publish_every_n_frames": ParameterValue(
                    LaunchConfiguration("publish_every_n_frames"),
                    value_type=int,
                ),
                "log_interval": 30,
            }],
        ),
    ])
