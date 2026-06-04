from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ground_truth_csv',
            default_value=(
                '/home/hashizume/experiment/MH_01_easy/'
                'mav0/state_groundtruth_estimate0/data.csv'
            ),
            description=(
                'Path to EuRoC state_groundtruth_estimate0/data.csv or the exported '
                '/leica/position CSV.'
            ),
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/visual_slam/tracking/odometry',
            description='Visual SLAM odometry topic to log.',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/hashizume/experiment/results',
            description='Directory for output CSV logs.',
        ),
        DeclareLaunchArgument(
            'max_match_dt_sec',
            default_value='0.1',
            description='Warn when the nearest/interpolated ground truth is farther than this value.',
        ),
        DeclareLaunchArgument(
            'qos_reliability',
            default_value='reliable',
            description='Odometry subscription reliability: reliable or best_effort.',
        ),
        Node(
            package='mh01_eval_logger',
            executable='mh01_eval_logger',
            name='mh01_eval_logger',
            output='screen',
            parameters=[{
                'ground_truth_csv': LaunchConfiguration('ground_truth_csv'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'max_match_dt_sec': ParameterValue(
                    LaunchConfiguration('max_match_dt_sec'),
                    value_type=float,
                ),
                'qos_reliability': LaunchConfiguration('qos_reliability'),
            }],
        ),
    ])
