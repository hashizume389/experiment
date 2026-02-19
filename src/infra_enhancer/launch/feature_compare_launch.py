"""Visual SLAM 特徴点比較用 Launch ファイル

Visual SLAM が検出した特徴点 (observations_cloud) を
raw 画像と LDFE 処理後画像の両方に投影して可視化する。

トピック構成:
  入力:
    /camera/infra1/image_rect_raw           (raw カメラ画像)
    /proc/infra1/image_ldfe                 (LDFE 処理後画像)
    /visual_slam/vis/observations_cloud     (SLAM 特徴点)
    /camera/infra1/camera_info              (カメラ内部パラメータ)

  出力:
    /debug/infra1/slam_features_raw         (raw に特徴点描画)
    /debug/infra1/slam_features_enhanced    (LDFE に特徴点描画)
    /debug/infra1/feature_count             (特徴点数)

使用例:
  # 1. rosbag 再生 (SLAM 出力を含む bag)
  ros2 bag play ~/experiment/bag/rosbag2_... -l

  # 2. 特徴点可視化ノード起動
  ros2 launch infra_enhancer feature_compare_launch.py

  # 3. rqt_image_view で比較
  ros2 run rqt_image_view rqt_image_view
  # → /debug/infra1/slam_features_raw と
  #   /debug/infra1/slam_features_enhanced を選択して比較
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ─── SLAM 特徴点を raw / LDFE 両方の画像に投影・描画 ──────
        Node(
            package='infra_enhancer',
            executable='feature_overlay_node',
            name='feature_overlay_slam',
            remappings=[
                ('image_in',       '/camera/infra1/image_rect_raw'),
                ('image_in_2',     '/proc/infra1/image_ldfe'),
                ('cloud_in',       '/visual_slam/vis/observations_cloud'),
                ('camera_info_in', '/camera/infra1/camera_info'),
                ('image_out',      '/debug/infra1/slam_features_raw'),
                ('image_out_2',    '/debug/infra1/slam_features_enhanced'),
                ('feature_count',  '/debug/infra1/feature_count'),
            ],
            parameters=[{
                'marker_color_r': 0,
                'marker_color_g': 255,
                'marker_color_b': 0,
                'marker_radius': 3,
                'sync_slop': 0.1,
            }],
        ),
    ])
