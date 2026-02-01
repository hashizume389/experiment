#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import datetime

class MoveForwardVisualSlamNode(Node):
    """
    SCOUT MINIを指定された距離だけ前進させ、
    Visual SLAM(制御用)とホイールオドメトリ(比較用)の両方をCSVに記録するノード。
    """
    def __init__(self):
        super().__init__('move_forward_vslam_node')
        
        # ▼▼▼ 設定パラメータ ▼▼▼
        self.target_distance_ = 1.0   # 目標距離 (メートル)
        self.linear_velocity_ = 0.1   # 前進速度 (メートル/秒)
        
        self.vslam_topic_ = '/visual_slam/tracking/odometry' # 制御用
        self.wheel_topic_ = '/odom'                          # ログ記録用
        self.cmd_vel_topic_ = '/cmd_vel'
        # ▲▲▲ 設定パラメータ ▲▲▲
        
        # 状態変数
        self.initial_position_ = None # V-SLAMの初期位置
        self.current_position_ = None # V-SLAMの現在位置
        
        self.wheel_position_ = None   # ホイールオドメトリの現在位置
        
        self.distance_traveled_ = 0.0
        self.goal_reached_ = False
        self.timer_ = None
        self.init_timer_ = None

        # CSV保存用の変数
        self.csv_file_ = None
        self.csv_writer_ = None
        self.csv_filename_ = ""
        
        try:
            now = datetime.datetime.now()
            # ファイル名を変更
            self.csv_filename_ = f'forward_compare_log_{now.strftime("%Y%m%d_%H%M%S")}.csv'
            
            self.csv_file_ = open(self.csv_filename_, 'w', newline='', encoding='utf-8')
            self.csv_writer_ = csv.writer(self.csv_file_)
            
            # ヘッダー書き込み (v_: Visual SLAM, w_: Wheel Odometry)
            self.csv_writer_.writerow(['timestamp', 'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z'])
            
            self.get_logger().info(f'比較ログデータを [{self.csv_filename_}] に保存します。')

        except IOError as e:
            self.get_logger().error(f'CSVファイルのオープンに失敗しました: {e}')
            self.csv_file_ = None
            self.csv_writer_ = None

        # パブリッシャー
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic_, 10)
        
        # サブスクライバー 1: Visual SLAM (制御用メイン)
        self.vslam_sub_ = self.create_subscription(
            Odometry,
            self.vslam_topic_,
            self.vslam_callback,
            10)

        # サブスクライバー 2: Wheel Odometry (ログ用サブ)
        self.wheel_sub_ = self.create_subscription(
            Odometry,
            self.wheel_topic_,
            self.wheel_odom_callback,
            10)
        
        self.get_logger().info(
            f'ノード起動: {self.target_distance_}m 前進します (Visual SLAMベース)...'
        )
        self.get_logger().info(f'トピック待機中: {self.vslam_topic_} & {self.wheel_topic_}')

        self.get_logger().warn('V-SLAMの安定を待機中... (5秒)')
        self.init_timer_ = self.create_timer(5.0, self.start_control_loop)

    def start_control_loop(self):
        if self.init_timer_:
            self.init_timer_.cancel()
            self.init_timer_ = None
        
        self.get_logger().info('待機完了。制御ループを開始します。')
        self.timer_ = self.create_timer(0.02, self.timer_callback)

    def wheel_odom_callback(self, msg: Odometry):
        """ホイールオドメトリのデータを受信して保持する"""
        self.wheel_position_ = msg.pose