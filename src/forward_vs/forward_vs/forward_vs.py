#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import datetime
import os

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
            log_dir = 'results'
            os.makedirs(log_dir, exist_ok=True)
            self.csv_filename_ = os.path.join(log_dir, f'forward_compare_log_{now.strftime("%Y%m%d_%H%M%S")}.csv')
            
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
        self.wheel_position_ = msg.pose.pose.position

    def vslam_callback(self, msg: Odometry):
        """Visual SLAMのデータを受信し、CSV記録と制御用の位置を更新する"""
        self.current_position_ = msg.pose.pose.position
        
        # CSV記録 (V-SLAM受信時に、最新のホイールオドメトリも一緒に書く)
        if self.csv_writer_:
            try:
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                # ホイールデータがまだ来ていない場合のガード
                wx = self.wheel_position_.x if self.wheel_position_ else 0.0
                wy = self.wheel_position_.y if self.wheel_position_ else 0.0
                wz = self.wheel_position_.z if self.wheel_position_ else 0.0
                
                self.csv_writer_.writerow([
                    ts,
                    self.current_position_.x, self.current_position_.y, self.current_position_.z,
                    wx, wy, wz
                ])
            except Exception as e:
                self.get_logger().error(f"CSV書き込みエラー: {e}", throttle_duration_sec=10)
        
        # 最初の位置を記録
        if self.initial_position_ is None:
            self.initial_position_ = self.current_position_
            self.get_logger().info(
                f'初期位置を記録 (V-SLAM): x={self.initial_position_.x:.2f}, y={self.initial_position_.y:.2f}'
            )

    def timer_callback(self):
        """制御ループ。50Hzで実行されます。"""
        # オドメトリをまだ受信していないか、目標達成済みの場合は何もしない
        if self.initial_position_ is None:
            self.get_logger().warn('まだV-SLAMデータを受信していません...', throttle_duration_sec=5)
            return

        if self.goal_reached_:
            # 目標達成後は停止コマンドを送り続ける
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            return

        # 初期位置からの移動距離（ユークリッド距離）を計算
        dx = self.current_position_.x - self.initial_position_.x
        dy = self.current_position_.y - self.initial_position_.y
        self.distance_traveled_ = math.sqrt(dx*dx + dy*dy)

        # 目標距離に達しているかチェック
        if self.distance_traveled_ < self.target_distance_:
            # 目標未達: 前進コマンドを送信
            move_msg = Twist()
            move_msg.linear.x = self.linear_velocity_
            self.publisher_.publish(move_msg)
            
            self.get_logger().info(
                f'移動中... {self.distance_traveled_:.2f}m / {self.target_distance_}m',
                throttle_duration_sec=1
            )
        else:
            # 目標達成: 停止コマンドを送信し、フラグを立てる
            self.goal_reached_ = True
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            
            self.get_logger().info(
                f'目標距離 {self.target_distance_}m に到達しました。ロボットを停止します。'
            )
            self.close_csv_file()
            self.get_logger().info('ノードを終了するには Ctrl+C を押してください。')

    def close_csv_file(self):
        """CSVファイルをクローズする"""
        if self.csv_file_:
            self.csv_file_.close()
            self.csv_file_ = None
            self.get_logger().info(f'CSVファイル [{self.csv_filename_}] をクローズしました。')

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardVisualSlamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('キーボード割り込みによりシャットダウンします...')
    finally:
        # シャットダウン時に必ず停止コマンドを送信する
        if rclpy.ok():
            node.get_logger().info('最後に停止コマンドを送信します。')
            node.publisher_.publish(Twist()) # 停止コマンド
            
            node.close_csv_file()
            
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()