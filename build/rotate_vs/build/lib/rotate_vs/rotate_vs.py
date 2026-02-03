#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import datetime

class RotateInPlaceVisualSlamNode(Node):
    """
    Visual SLAMとホイールオドメトリの両方を記録しながら、
    その場で0.5回転（時計回り）して停止するノード。
    """
    def __init__(self):
        super().__init__('rotate_in_place_vslam_node')
        
        # 設定
        self.target_angle_ = -0.5 * 2.0 * math.pi 
        self.rotation_tolerance_ = 0.0
        self.angular_velocity_ = 0.1
        
        # トピック名
        self.vslam_topic_ = '/visual_slam/tracking/odometry'
        self.wheel_topic_ = '/odom'
        self.cmd_vel_topic_ = '/cmd_vel'
    
        # 状態
        self.STATE_INIT = "INIT"
        self.STATE_ROTATING = "ROTATING"
        self.STATE_DONE = "DONE"
        self.current_state_ = self.STATE_INIT
        
        # V-SLAMデータ用変数
        self.vslam_initial_pos_ = None
        self.vslam_yaw_ = 0.0
        self.vslam_pos_ = None
        self.angle_traveled_ = 0.0 
        self.last_vslam_yaw_ = 0.0

        # ホイールオドメトリデータ用変数
        self.wheel_yaw_ = 0.0
        self.wheel_pos_ = None
        
        # CSVセットアップ
        self.csv_file_ = None
        self.csv_writer_ = None
        self.setup_csv()

        # パブリッシャー
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic_, 10)

        # サブスクライバー (V-SLAM)
        self.vslam_sub_ = self.create_subscription(
            Odometry, self.vslam_topic_, self.vslam_callback, 10)
        
        # サブスクライバー (ホイール)
        self.wheel_sub_ = self.create_subscription(
            Odometry, self.wheel_topic_, self.wheel_callback, 10)
        
        self.get_logger().info(f'V-SLAM: {self.vslam_topic_} / Wheel: {self.wheel_topic_} を記録します。')

        # SLAMの安定待ち（5秒）
        self.get_logger().warn('V-SLAMの安定を待機中... (5秒)')
        self.init_timer_ = self.create_timer(5.0, self.start_control_loop)

    def setup_csv(self):
        try:
            now = datetime.datetime.now()
            self.csv_filename_ = f'combined_odom_log_{now.strftime("%Y%m%d_%H%M%S")}.csv'
            self.csv_file_ = open(self.csv_filename_, 'w', newline='', encoding='utf-8')
            self.csv_writer_ = csv.writer(self.csv_file_)
            # ヘッダーにホイールオドメトリ項目を追加
            self.csv_writer_.writerow([
                'timestamp', 
                'vslam_x', 'vslam_y', 'vslam_yaw',
                'wheel_x', 'wheel_y', 'wheel_yaw'
            ])
            self.get_logger().info(f'データを [{self.csv_filename_}] に保存します。')
        except IOError as e:
            self.get_logger().error(f'CSVファイルのオープンに失敗しました: {e}')

    def get_yaw_from_quat(self, q):
        """クォータニオンからYaw角を計算"""
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def vslam_callback(self, msg: Odometry):
        self.vslam_pos_ = msg.pose.pose.position
        self.vslam_yaw_ = self.get_yaw_from_quat(msg.pose.pose.orientation)

        # 初期値記録
        if self.vslam_initial_pos_ is None:
            self.vslam_initial_pos_ = self.vslam_pos_
            self.last_vslam_yaw_ = self.vslam_yaw_
            self.get_logger().info(f'初期ヨー角(V-SLAM): {self.vslam_yaw_:.2f} rad')

        # 累積回転角の計算
        if self.vslam_initial_pos_ is not None and self.current_state_ == self.STATE_ROTATING:
            diff_yaw = self.vslam_yaw_ - self.last_vslam_yaw_
            if diff_yaw > math.pi: diff_yaw -= 2 * math.pi
            elif diff_yaw < -math.pi: diff_yaw += 2 * math.pi
            self.angle_traveled_ += diff_yaw
        
        self.last_vslam_yaw_ = self.vslam_yaw_
        
        # ログ記録 (V-SLAMの更新タイミングで記録)
        self.write_to_csv(msg.header.stamp)

    def wheel_callback(self, msg: Odometry):
        self.wheel_pos_ = msg.pose.pose.position
        self.wheel_yaw_ = self.get_yaw_from_quat(msg.pose.pose.orientation)

    def write_to_csv(self, stamp):
        if self.csv_writer_ and self.vslam_pos_ and self.wheel_pos_:
            try:
                timestamp = stamp.sec + stamp.nanosec * 1e-9
                self.csv_writer_.writerow([
                    timestamp,
                    self.vslam_pos_.x, self.vslam_pos_.y, self.vslam_yaw_,
                    self.wheel_pos_.x, self.wheel_pos_.y, self.wheel_yaw_
                ])
            except Exception as e:
                self.get_logger().error(f"CSV書き込みエラー: {e}", throttle_duration_sec=10)

    def start_control_loop(self):
        if self.init_timer_:
            self.init_timer_.cancel()
        self.current_state_ = self.STATE_ROTATING
        self.timer_ = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('制御ループを開始しました。')

    def timer_callback(self):
        if self.vslam_initial_pos_ is None:
            self.get_logger().warn('V-SLAMトピック待機中...', throttle_duration_sec=5)
            return

        twist_msg = Twist()
        if self.current_state_ == self.STATE_ROTATING:
            if self.angle_traveled_ > (self.target_angle_ + self.rotation_tolerance_):
                twist_msg.angular.z = -self.angular_velocity_
                self.get_logger().info(
                    f'回転中: {self.angle_traveled_:.2f} / {self.target_angle_:.2f} rad',
                    throttle_duration_sec=1
                )
            else:
                self.get_logger().info('目標到達。')
                self.current_state_ = self.STATE_DONE
                self.close_csv_file()
        
        self.publisher_.publish(twist_msg)

    def close_csv_file(self):
        if self.csv_file_:
            self.csv_file_.close()
            self.csv_file_ = None
            self.get_logger().info(f'CSV [{self.csv_filename_}] を保存完了。')

def main(args=None):
    rclpy.init(args=args)
    node = RotateInPlaceVisualSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())
        node.close_csv_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()