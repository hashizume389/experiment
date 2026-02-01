#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import datetime

class RotateInPlaceWheelOdomNode(Node):
    """
    ホイールオドメトリのみを使用して、
    その場で0.5回転（時計回り）して停止するノード。
    """
    def __init__(self):
        super().__init__('rotate_in_place_wheel_odom_node')
        
        # --- 設定パラメータ ---
        self.target_angle_ = -0.5 * 2.0 * math.pi  # -π (半周・時計回り)
        self.rotation_tolerance_ = 0.0             # 許容誤差 (rad)
        self.angular_velocity_ = 0.1               # 回転速度 (rad/s)
        
        # トピック名
        self.odom_topic_ = '/odom'
        self.cmd_vel_topic_ = '/cmd_vel'
    
        # --- 状態変数 ---
        self.STATE_INIT = "INIT"
        self.STATE_ROTATING = "ROTATING"
        self.STATE_DONE = "DONE"
        self.current_state_ = self.STATE_INIT
        
        self.current_yaw_ = 0.0
        self.last_yaw_ = 0.0
        self.angle_traveled_ = 0.0  # 累積回転角
        self.initial_odom_received_ = False

        # --- CSV保存設定 (ホイールオドメトリのみ) ---
        self.setup_csv()

        # --- 通信設定 ---
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic_, 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            self.odom_topic_,
            self.odom_callback,
            10)
        
        self.get_logger().info(f'ノード起動: ホイールオドメトリ基準で回転します...')
        
        # 制御ループ開始用のタイマー（少し待機してから開始）
        self.init_timer_ = self.create_timer(1.0, self.start_control_loop)

    def setup_csv(self):
        try:
            now = datetime.datetime.now()
            self.csv_filename_ = f'wheel_odom_log_{now.strftime("%Y%m%d_%H%M%S")}.csv'
            self.csv_file_ = open(self.csv_filename_, 'w', newline='', encoding='utf-8')
            self.csv_writer_ = csv.writer(self.csv_file_)
            self.csv_writer_.writerow(['timestamp', 'x', 'y', 'yaw'])
            self.get_logger().info(f'ログを [{self.csv_filename_}] に保存します。')
        except IOError as e:
            self.get_logger().error(f'CSVオープン失敗: {e}')

    def get_yaw_from_quat(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg: Odometry):
        # 現在のYaw角を取得
        self.current_yaw_ = self.get_yaw_from_quat(msg.pose.pose.orientation)
        pos = msg.pose.pose.position

        # 初回受信時の処理
        if not self.initial_odom_received_:
            self.last_yaw_ = self.current_yaw_
            self.initial_odom_received_ = True
            self.get_logger().info(f'初期ヨー角(Wheel): {self.current_yaw_:.2f} rad')

        # 累積回転角の更新（回転状態の時のみ）
        if self.current_state_ == self.STATE_ROTATING:
            diff_yaw = self.current_yaw_ - self.last_yaw_
            
            # ラップアラウンド補正 (-π <-> π)
            if diff_yaw > math.pi:
                diff_yaw -= 2 * math.pi
            elif diff_yaw < -math.pi:
                diff_yaw += 2 * math.pi
            
            self.angle_traveled_ += diff_yaw
        
        self.last_yaw_ = self.current_yaw_

        # CSV記録
        if self.csv_writer_:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.csv_writer_.writerow([timestamp, pos.x, pos.y, self.current_yaw_])

    def start_control_loop(self):
        if self.init_timer_:
            self.init_timer_.cancel()
        self.current_state_ = self.STATE_ROTATING
        self.timer_ = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if not self.initial_odom_received_:
            return

        twist_msg = Twist()

        if self.current_state_ == self.STATE_ROTATING:
            # 目標角度に達したかチェック (時計回りなので累積角がtargetより小さくなったら停止)
            if self.angle_traveled_ > (self.target_angle_ + self.rotation_tolerance_):
                twist_msg.angular.z = -self.angular_velocity_
                self.get_logger().info(
                    f'回転中: {self.angle_traveled_:.2f} / {self.target_angle_:.2f} rad',
                    throttle_duration_sec=1
                )
            else:
                self.get_logger().info(f'目標角度に到達しました。')
                self.current_state_ = self.STATE_DONE
                self.close_csv_file()

        self.publisher_.publish(twist_msg)

    def close_csv_file(self):
        if self.csv_file_:
            self.csv_file_.close()
            self.csv_file_ = None
            self.get_logger().info(f'CSVファイルをクローズしました。')

def main(args=None):
    rclpy.init(args=args)
    node = RotateInPlaceWheelOdomNode()
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