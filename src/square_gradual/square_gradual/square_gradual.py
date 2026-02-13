#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import datetime
import os

class MoveSquareGradualNode(Node):
    """
    Visual SLAMのオドメトリを使用して正方形の軌道を描くノード。
    目標距離/角度に近づくにつれて速度が徐々に減少します。
    同時にホイールオドメトリ(/odom)のデータもCSVに記録し、軌道の比較を可能にします。
    """
    def __init__(self):
        super().__init__('move_square_gradual_node')
        
        # ▼▼▼ 設定パラメータ ▼▼▼
        self.target_distance_ = 2.0         # 1辺の長さ (m)
        self.target_angle_ = -math.pi / 2.0 # 回転角度 (rad) (-90度: 時計回り)
        self.rotation_tolerance_ = 0.00     # 回転の許容誤差 (rad)
        self.max_loops_ = 4                 # 正方形なので4辺
        
        # 速度パラメータ
        self.max_linear_velocity_ = 0.1     # 最大前進速度 (m/s)
        self.min_linear_velocity_ = 0.02    # 最小前進速度 (m/s)
        self.max_angular_velocity_ = 0.1    # 最大回転速度 (rad/s)
        self.min_angular_velocity_ = 0.02   # 最小回転速度 (rad/s)
        
        # 減速開始位置 (目標の何%から減速を開始するか)
        self.decel_start_ratio_ = 0.7       # 70%の位置から減速開始
        
        # トピック設定
        self.vslam_topic_ = '/visual_slam/tracking/odometry' # 制御に使用
        self.wheel_topic_ = '/odom'                         # ログ記録のみに使用
        self.cmd_vel_topic_ = '/cmd_vel'
        # ▲▲▲ 設定パラメータ ▲▲▲
        
        # 状態定義
        self.STATE_INIT = "INIT"
        self.STATE_MOVING_FORWARD = "MOVING_FORWARD"
        self.STATE_ROTATING = "ROTATING"
        self.STATE_DONE = "DONE"
        
        # 状態変数 (Visual SLAM用 - 制御に使用)
        self.current_state_ = self.STATE_INIT
        self.current_position_ = None
        self.current_yaw_ = 0.0
        
        # 状態変数 (ホイールオドメトリ用 - ログ記録のみ)
        self.wheel_position_ = None
        self.wheel_yaw_ = 0.0

        # 各動作の開始時点の基準値 (Visual SLAMベース)
        self.start_position_ = None 
        self.start_yaw_ = 0.0      
        
        self.distance_traveled_ = 0.0
        self.angle_traveled_ = 0.0
        self.loop_count_ = 0        
        self.last_yaw_ = 0.0        
        
        self.timer_ = None
        self.init_timer_ = None

        # CSV保存
        self.csv_file_ = None
        self.csv_writer_ = None
        self.csv_filename_ = ""
        try:
            now = datetime.datetime.now()
            log_dir = 'results'
            os.makedirs(log_dir, exist_ok=True)
            self.csv_filename_ = os.path.join(log_dir, f'square_gradual_log_{now.strftime("%Y%m%d_%H%M%S")}.csv')
            self.csv_file_ = open(self.csv_filename_, 'w', newline='', encoding='utf-8')
            self.csv_writer_ = csv.writer(self.csv_file_)
            
            # ヘッダー: v_ がVisual SLAM, w_ がWheel Odometry
            header = [
                'timestamp', 
                'state', 'loop_count',
                'v_x', 'v_y', 'v_z', 'v_yaw',  # Visual SLAM (Control Source)
                'w_x', 'w_y', 'w_z', 'w_yaw',  # Wheel Odometry (Reference)
                'cmd_linear_x', 'cmd_angular_z'  # 指令速度
            ]
            self.csv_writer_.writerow(header)
            self.get_logger().info(f'ログ保存先: {self.csv_filename_}')
        except IOError as e:
            self.get_logger().error(f'CSVエラー: {e}')

        # 現在の指令速度を保持
        self.current_cmd_linear_ = 0.0
        self.current_cmd_angular_ = 0.0

        # パブリッシャー
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic_, 10)

        # サブスクライバー 1: Visual SLAM (制御メイン)
        self.vslam_sub_ = self.create_subscription(
            Odometry,
            self.vslam_topic_,
            self.vslam_callback,
            10)

        # サブスクライバー 2: Wheel Odometry (ログ用)
        self.wheel_sub_ = self.create_subscription(
            Odometry,
            self.wheel_topic_,
            self.wheel_odom_callback,
            10)
        
        self.get_logger().info('ノード起動: V-SLAM制御 + 徐行制御 + ホイールオドメトリ記録')
        self.get_logger().warn('V-SLAMの安定待ち (5秒)...')
        self.init_timer_ = self.create_timer(5.0, self.start_control_loop)

    def start_control_loop(self):
        if self.init_timer_:
            self.init_timer_.cancel()
            self.init_timer_ = None
        
        self.get_logger().info('制御開始。')
        self.current_state_ = self.STATE_MOVING_FORWARD
        self.start_position_ = None 
        self.timer_ = self.create_timer(0.02, self.timer_callback)

    def get_yaw_from_quat(self, q):
        """クォータニオンからヨー角を計算するヘルパー関数"""
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def calculate_gradual_velocity(self, current_progress, target, max_vel, min_vel):
        """
        目標に近づくにつれて速度を徐々に減少させる関数。
        
        Args:
            current_progress: 現在の進行量 (距離または角度の絶対値)
            target: 目標量 (距離または角度の絶対値)
            max_vel: 最大速度
            min_vel: 最小速度
        
        Returns:
            計算された速度
        """
        if target <= 0:
            return min_vel
        
        progress_ratio = current_progress / target
        
        if progress_ratio < self.decel_start_ratio_:
            # 減速開始位置より前は最大速度
            return max_vel
        else:
            # 減速開始位置から目標まで線形に減速
            # decel_start_ratio_ → 1.0 の範囲で max_vel → min_vel に変化
            decel_range = 1.0 - self.decel_start_ratio_
            if decel_range <= 0:
                return min_vel
            
            decel_progress = (progress_ratio - self.decel_start_ratio_) / decel_range
            # 0.0 → 1.0 の範囲を max_vel → min_vel にマッピング
            velocity = max_vel - (max_vel - min_vel) * decel_progress
            return max(min_vel, velocity)

    def wheel_odom_callback(self, msg: Odometry):
        """ホイールオドメトリのデータを受信して保持するだけ"""
        self.wheel_position_ = msg.pose.pose.position
        self.wheel_yaw_ = self.get_yaw_from_quat(msg.pose.pose.orientation)

    def vslam_callback(self, msg: Odometry):
        """Visual SLAMのコールバック。ここでCSV記録と状態更新用の計算を行う"""
        self.current_position_ = msg.pose.pose.position
        self.current_yaw_ = self.get_yaw_from_quat(msg.pose.pose.orientation)

        # CSV記録 (V-SLAM受信時に、最新のホイールオドメトリも一緒に書く)
        if self.csv_writer_:
            try:
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                # ホイールデータがまだ来ていない場合のガード
                wx = self.wheel_position_.x if self.wheel_position_ else 0.0
                wy = self.wheel_position_.y if self.wheel_position_ else 0.0
                wz = self.wheel_position_.z if self.wheel_position_ else 0.0
                wyaw = self.wheel_yaw_
                
                self.csv_writer_.writerow([
                    ts, 
                    self.current_state_, self.loop_count_,
                    self.current_position_.x, self.current_position_.y, self.current_position_.z, self.current_yaw_,
                    wx, wy, wz, wyaw,
                    self.current_cmd_linear_, self.current_cmd_angular_
                ])
            except Exception:
                pass
        
        # 累積回転角の計算 (V-SLAMベース)
        if self.current_state_ == self.STATE_ROTATING and self.start_position_ is not None:
            diff_yaw = self.current_yaw_ - self.last_yaw_
            # -π〜πの範囲に正規化
            while diff_yaw > math.pi:
                diff_yaw -= 2 * math.pi
            while diff_yaw < -math.pi:
                diff_yaw += 2 * math.pi
            self.angle_traveled_ += diff_yaw
        self.last_yaw_ = self.current_yaw_

    def timer_callback(self):
        """制御ループ: 状態に応じてロボットを動かす（徐行制御付き）"""
        if self.current_position_ is None:
            # V-SLAMデータがまだ来ていない
            return

        twist = Twist()

        if self.current_state_ == self.STATE_MOVING_FORWARD:
            # 前進開始時の位置を記録
            if self.start_position_ is None:
                self.start_position_ = self.current_position_
                self.distance_traveled_ = 0.0
                self.get_logger().info(f'辺 {self.loop_count_ + 1}: 前進開始')

            # 移動距離を計算
            dx = self.current_position_.x - self.start_position_.x
            dy = self.current_position_.y - self.start_position_.y
            self.distance_traveled_ = math.sqrt(dx * dx + dy * dy)

            if self.distance_traveled_ >= self.target_distance_:
                # 目標距離に到達 → 回転へ
                twist.linear.x = 0.0
                self.current_cmd_linear_ = 0.0
                self.current_cmd_angular_ = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info(f'辺 {self.loop_count_ + 1}: 前進完了 ({self.distance_traveled_:.2f}m)')
                self.current_state_ = self.STATE_ROTATING
                self.start_position_ = None
                self.angle_traveled_ = 0.0
                self.start_yaw_ = self.current_yaw_
            else:
                # 前進継続（徐行制御付き）
                velocity = self.calculate_gradual_velocity(
                    self.distance_traveled_,
                    self.target_distance_,
                    self.max_linear_velocity_,
                    self.min_linear_velocity_
                )
                twist.linear.x = velocity
                self.current_cmd_linear_ = velocity
                self.current_cmd_angular_ = 0.0
                self.publisher_.publish(twist)

        elif self.current_state_ == self.STATE_ROTATING:
            # 回転開始時の基準を設定
            if self.start_position_ is None:
                self.start_position_ = self.current_position_
                self.start_yaw_ = self.current_yaw_
                self.angle_traveled_ = 0.0
                self.get_logger().info(f'辺 {self.loop_count_ + 1}: 回転開始')

            # 目標角度に到達したか確認
            if abs(self.angle_traveled_) >= abs(self.target_angle_) - self.rotation_tolerance_:
                # 回転完了
                twist.angular.z = 0.0
                self.current_cmd_linear_ = 0.0
                self.current_cmd_angular_ = 0.0
                self.publisher_.publish(twist)
                self.loop_count_ += 1
                self.get_logger().info(f'辺 {self.loop_count_}: 回転完了 ({math.degrees(self.angle_traveled_):.1f}°)')

                if self.loop_count_ >= self.max_loops_:
                    # 全ての辺を完了
                    self.current_state_ = self.STATE_DONE
                    self.get_logger().info('正方形軌道完了！')
                    self._cleanup()
                else:
                    # 次の辺へ
                    self.current_state_ = self.STATE_MOVING_FORWARD
                    self.start_position_ = None
            else:
                # 回転継続（徐行制御付き）
                angular_velocity = self.calculate_gradual_velocity(
                    abs(self.angle_traveled_),
                    abs(self.target_angle_),
                    self.max_angular_velocity_,
                    self.min_angular_velocity_
                )
                # 時計回り: 負の角速度
                if self.target_angle_ < 0:
                    angular_velocity = -angular_velocity
                
                twist.angular.z = angular_velocity
                self.current_cmd_linear_ = 0.0
                self.current_cmd_angular_ = angular_velocity
                self.publisher_.publish(twist)

        elif self.current_state_ == self.STATE_DONE:
            # 完了状態: 何もしない
            pass

    def _cleanup(self):
        """終了処理"""
        # タイマー停止
        if self.timer_:
            self.timer_.cancel()
            self.timer_ = None
        # CSVファイルを閉じる
        if self.csv_file_:
            self.csv_file_.close()
            self.csv_file_ = None
            self.get_logger().info(f'CSV保存完了: {self.csv_filename_}')


def main(args=None):
    rclpy.init(args=args)
    node = MoveSquareGradualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
