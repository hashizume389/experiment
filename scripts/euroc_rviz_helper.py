#!/usr/bin/env python3
"""Republish EuRoC bag topics in RViz-friendly forms."""

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class EurocRvizHelper(Node):
    def __init__(self) -> None:
        super().__init__("euroc_rviz_helper")
        self.declare_parameter("fixed_frame", "map")
        self.fixed_frame = self.get_parameter("fixed_frame").value

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.point_pub = self.create_publisher(PointStamped, "/leica/position_map", 10)
        self.path_pub = self.create_publisher(Marker, "/leica/path_marker", 10)
        self.imu_marker_pub = self.create_publisher(MarkerArray, "/imu0/vector_markers", 10)

        self.path_points: list[Point] = []

        self.create_subscription(
            PointStamped,
            "/leica/position",
            self.on_leica_position,
            qos_profile_sensor_data,
        )
        self.create_subscription(Imu, "/imu0", self.on_imu, qos_profile_sensor_data)

        self.publish_static_transforms()
        self.create_timer(2.0, self.publish_static_transforms)
        self.get_logger().info(
            "Publishing /leica/position_map, /leica/path_marker, and /imu0/vector_markers"
        )

    def publish_static_transforms(self) -> None:
        transforms = []
        for child in ("cam0", "imu4"):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.fixed_frame
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self.tf_broadcaster.sendTransform(transforms)

    def on_leica_position(self, msg: PointStamped) -> None:
        out = PointStamped()
        out.header = msg.header
        out.header.frame_id = self.fixed_frame
        out.point = msg.point
        self.point_pub.publish(out)

        self.path_points.append(msg.point)
        marker = Marker()
        marker.header = out.header
        marker.ns = "leica"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = self.path_points[-5000:]
        self.path_pub.publish(marker)

    def on_imu(self, msg: Imu) -> None:
        markers = MarkerArray()
        markers.markers.append(
            self.make_arrow(
                msg,
                "linear_acceleration",
                0,
                msg.linear_acceleration.x * 0.08,
                msg.linear_acceleration.y * 0.08,
                msg.linear_acceleration.z * 0.08,
                (1.0, 0.2, 0.1, 1.0),
            )
        )
        markers.markers.append(
            self.make_arrow(
                msg,
                "angular_velocity",
                1,
                msg.angular_velocity.x * 2.0,
                msg.angular_velocity.y * 2.0,
                msg.angular_velocity.z * 2.0,
                (0.2, 0.8, 1.0, 1.0),
            )
        )
        self.imu_marker_pub.publish(markers)

    @staticmethod
    def make_arrow(
        msg: Imu,
        namespace: str,
        marker_id: int,
        x: float,
        y: float,
        z: float,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header = msg.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.08
        marker.scale.z = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = Duration(sec=0, nanosec=250_000_000)
        marker.points = [Point(), Point(x=x, y=y, z=z)]

        length = math.sqrt(x * x + y * y + z * z)
        if length < 0.02:
            marker.points[1].x = 0.02
        return marker


def main() -> None:
    rclpy.init()
    node = EurocRvizHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
