#!/usr/bin/env python3
# Custom tracking point sender for UAV position targets
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class TrackingPointSender(Node):
    def __init__(self):
        super().__init__("tracking_point_sender")

        self.declare_parameter("target_frame", "/uav1/map")
        self.declare_parameter("cmd_topic", "/uav1/user/target_point")
        self.declare_parameter("tracking_point_topic", "/uav1/tracking_point")
        self.declare_parameter("publish_rate_hz", 50.0)

        self.frame_id = self.get_parameter("target_frame").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        out_topic = (
            self.get_parameter("tracking_point_topic").get_parameter_value().string_value
        )
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        if rate_hz <= 0.0:
            rate_hz = 50.0

        self._current_target = None
        self._lock = threading.Lock()

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub_cmd = self.create_subscription(Point, cmd_topic, self._cmd_cb, 1)

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"[tracking_point_sender] Publishing to {out_topic} at {rate_hz} Hz. "
            f"Listening on {cmd_topic}. frame_id='{self.frame_id}'"
        )

    def _cmd_cb(self, msg: Point):
        with self._lock:
            self._current_target = (float(msg.x), float(msg.y), float(msg.z))
            xyz = self._current_target
        self.get_logger().info(f"[tracking_point_sender] New target: {xyz}")
        self._publish_xyz(xyz)

    def _on_timer(self):
        with self._lock:
            xyz = self._current_target
        if xyz is not None:
            self._publish_xyz(xyz)

    def _publish_xyz(self, xyz):
        x, y, z = xyz
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingPointSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
