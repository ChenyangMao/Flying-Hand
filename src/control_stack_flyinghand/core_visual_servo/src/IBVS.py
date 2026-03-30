#!/usr/bin/env python3
# pure control node
import time
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TwistStamped, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class VisualServo(Node):
    def __init__(self, verbose=False):
        super().__init__("visual_servo_controller")
        self.verbose = verbose
        self.count = 0
        self._vel = (0.0, 0.0, 0.0)
        self._last_circle_log_t = 0.0
        self._last_verbose_cmd_log_t = 0.0
        self._stationary_warned = False

        self.declare_parameter("camera.fx", 349.21872)
        self.declare_parameter("camera.fy", 349.21872)
        self.declare_parameter("camera.cx", 320.0)
        self.declare_parameter("camera.cy", 180.0)
        self.declare_parameter("target_center_y", 120.0)
        self.declare_parameter("target_center_y_final", 70.0)
        self.declare_parameter("target_x_bias", 0.0)
        self.declare_parameter("target_radius_pixel", 100.0)
        self.declare_parameter("target_radius_pixel_final", 115.0)
        self.declare_parameter("target_radius", 0.15)

        self.fx = float(self.get_parameter("camera.fx").value)
        self.fy = float(self.get_parameter("camera.fy").value)
        self.cx = float(self.get_parameter("camera.cx").value)
        self.cy = float(self.get_parameter("camera.cy").value)
        self.K = np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]]
        )

        target_center_y = float(self.get_parameter("target_center_y").value)
        self.target_center_y_final = float(
            self.get_parameter("target_center_y_final").value
        )
        self.target_x_bias = float(self.get_parameter("target_x_bias").value)
        self.get_logger().info(f"target_x_bias: {self.target_x_bias}")
        self.target_center = (self.cx + self.target_x_bias, target_center_y)
        self.target_radius_pixel = float(
            self.get_parameter("target_radius_pixel").value
        )
        self.target_radius_pixel_final = float(
            self.get_parameter("target_radius_pixel_final").value
        )
        self.rd = self.target_radius_pixel
        self.target_radius = float(self.get_parameter("target_radius").value)

        self.lambda_gain = 3.0
        self.forward_gain = 0.003

        self.circle_sub = self.create_subscription(
            Vector3, "/detected_circle", self.circle_cb, 10
        )
        self.vehicle_velocity_sub = self.create_subscription(
            Odometry, "/uav1/odometry", self.vehicle_velocity_cb, 10
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "/visual_servo/cmd_vel", 1)
        self.visual_depth_pub = self.create_publisher(
            Vector3Stamped, "/visual_servo/depth", 1
        )
        self.visual_servo_active = self.create_publisher(
            Bool, "/visual_servo/active", 1
        )

    def _throttle_info(self, last_attr: str, period_s: float, text: str) -> None:
        now = time.monotonic()
        last = getattr(self, last_attr)
        if now - last >= period_s:
            setattr(self, last_attr, now)
            self.get_logger().info(text)

    def circle_cb(self, msg: Vector3):
        self._throttle_info(
            "_last_circle_log_t",
            1.0,
            f"Circle detected at: ({msg.x}, {msg.y}) with radius {msg.z}",
        )

        u = float(msg.x)
        v = float(msg.y)
        r = float(msg.z)

        if r <= 0.0:
            return

        depth = (self.fx * self.target_radius) / r

        depth_msg = Vector3Stamped()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.vector.x = 0.0
        depth_msg.vector.y = 0.0
        depth_msg.vector.z = float(depth)
        self.visual_depth_pub.publish(depth_msg)

        L = np.array(
            [
                [
                    -self.fx / depth,
                    0,
                    u / depth,
                    u * v / self.fx,
                    -(self.fx**2 + u**2) / self.fx,
                    v,
                ],
                [
                    0,
                    -self.fy / depth,
                    v / depth,
                    (self.fx**2 + v**2) / self.fy,
                    -u * v / self.fy,
                    -u,
                ],
            ]
        )

        error = np.array(
            [(u - self.target_center[0]), (v - self.target_center[1])]
        )
        v_camera = -self.lambda_gain * (np.linalg.pinv(L) @ error)
        error_r = r - self.rd
        v_camera_z = -self.forward_gain * error_r

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = float(v_camera[0])
        twist.twist.linear.y = float(v_camera[1])
        twist.twist.linear.z = float(v_camera_z)
        self.cmd_pub.publish(twist)
        self.visual_servo_active.publish(Bool(data=True))

        if self.verbose:
            self._throttle_info(
                "_last_verbose_cmd_log_t",
                1.0,
                f"v_camera={v_camera}, cmd linear=({twist.twist.linear.x}, "
                f"{twist.twist.linear.y}, {twist.twist.linear.z})",
            )

    def vehicle_velocity_cb(self, msg: Odometry):
        lin = msg.twist.twist.linear
        self._vel = (float(lin.x), float(lin.y), float(lin.z))
        if np.linalg.norm(self._vel) < 0.01:
            self.count += 1
            if self.count > 1000 and not self._stationary_warned:
                self._stationary_warned = True
                self.get_logger().warning(
                    "Vehicle is stationary, changing target center."
                )
                self.target_center = (
                    self.cx + self.target_x_bias,
                    self.target_center_y_final,
                )
                self.rd = self.target_radius_pixel_final
                self.lambda_gain = 6.0


def main(args=None):
    rclpy.init(args=args)
    node = VisualServo(verbose=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
