#!/usr/bin/env python3
# pure control node
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self._target_active = None
        self._last_circle_t = None

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
        self.declare_parameter("odometry_topic", "/uav1/odometry")
        self.declare_parameter("lost_target_timeout_sec", 0.5)
        self.declare_parameter("lambda_gain", 2.0)
        self.declare_parameter("forward_gain", 0.0005)
        self.declare_parameter("max_cmd_camera_xy", 0.03)
        self.declare_parameter("max_cmd_camera_z", 0.06)
        self.declare_parameter("detection_ema_alpha", 0.4)
        self.declare_parameter("depth_ema_alpha", 0.3)
        self.declare_parameter("max_cmd_rate_xy", 0.02)
        self.declare_parameter("max_cmd_rate_z", 0.03)
        self.declare_parameter("stationary_retarget_enable", False)
        self.declare_parameter("stationary_speed_threshold", 0.01)
        self.declare_parameter("stationary_count_threshold", 1000)

        self.fx = float(self.get_parameter("camera.fx").value)
        self.fy = float(self.get_parameter("camera.fy").value)
        self.cx = float(self.get_parameter("camera.cx").value)
        self.cy = float(self.get_parameter("camera.cy").value)
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.lost_target_timeout_sec = float(
            self.get_parameter("lost_target_timeout_sec").value
        )
        self.lambda_gain = float(self.get_parameter("lambda_gain").value)
        self.forward_gain = float(self.get_parameter("forward_gain").value)
        self.max_cmd_camera_xy = float(self.get_parameter("max_cmd_camera_xy").value)
        self.max_cmd_camera_z = float(self.get_parameter("max_cmd_camera_z").value)
        self.detection_ema_alpha = float(self.get_parameter("detection_ema_alpha").value)
        self.depth_ema_alpha = float(self.get_parameter("depth_ema_alpha").value)
        self.max_cmd_rate_xy = float(self.get_parameter("max_cmd_rate_xy").value)
        self.max_cmd_rate_z = float(self.get_parameter("max_cmd_rate_z").value)
        self.stationary_retarget_enable = bool(
            self.get_parameter("stationary_retarget_enable").value
        )
        self.stationary_speed_threshold = float(
            self.get_parameter("stationary_speed_threshold").value
        )
        self.stationary_count_threshold = int(
            self.get_parameter("stationary_count_threshold").value
        )
        self.K = np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]]
        )

        # EMA state for detection smoothing
        self._ema_u = None
        self._ema_v = None
        self._ema_r = None
        self._ema_depth = None
        # Previous command for rate limiting
        self._prev_cmd_x = 0.0
        self._prev_cmd_y = 0.0
        self._prev_cmd_z = 0.0

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
        self.get_logger().info(
            f"IBVS gains: lambda={self.lambda_gain:.3f} "
            f"forward_gain={self.forward_gain:.5f} "
            f"max_cmd_xy={self.max_cmd_camera_xy:.3f} "
            f"max_cmd_z={self.max_cmd_camera_z:.3f}"
        )
        self.get_logger().info(
            f"stationary_retarget_enable={self.stationary_retarget_enable} "
            f"threshold={self.stationary_speed_threshold:.3f} "
            f"count={self.stationary_count_threshold}"
        )

        self.circle_sub = self.create_subscription(
            Vector3, "/detected_circle", self.circle_cb, 10
        )
        self.vehicle_velocity_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.vehicle_velocity_cb,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "/visual_servo/cmd_vel", 1)
        self.visual_depth_pub = self.create_publisher(
            Vector3Stamped, "/visual_servo/depth", 1
        )
        self.visual_servo_active = self.create_publisher(
            Bool, "/visual_servo/active", 1
        )
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)
        self._publish_active(False)

    def _throttle_info(self, last_attr: str, period_s: float, text: str) -> None:
        now = time.monotonic()
        last = getattr(self, last_attr)
        if now - last >= period_s:
            setattr(self, last_attr, now)
            self.get_logger().info(text)

    def _ema(self, prev, new, alpha):
        """Exponential moving average helper."""
        if prev is None:
            return new
        return prev * (1.0 - alpha) + new * alpha

    def _rate_limit(self, prev, target, max_delta):
        """Clamp the change per step to max_delta."""
        delta = target - prev
        delta = np.clip(delta, -max_delta, max_delta)
        return prev + delta

    def circle_cb(self, msg: Vector3):
        self._throttle_info(
            "_last_circle_log_t",
            1.0,
            f"Circle detected at: ({msg.x}, {msg.y}) with radius {msg.z}",
        )

        u_raw = float(msg.x)
        v_raw = float(msg.y)
        r_raw = float(msg.z)

        if r_raw <= 0.0:
            return

        self._last_circle_t = self.get_clock().now()
        self._publish_active(True, force=True)

        # EMA smoothing on detected circle coordinates
        alpha = self.detection_ema_alpha
        self._ema_u = self._ema(self._ema_u, u_raw, alpha)
        self._ema_v = self._ema(self._ema_v, v_raw, alpha)
        self._ema_r = self._ema(self._ema_r, r_raw, alpha)
        u = self._ema_u
        v = self._ema_v
        r = self._ema_r

        depth_raw = (self.fx * self.target_radius) / r
        self._ema_depth = self._ema(self._ema_depth, depth_raw, self.depth_ema_alpha)
        depth = self._ema_depth

        depth_msg = Vector3Stamped()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "camera"
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
        v_camera[0] = np.clip(v_camera[0], -self.max_cmd_camera_xy, self.max_cmd_camera_xy)
        v_camera[1] = np.clip(v_camera[1], -self.max_cmd_camera_xy, self.max_cmd_camera_xy)
        error_r = r - self.rd
        v_camera_z = np.clip(
            -self.forward_gain * error_r,
            -self.max_cmd_camera_z,
            self.max_cmd_camera_z,
        )

        # Rate-limit the velocity commands to prevent sudden spikes
        cmd_x = self._rate_limit(self._prev_cmd_x, float(v_camera[0]), self.max_cmd_rate_xy)
        cmd_y = self._rate_limit(self._prev_cmd_y, float(v_camera[1]), self.max_cmd_rate_xy)
        cmd_z = self._rate_limit(self._prev_cmd_z, float(v_camera_z), self.max_cmd_rate_z)
        self._prev_cmd_x = cmd_x
        self._prev_cmd_y = cmd_y
        self._prev_cmd_z = cmd_z

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "camera"
        twist.twist.linear.x = cmd_x
        twist.twist.linear.y = cmd_y
        twist.twist.linear.z = cmd_z
        self.cmd_pub.publish(twist)

        if self.verbose:
            self._throttle_info(
                "_last_verbose_cmd_log_t",
                1.0,
                f"v_camera={v_camera}, cmd linear=({cmd_x}, {cmd_y}, {cmd_z})",
            )

    def vehicle_velocity_cb(self, msg: Odometry):
        lin = msg.twist.twist.linear
        self._vel = (float(lin.x), float(lin.y), float(lin.z))
        if not self.stationary_retarget_enable:
            return

        if np.linalg.norm(self._vel) < self.stationary_speed_threshold:
            self.count += 1
            if self.count > self.stationary_count_threshold and not self._stationary_warned:
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
        else:
            self.count = 0

    def watchdog_cb(self):
        if self._last_circle_t is None:
            return

        age = (self.get_clock().now() - self._last_circle_t).nanoseconds * 1e-9
        if age > self.lost_target_timeout_sec:
            self._publish_active(False)
            # Reset EMA state so a new lock starts fresh
            self._ema_u = None
            self._ema_v = None
            self._ema_r = None
            self._ema_depth = None
            self._prev_cmd_x = 0.0
            self._prev_cmd_y = 0.0
            self._prev_cmd_z = 0.0

    def _publish_active(self, is_active: bool, force: bool = False):
        if not force and self._target_active == is_active:
            return
        self._target_active = is_active
        self.visual_servo_active.publish(Bool(data=is_active))


def main(args=None):
    rclpy.init(args=args)
    node = VisualServo(verbose=False)
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
