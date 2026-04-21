#!/usr/bin/env python3
"""
Publish the extra ROS 2 topics needed by plot_thrust_live_ros2.py while using
the real odometry topic from this repository.

This helper:
  - publishes ft_setpoint.x = 5 N
  - keeps wrench_controller/switch = true by default
  - subscribes to real odometry on /mavros/local_position/odom
  - republishes each message on tracking_point (full copy: tracking = odometry)
  - publishes a lightweight synthetic attitude_thrust_command for plotting

When using an external force filter (ft_data -> ft_data_filtered), do NOT republish
force inside this helper. Let the force source publish ft_data, the filter publish
ft_data_filtered, and downstream nodes (plot/controller) subscribe to ft_data_filtered.

Typical usage:
  Terminal 1:
    python3 scripts/sim_ft_sensor.py

  Terminal 2:
    python3 scripts/test_plot_inputs_ros2.py

  Terminal 3:
    python3 scripts/plot_thrust_live_ros2.py
"""

from __future__ import annotations

import argparse
import copy
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, WrenchStamped
from mav_msgs.msg import AttitudeThrust
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force-setpoint-topic", default="ft_setpoint",
                        help="Desired force topic (default: ft_setpoint)")
    parser.add_argument("--switch-topic", default="wrench_controller/switch",
                        help="Mode switch topic (default: wrench_controller/switch)")
    parser.add_argument("--topic", default="attitude_thrust_command",
                        help="AttitudeThrust topic (default: attitude_thrust_command)")
    parser.add_argument("--odom-topic", default="mavros/local_position/odom",
                        help="Real odometry topic (default: mavros/local_position/odom)")
    parser.add_argument("--tracking-topic", default="tracking_point",
                        help="Tracking target topic (default: tracking_point)")
    parser.add_argument("--sensor-frame", default="ft_sensor",
                        help="Force sensor frame_id (default: ft_sensor)")
    parser.add_argument("--rate", type=float, default=40.0,
                        help="Publish rate in Hz (default: 40)")
    parser.add_argument("--desired-force", type=float, default=5.0,
                        help="Desired force setpoint in N (default: 5.0)")
    parser.add_argument("--pose-only", action="store_true",
                        help="Publish wrench_controller/switch=false instead of true")
    parser.add_argument("--hover-thrust", type=float, default=0.58,
                        help="Baseline z thrust for the fake command (default: 0.58)")
    parser.add_argument("--force-thrust-kp", type=float, default=0.07,
                        help="Gain from force error to thrust.x (default: 0.07)")
    parser.add_argument("--thrust-x-limit", type=float, default=0.40,
                        help="Clamp for thrust.x (default: 0.40)")
    parser.add_argument("--thrust-z-min", type=float, default=0.35,
                        help="Minimum thrust.z (default: 0.35)")
    parser.add_argument("--thrust-z-max", type=float, default=0.80,
                        help="Maximum thrust.z (default: 0.80)")
    parser.add_argument("--fallback-force-amp", type=float, default=0.25,
                        help="Fallback force amplitude if no input force exists (default: 0.25)")
    parser.add_argument("--fallback-force-freq", type=float, default=0.55,
                        help="Fallback force frequency in Hz (default: 0.55)")
    return parser.parse_args()


class PlotInputFeeder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("plot_input_feeder")
        self._args = args
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        self._latest_force_x = args.desired_force
        self._latest_odom: Optional[Odometry] = None

        self.create_subscription(
            Odometry, args.odom_topic, self._odom_cb, 10)

        self._setpoint_pub = self.create_publisher(
            WrenchStamped, args.force_setpoint_topic, 10)
        self._switch_pub = self.create_publisher(Bool, args.switch_topic, 10)
        self._tracking_pub = self.create_publisher(Odometry, args.tracking_topic, 10)
        self._thrust_pub = self.create_publisher(AttitudeThrust, args.topic, 10)

        period = 1.0 / max(args.rate, 1e-3)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            "PlotInputFeeder publishing: "
            f"setpoint='{args.force_setpoint_topic}', "
            f"switch='{args.switch_topic}', "
            f"tracking='{args.tracking_topic}', "
            f"thrust='{args.topic}'. "
            f"real odom='{args.odom_topic}'."
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg
        # tracking_point := full copy of this odometry (same pose/twist/covariance/stamp).
        self._publish_tracking_from_msg(msg)

    def _active_force_x(self, t: float) -> float:
        return (
            self._args.desired_force +
            self._args.fallback_force_amp *
            math.sin(2.0 * math.pi * self._args.fallback_force_freq * t)
        )

    def _publish_setpoint(self) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._args.sensor_frame
        msg.wrench.force.x = float(self._args.desired_force)
        self._setpoint_pub.publish(msg)

    def _publish_switch(self) -> None:
        msg = Bool()
        msg.data = not self._args.pose_only
        self._switch_pub.publish(msg)

    def _publish_tracking_from_msg(self, src: Odometry) -> None:
        self._tracking_pub.publish(copy.deepcopy(src))

    def _publish_thrust(self, force_x: float) -> None:
        msg = AttitudeThrust()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._latest_odom is not None and self._latest_odom.header.frame_id:
            msg.header.frame_id = self._latest_odom.header.frame_id
        else:
            msg.header.frame_id = "map"

        force_err = self._args.desired_force - force_x
        tx = _clamp(
            self._args.force_thrust_kp * force_err,
            -self._args.thrust_x_limit,
            self._args.thrust_x_limit,
        )
        tz = _clamp(
            self._args.hover_thrust,
            self._args.thrust_z_min,
            self._args.thrust_z_max,
        )
        pitch = _clamp(-0.60 * tx, -0.25, 0.25)
        msg.attitude = _quat_from_rpy(0.0, pitch, 0.0)
        msg.thrust.x = tx
        msg.thrust.y = 0.0
        msg.thrust.z = tz
        self._thrust_pub.publish(msg)

    def _tick(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        force_x = self._active_force_x(t)

        self._publish_setpoint()
        self._publish_switch()
        self._publish_thrust(force_x)

        if self._latest_odom is None:
            self.get_logger().info(
                f"Waiting for real odometry on '{self._args.odom_topic}'...",
                throttle_duration_sec=2.0)


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node: Optional[PlotInputFeeder] = None
    try:
        node = PlotInputFeeder(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
