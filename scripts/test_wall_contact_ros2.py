#!/usr/bin/env python3
"""
Wall contact test using ROS 2 control stack.

This script is intended to be used with the whole C++ control stack in `src/`:
  - core_drone_interface  (velocity / attitude / pose commands to the drone)
  - core_px4_interface    (PX4 + MAVROS bridge)
  - core_pose_controller  (position controller)
  - core_wrench_controller (force controller)

Goal:
  - Use a forward velocity command to gently approach a wall.
  - Detect contact using force/torque (FT) measurements.
  - After contact, command a constant desired force through the wrench controller
    and hold that contact for a configured time.

This node does NOT talk to PX4 directly. It only uses ROS 2 topics:
  - Publishes:
      * /velocity_command          (geometry_msgs/TwistStamped)
      * /ft_setpoint              (geometry_msgs/WrenchStamped)
      * /wrench_controller/switch (std_msgs/Bool)
  - Subscribes:
      * /ft_data_filtered or /ft_data (geometry_msgs/WrenchStamped)

Typical usage (example, adjust to your setup):
  1. Start PX4 SITL + Gazebo + MAVROS.
  2. Start ROS 2 control stack:
       - core_drone_interface  (with PX4Interface plugin)
       - core_pose_controller  (Gazebo params)
       - core_wrench_controller (Gazebo params)
       - FT bridge node from Gazebo to /ft_data or /ft_data_filtered
  3. In a ROS 2 environment:
       ros2 run <your_pkg> test_wall_contact_ros2.py

You may need to adapt topic names and frames according to your setup.
"""

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, WrenchStamped
from std_msgs.msg import Bool


class TestState(Enum):
  """Simple finite-state machine for the contact test."""

  APPROACH = 0
  HOLD_FORCE = 1
  DONE = 2


class WallContactTester(Node):
  """
  ROS 2 node that:
    - Commands a forward body velocity to approach a wall.
    - Monitors FT data to detect contact.
    - After contact, sends a constant wrench setpoint to the wrench controller
      and enables the motion–force control mixing via /wrench_controller/switch.
  """

  def __init__(self) -> None:
    super().__init__("wall_contact_tester")

    # Parameters (can be overridden via ROS 2 parameters)
    self.declare_parameter("force_topic", "ft_data_filtered")
    self.declare_parameter("force_threshold", 1.0)          # [N], contact detection threshold
    self.declare_parameter("desired_force", 5.0)            # [N], desired contact force magnitude
    self.declare_parameter("desired_force_axis", "x")       # "x", "y", or "z" in sensor frame
    self.declare_parameter("approach_velocity", 0.2)        # [m/s] forward velocity during approach
    self.declare_parameter("hold_time", 10.0)               # [s] hold time in force control mode
    self.declare_parameter("velocity_command_frame", "map") # frame_id for velocity_command
    self.declare_parameter("sensor_frame", "ft_sensor")     # frame_id of FT sensor for setpoint
    self.declare_parameter("loop_rate", 50.0)               # [Hz] main loop timer

    force_topic = self.get_parameter("force_topic").get_parameter_value().string_value
    self.force_threshold = (
      self.get_parameter("force_threshold").get_parameter_value().double_value
    )
    self.desired_force = (
      self.get_parameter("desired_force").get_parameter_value().double_value
    )
    self.desired_force_axis = (
      self.get_parameter("desired_force_axis").get_parameter_value().string_value
    ).lower()
    self.approach_velocity = (
      self.get_parameter("approach_velocity").get_parameter_value().double_value
    )
    self.hold_time = self.get_parameter("hold_time").get_parameter_value().double_value
    self.velocity_frame = (
      self.get_parameter("velocity_command_frame").get_parameter_value().string_value
    )
    self.sensor_frame = (
      self.get_parameter("sensor_frame").get_parameter_value().string_value
    )
    loop_rate = self.get_parameter("loop_rate").get_parameter_value().double_value

    # Publishers
    self.vel_pub = self.create_publisher(TwistStamped, "velocity_command", 10)
    self.ft_setpoint_pub = self.create_publisher(WrenchStamped, "ft_setpoint", 10)
    self.switch_pub = self.create_publisher(Bool, "wrench_controller/switch", 10)

    # Subscriber: FT data (filtered or raw)
    self.ft_sub = self.create_subscription(
      WrenchStamped,
      force_topic,
      self._ft_callback,
      10,
    )

    # Internal state
    self.state = TestState.APPROACH
    self.last_force_msg: Optional[WrenchStamped] = None
    self.last_force_norm: float = 0.0
    self.contact_time: Optional[Time] = None

    # Main loop timer
    period = 1.0 / loop_rate if loop_rate > 0.0 else 0.02
    self.timer = self.create_timer(period, self._loop)

    self.get_logger().info(
      f"WallContactTester started. Subscribing to '{force_topic}', "
      f"approach_velocity={self.approach_velocity:.3f} m/s, "
      f"force_threshold={self.force_threshold:.3f} N, "
      f"desired_force={self.desired_force:.3f} N along {self.desired_force_axis.upper()} axis."
    )

  # --------------------------------------------------------------------------- #
  # Callbacks
  # --------------------------------------------------------------------------- #

  def _ft_callback(self, msg: WrenchStamped) -> None:
    """Store the latest FT message and compute its force magnitude."""
    self.last_force_msg = msg
    fx = msg.wrench.force.x
    fy = msg.wrench.force.y
    fz = msg.wrench.force.z
    self.last_force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)

  # --------------------------------------------------------------------------- #
  # Main control loop
  # --------------------------------------------------------------------------- #

  def _loop(self) -> None:
    """Main FSM loop; runs at the configured loop_rate."""
    if self.state == TestState.APPROACH:
      self._step_approach()
    elif self.state == TestState.HOLD_FORCE:
      self._step_hold_force()
    elif self.state == TestState.DONE:
      # Keep publishing a safe zero command; node can be stopped by user.
      self._publish_velocity(0.0, 0.0, 0.0)
      return

  # --------------------------------------------------------------------------- #
  # FSM states
  # --------------------------------------------------------------------------- #

  def _step_approach(self) -> None:
    """Send a forward velocity command until contact is detected."""
    # Send forward velocity (in the velocity_frame; usually "map" or "world").
    self._publish_velocity(self.approach_velocity, 0.0, 0.0)

    # Only attempt contact detection if we have FT data.
    if self.last_force_msg is None:
      return

    if self.last_force_norm > self.force_threshold:
      self.get_logger().info(
        f"Contact detected. |F|={self.last_force_norm:.3f} N "
        f"(threshold={self.force_threshold:.3f} N)."
      )
      # Stop forward motion.
      self._publish_velocity(0.0, 0.0, 0.0)

      # Start force control: send desired wrench and enable switch.
      self._publish_wrench_setpoint(self.desired_force, self.desired_force_axis)
      self._set_wrench_switch(True)

      # Record contact time and transition to HOLD_FORCE.
      self.contact_time = self.get_clock().now()
      self.state = TestState.HOLD_FORCE

  def _step_hold_force(self) -> None:
    """Keep contact by holding a constant wrench setpoint for a fixed duration."""
    # Keep velocity at zero; thrust is modulated by the wrench controller.
    self._publish_velocity(0.0, 0.0, 0.0)

    # Re-publish wrench setpoint and keep switch enabled (robust to small losses).
    self._publish_wrench_setpoint(self.desired_force, self.desired_force_axis)
    self._set_wrench_switch(True)

    # Check hold time.
    if self.contact_time is None:
      # Should not happen, but safeguard.
      self.contact_time = self.get_clock().now()
      return

    elapsed = (self.get_clock().now() - self.contact_time).nanoseconds * 1e-9
    # Log current force from FT if available.
    if self.last_force_msg is not None:
      self.get_logger().info(
        f"Holding contact for {elapsed:.1f}/{self.hold_time:.1f}s, "
        f"|F|={self.last_force_norm:.3f} N"
      )

    if elapsed >= self.hold_time:
      self.get_logger().info(
        f"Force hold completed ({self.hold_time:.1f}s). Releasing wrench control."
      )
      # Disable wrench control and clear setpoint.
      self._set_wrench_switch(False)
      self._publish_wrench_setpoint(0.0, self.desired_force_axis)

      self.state = TestState.DONE

  # --------------------------------------------------------------------------- #
  # Helper publishers
  # --------------------------------------------------------------------------- #

  def _publish_velocity(self, vx: float, vy: float, vz: float) -> None:
    """Publish a velocity command (TwistStamped) to velocity_command."""
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.velocity_frame
    msg.twist.linear.x = float(vx)
    msg.twist.linear.y = float(vy)
    msg.twist.linear.z = float(vz)
    self.vel_pub.publish(msg)

  def _publish_wrench_setpoint(self, force_mag: float, axis: str) -> None:
    """Publish a wrench setpoint with a given force magnitude along one axis."""
    msg = WrenchStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = self.sensor_frame

    fx = fy = fz = 0.0
    if axis == "x":
      fx = force_mag
    elif axis == "y":
      fy = force_mag
    elif axis == "z":
      fz = force_mag
    else:
      # Fallback: use X-axis if an unknown axis is specified.
      fx = force_mag

    msg.wrench.force.x = fx
    msg.wrench.force.y = fy
    msg.wrench.force.z = fz
    # Torque setpoint is kept zero by default; extend if needed.
    self.ft_setpoint_pub.publish(msg)

  def _set_wrench_switch(self, enabled: bool) -> None:
    """Publish to wrench_controller/switch to enable/disable force-control mixing."""
    msg = Bool()
    msg.data = bool(enabled)
    self.switch_pub.publish(msg)


def main(args=None) -> None:
  rclpy.init(args=args)
  node = WallContactTester()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info("WallContactTester interrupted by user.")
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()

