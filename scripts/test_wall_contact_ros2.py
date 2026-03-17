#!/usr/bin/env python3
"""
Wall contact test using ROS 2 + PX4 SITL + MAVROS.

This script arms the drone, enters OFFBOARD mode, and uses MAVROS velocity
setpoints to approach a wall.  After contact is detected via FT data, it
switches the wrench controller to force-control mode and holds a constant
desired force for a configurable duration.

Required running processes:
  1. PX4 SITL (make px4_sitl gz_hexa_scorpion)
  2. MAVROS   (ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580)
  3. gz_ft_bridge.py  (bridges Gazebo FT sensor -> /ft_data)
  4. core_wrench_controller  (filters /ft_data -> /ft_data_filtered, force control)
  5. core_pose_controller    (optional, for wrench controller dependency)
  6. core_drone_interface    (with PX4Interface plugin)

Usage:
  python3 scripts/test_wall_contact_ros2.py
"""

import math
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped, WrenchStamped
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class TestState(Enum):
    PREFLIGHT = auto()
    TAKEOFF = auto()
    APPROACH = auto()
    HOLD_FORCE = auto()
    DONE = auto()


class WallContactTester(Node):

    def __init__(self) -> None:
        super().__init__("wall_contact_tester")

        # --------------- parameters --------------- #
        self.declare_parameter("force_topic", "ft_data_filtered")
        self.declare_parameter("force_threshold", 1.0)
        self.declare_parameter("desired_force", 5.0)
        self.declare_parameter("desired_force_axis", "x")
        self.declare_parameter("approach_velocity", 0.2)
        self.declare_parameter("hold_time", 10.0)
        self.declare_parameter("sensor_frame", "ft_sensor")
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("takeoff_velocity", 0.5)
        self.declare_parameter("loop_rate", 50.0)

        force_topic = self.get_parameter("force_topic").value
        self.force_threshold = self.get_parameter("force_threshold").value
        self.desired_force = self.get_parameter("desired_force").value
        self.desired_force_axis = self.get_parameter("desired_force_axis").value.lower()
        self.approach_velocity = self.get_parameter("approach_velocity").value
        self.hold_time = self.get_parameter("hold_time").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.takeoff_alt = self.get_parameter("takeoff_altitude").value
        self.takeoff_vel = self.get_parameter("takeoff_velocity").value
        loop_rate = self.get_parameter("loop_rate").value

        # --------------- MAVROS publishers / subscribers --------------- #
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.mavros_state: Optional[MavrosState] = None
        self.state_sub = self.create_subscription(
            MavrosState, "mavros/state", self._mavros_state_cb, state_qos)

        self.vel_pub = self.create_publisher(
            TwistStamped, "mavros/setpoint_velocity/cmd_vel", 10)

        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        # --------------- wrench controller publishers --------------- #
        self.ft_setpoint_pub = self.create_publisher(WrenchStamped, "ft_setpoint", 10)
        self.switch_pub = self.create_publisher(Bool, "wrench_controller/switch", 10)
        self.tracking_point_pub = self.create_publisher(Odometry, "tracking_point", 10)

        # --------------- odometry subscriber (for altitude & position) --------------- #
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.current_alt: float = 0.0
        self.last_odom: Optional[Odometry] = None
        self.odom_sub = self.create_subscription(
            Odometry, "mavros/local_position/odom", self._odom_callback, odom_qos)

        # --------------- FT subscriber --------------- #
        self.ft_sub = self.create_subscription(
            WrenchStamped, force_topic, self._ft_callback, 10)

        # --------------- internal state --------------- #
        self.state = TestState.PREFLIGHT
        self.last_force_msg: Optional[WrenchStamped] = None
        self.last_force_norm: float = 0.0
        self.contact_time: Optional[Time] = None
        self.hold_odom: Optional[Odometry] = None  # snapshot of odom at contact
        self._arm_requested = False
        self._offboard_requested = False
        self._preflight_log_timer = 0

        # --------------- main loop --------------- #
        period = 1.0 / loop_rate if loop_rate > 0 else 0.02
        self.timer = self.create_timer(period, self._loop)

        self.get_logger().info(
            f"WallContactTester started. force_topic='{force_topic}', "
            f"approach_vel={self.approach_velocity:.2f} m/s, "
            f"threshold={self.force_threshold:.1f} N, "
            f"desired_force={self.desired_force:.1f} N ({self.desired_force_axis.upper()})."
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _mavros_state_cb(self, msg: MavrosState) -> None:
        self.mavros_state = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_alt = msg.pose.pose.position.z
        self.last_odom = msg

    def _ft_callback(self, msg: WrenchStamped) -> None:
        self.last_force_msg = msg
        f = msg.wrench.force
        self.last_force_norm = math.sqrt(f.x**2 + f.y**2 + f.z**2)

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def _loop(self) -> None:
        if self.state == TestState.PREFLIGHT:
            self._step_preflight()
        elif self.state == TestState.TAKEOFF:
            self._step_takeoff()
        elif self.state == TestState.APPROACH:
            self._step_approach()
        elif self.state == TestState.HOLD_FORCE:
            self._step_hold_force()
        elif self.state == TestState.DONE:
            self._publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info("Test done. Hovering. Ctrl+C to exit.",
                                   throttle_duration_sec=5.0)

    # ------------------------------------------------------------------ #
    # PREFLIGHT: wait for MAVROS connection, arm, OFFBOARD
    # ------------------------------------------------------------------ #

    def _step_preflight(self) -> None:
        # PX4 OFFBOARD requires setpoint stream before mode switch
        self._publish_velocity(0.0, 0.0, 0.0)

        if self.mavros_state is None:
            self._preflight_log_timer += 1
            if self._preflight_log_timer % 100 == 1:
                self.get_logger().info("Waiting for MAVROS state...")
            return

        if not self.mavros_state.connected:
            self._preflight_log_timer += 1
            if self._preflight_log_timer % 100 == 1:
                self.get_logger().info("Waiting for FCU connection...")
            return

        # Request OFFBOARD mode (must send setpoints first)
        if self.mavros_state.mode != "OFFBOARD" and not self._offboard_requested:
            self.get_logger().info("Requesting OFFBOARD mode...")
            if self.set_mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                self.set_mode_client.call_async(req)
                self._offboard_requested = True
            return

        if self.mavros_state.mode != "OFFBOARD":
            self._preflight_log_timer += 1
            if self._preflight_log_timer % 100 == 1:
                self.get_logger().info(
                    f"Waiting for OFFBOARD (current mode: {self.mavros_state.mode})...")
            # Re-request periodically
            if self._preflight_log_timer % 200 == 0:
                self._offboard_requested = False
            return

        # Arm
        if not self.mavros_state.armed and not self._arm_requested:
            self.get_logger().info("Arming drone...")
            if self.arming_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self._arm_requested = True
            return

        if not self.mavros_state.armed:
            self._preflight_log_timer += 1
            if self._preflight_log_timer % 100 == 1:
                self.get_logger().info("Waiting for arm confirmation...")
            if self._preflight_log_timer % 200 == 0:
                self._arm_requested = False
            return

        self.get_logger().info(
            f"Drone armed and in OFFBOARD mode. Taking off to {self.takeoff_alt:.1f}m...")
        self.state = TestState.TAKEOFF

    # ------------------------------------------------------------------ #
    # TAKEOFF: climb to target altitude
    # ------------------------------------------------------------------ #

    def _step_takeoff(self) -> None:
        if not self.mavros_state or not self.mavros_state.armed:
            self.get_logger().warn("Lost arm during takeoff, re-arming...")
            self._arm_requested = False
            self.state = TestState.PREFLIGHT
            return

        self._publish_velocity(0.0, 0.0, self.takeoff_vel)

        if self.current_alt >= self.takeoff_alt * 0.95:
            self.get_logger().info(
                f"Reached altitude {self.current_alt:.2f}m (target {self.takeoff_alt:.1f}m). "
                "Starting approach.")
            self.state = TestState.APPROACH
        else:
            self.get_logger().info(
                f"Taking off... alt={self.current_alt:.2f}/{self.takeoff_alt:.1f}m",
                throttle_duration_sec=1.0,
            )

    # ------------------------------------------------------------------ #
    # APPROACH: fly forward until contact
    # ------------------------------------------------------------------ #

    def _step_approach(self) -> None:
        self._publish_velocity(self.approach_velocity, 0.0, 0.0)

        if self.last_force_msg is None:
            return

        if self.last_force_norm > self.force_threshold:
            self.get_logger().info(
                f"Contact detected! |F|={self.last_force_norm:.3f} N "
                f"(threshold={self.force_threshold:.1f} N)")
            # Snapshot current pose as the hold position for pose controller
            self.hold_odom = self.last_odom
            self._publish_wrench_setpoint(self.desired_force, self.desired_force_axis)
            self._set_wrench_switch(True)
            self.contact_time = self.get_clock().now()
            self.get_logger().info(
                "Switched to force control. Wrench controller now commands attitude/thrust.")
            self.state = TestState.HOLD_FORCE

    # ------------------------------------------------------------------ #
    # HOLD_FORCE: maintain desired contact force
    # ------------------------------------------------------------------ #

    def _step_hold_force(self) -> None:
        # Publish tracking_point so the wrench controller's internal pose
        # controller has a valid target.  Use the snapshot taken at contact.
        self._publish_tracking_point()

        # Keep re-sending the wrench setpoint and switch.
        self._publish_wrench_setpoint(self.desired_force, self.desired_force_axis)
        self._set_wrench_switch(True)

        if self.contact_time is None:
            self.contact_time = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() - self.contact_time).nanoseconds * 1e-9
        if self.last_force_msg is not None:
            self.get_logger().info(
                f"Hold {elapsed:.1f}/{self.hold_time:.1f}s  |F|={self.last_force_norm:.3f} N",
                throttle_duration_sec=1.0,
            )

        if elapsed >= self.hold_time:
            self.get_logger().info(
                f"Force hold completed ({self.hold_time:.1f}s). Releasing.")
            self._set_wrench_switch(False)
            self._publish_wrench_setpoint(0.0, self.desired_force_axis)
            self.state = TestState.DONE

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _publish_tracking_point(self) -> None:
        """Publish the hold-position as a tracking_point for the wrench controller's
        internal pose controller.  Also keeps PX4 OFFBOARD alive via the attitude
        setpoint stream from wrench_controller → drone_interface → MAVROS."""
        if self.hold_odom is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.hold_odom.header.frame_id
        msg.child_frame_id = self.hold_odom.child_frame_id
        msg.pose = self.hold_odom.pose
        # Zero velocity target (hold position)
        self.tracking_point_pub.publish(msg)

    def _publish_velocity(self, vx: float, vy: float, vz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def _publish_wrench_setpoint(self, force_mag: float, axis: str) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_frame
        if axis == "x":
            msg.wrench.force.x = force_mag
        elif axis == "y":
            msg.wrench.force.y = force_mag
        elif axis == "z":
            msg.wrench.force.z = force_mag
        else:
            msg.wrench.force.x = force_mag
        self.ft_setpoint_pub.publish(msg)

    def _set_wrench_switch(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.switch_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallContactTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
