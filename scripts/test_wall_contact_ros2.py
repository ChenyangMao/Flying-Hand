#!/usr/bin/env python3
"""
Wall contact test using ROS 2 + PX4 SITL + MAVROS.

Arms the drone, takes off, approaches a wall via velocity setpoints,
then switches the wrench controller to force-control mode.
After the contact force (sensor Fx in ft_sensor frame)
to world-X wall-normal after the TF chain) stays within the desired band
continuously for ``hold_time`` seconds, the drone retreats, descends, and
disarms.

Required running processes:
  1. PX4 SITL        (make px4_sitl gz_hexa_scorpion)
  2. MAVROS           (ros2 launch mavros px4.launch ...)
  3. gz_ft_bridge.py  (Gazebo FT sensor -> /ft_data)
  4. core_wrench_controller  (/ft_data -> /ft_data_filtered, force control)
  5. core_drone_interface    (PX4Interface plugin)

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

from geometry_msgs.msg import TransformStamped, TwistStamped, WrenchStamped
from mav_msgs.msg import AttitudeThrust
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class TestState(Enum):
    PREFLIGHT = auto()
    TAKEOFF = auto()
    HOVER_ALT = auto()
    APPROACH = auto()
    HOLD_FORCE = auto()
    RETREAT = auto()
    LAND = auto()
    IDLE = auto()


class WallContactTester(Node):

    def __init__(self) -> None:
        super().__init__("wall_contact_tester")

        # --------------- parameters --------------- #
        self.declare_parameter("force_topic", "ft_data_filtered")
        self.declare_parameter("force_threshold", 0.2)
        self.declare_parameter("desired_force", 1.5)
        self.declare_parameter("approach_velocity", 0.1)
        self.declare_parameter("hold_time", 10.0)
        self.declare_parameter("force_hold_tolerance", 0.2)
        self.declare_parameter("max_hold_timeout", 90.0)
        self.declare_parameter("sensor_frame", "ft_sensor")
        self.declare_parameter("takeoff_altitude", 1.5)
        self.declare_parameter("takeoff_velocity", 0.5)
        self.declare_parameter("pre_approach_hold_sec", 2.0)
        self.declare_parameter("approach_altitude_kp", 0.8)
        self.declare_parameter("approach_vz_limit", 0.35)
        self.declare_parameter("loop_rate", 50.0)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("map_ned_frame_id", "map_ned")
        self.declare_parameter("base_link_frame_id", "base_link")
        self.declare_parameter("frd_frame_id", "base_link_frd")
        self.declare_parameter("sensor_offset_z", -0.3)
        self.declare_parameter("hold_x_offset", 0.3)
        self.declare_parameter("retreat_velocity", 0.08)
        self.declare_parameter("retreat_duration_sec", 4.0)
        self.declare_parameter("land_velocity", 0.25)
        self.declare_parameter("land_alt_threshold", 0.12)

        force_topic = self.get_parameter("force_topic").value
        self.force_threshold = float(self.get_parameter("force_threshold").value)
        self.desired_force = float(self.get_parameter("desired_force").value)
        self.approach_velocity = self.get_parameter("approach_velocity").value
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.force_hold_tolerance = float(self.get_parameter("force_hold_tolerance").value)
        self.max_hold_timeout = float(self.get_parameter("max_hold_timeout").value)
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.takeoff_alt = self.get_parameter("takeoff_altitude").value
        self.takeoff_vel = self.get_parameter("takeoff_velocity").value
        self.pre_approach_hold_sec = float(self.get_parameter("pre_approach_hold_sec").value)
        self.approach_altitude_kp = float(self.get_parameter("approach_altitude_kp").value)
        self.approach_vz_limit = float(self.get_parameter("approach_vz_limit").value)
        loop_rate = self.get_parameter("loop_rate").value
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.map_ned_frame_id = self.get_parameter("map_ned_frame_id").value
        self.base_link_frame_id = self.get_parameter("base_link_frame_id").value
        self.frd_frame_id = self.get_parameter("frd_frame_id").value
        self.sensor_offset_z = float(self.get_parameter("sensor_offset_z").value)
        self.hold_x_offset = float(self.get_parameter("hold_x_offset").value)
        self.retreat_velocity = float(self.get_parameter("retreat_velocity").value)
        self.retreat_duration_sec = float(self.get_parameter("retreat_duration_sec").value)
        self.land_velocity = float(self.get_parameter("land_velocity").value)
        self.land_alt_threshold = float(self.get_parameter("land_alt_threshold").value)

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

        # --------------- odometry subscriber --------------- #
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.current_alt: float = 0.0
        self.last_odom: Optional[Odometry] = None
        self.odom_sub = self.create_subscription(
            Odometry, "mavros/local_position/odom", self._odom_callback, odom_qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        # --------------- FT subscriber --------------- #
        self.ft_sub = self.create_subscription(
            WrenchStamped, force_topic, self._ft_callback, 10)

        # --------------- attitude/thrust subscriber --------------- #
        self.last_attitude_thrust: Optional[AttitudeThrust] = None
        self.attitude_thrust_sub = self.create_subscription(
            AttitudeThrust, "attitude_thrust_command", self._attitude_thrust_cb, 10)

        # --------------- internal state --------------- #
        self.state = TestState.PREFLIGHT
        self.last_force_msg: Optional[WrenchStamped] = None
        self.last_fx: float = 0.0
        self.contact_time: Optional[Time] = None
        self._in_band_since: Optional[Time] = None
        self.hold_odom: Optional[Odometry] = None
        self._hover_t0: Optional[Time] = None
        self._arm_requested = False
        self._offboard_requested = False
        self._preflight_log_timer = 0
        self._retreat_t0: Optional[Time] = None
        self._retreat_pose_x0: float = 0.0
        self._land_t0: Optional[Time] = None
        self._land_z0: Optional[float] = None
        self._disarm_sent = False
        self._idle_logged = False

        # --------------- main loop --------------- #
        period = 1.0 / loop_rate if loop_rate > 0 else 0.02
        self.timer = self.create_timer(period, self._loop)

        self.get_logger().info(
            f"WallContactTester started. force_topic='{force_topic}', "
            f"approach_vel={self.approach_velocity:.2f} m/s, "
            f"contact_threshold={self.force_threshold:.2f} N, "
            f"desired_Fx={self.desired_force:.1f} N, "
            f"hold: {self.hold_time:.1f}s within "
            f"{self.desired_force:.1f}±{self.force_hold_tolerance:.2f} N, "
            f"hold_x_offset={self.hold_x_offset:.2f}m."
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
        self.last_fx = abs(float(msg.wrench.force.x))

    def _attitude_thrust_cb(self, msg: AttitudeThrust) -> None:
        self.last_attitude_thrust = msg

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def _loop(self) -> None:
        self._publish_map_to_base_link_tf()
        self._publish_map_to_ned_tf()
        self._publish_base_to_frd_tf()
        self._publish_base_to_sensor_tf()
        self._publish_map_to_contact_tf()

        handler = {
            TestState.PREFLIGHT: self._step_preflight,
            TestState.TAKEOFF: self._step_takeoff,
            TestState.HOVER_ALT: self._step_hover_alt,
            TestState.APPROACH: self._step_approach,
            TestState.HOLD_FORCE: self._step_hold_force,
            TestState.RETREAT: self._step_retreat,
            TestState.LAND: self._step_land,
            TestState.IDLE: self._step_idle,
        }.get(self.state)
        if handler:
            handler()

    # ------------------------------------------------------------------ #
    # PREFLIGHT
    # ------------------------------------------------------------------ #

    def _step_preflight(self) -> None:
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
            if self._preflight_log_timer % 200 == 0:
                self._offboard_requested = False
            return

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
    # TAKEOFF
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
                f"Hovering {self.pre_approach_hold_sec:.1f}s to stabilize before approach.")
            self.state = TestState.HOVER_ALT
            self._hover_t0 = self.get_clock().now()
        else:
            self.get_logger().info(
                f"Taking off... alt={self.current_alt:.2f}/{self.takeoff_alt:.1f}m",
                throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ #
    # HOVER_ALT
    # ------------------------------------------------------------------ #

    def _step_hover_alt(self) -> None:
        if not self.mavros_state or not self.mavros_state.armed:
            self.get_logger().warn("Lost arm during hover, re-arming...")
            self._arm_requested = False
            self.state = TestState.PREFLIGHT
            return

        if self._hover_t0 is None:
            self._hover_t0 = self.get_clock().now()

        vz = self._altitude_hold_vz(self.takeoff_alt)
        self._publish_velocity(0.0, 0.0, vz)

        elapsed = (self.get_clock().now() - self._hover_t0).nanoseconds * 1e-9
        self.get_logger().info(
            f"Pre-approach hover {elapsed:.1f}/{self.pre_approach_hold_sec:.1f}s  "
            f"alt={self.current_alt:.2f}m (target {self.takeoff_alt:.1f}m)",
            throttle_duration_sec=1.0)

        if elapsed >= self.pre_approach_hold_sec:
            self.get_logger().info(
                f"Hover done. Approaching wall at {self.approach_velocity:.2f} m/s.")
            self.state = TestState.APPROACH

    # ------------------------------------------------------------------ #
    # APPROACH
    # ------------------------------------------------------------------ #

    def _step_approach(self) -> None:
        vz = self._altitude_hold_vz(self.takeoff_alt)

        if self.last_force_msg is None:
            self._publish_velocity(self.approach_velocity, 0.0, vz)
            return

        if self.last_fx > self.force_threshold:
            # Same cycle as contact: full zero velocity setpoint. Horizontal zero avoids shoving
            # the wall; vertical zero avoids sending a vz altitude-hold command the same instant
            # we hand off to wrench attitude/thrust (reduces an upward kick in PX4 offboard).
            self._publish_velocity(0.0, 0.0, 0.0)
            self.get_logger().info(
                f"Contact detected! Fx={self.last_fx:.3f} N "
                f"(threshold={self.force_threshold:.2f} N)  "
                f"alt={self.current_alt:.2f}m")
            self.hold_odom = self.last_odom
            self._publish_wrench_setpoint()
            self._set_wrench_switch(True)
            self.contact_time = self.get_clock().now()
            self._in_band_since = None
            self.get_logger().info(
                "Switched to force control. Wrench controller now commands attitude/thrust.")
            self.state = TestState.HOLD_FORCE
            return

        self._publish_velocity(self.approach_velocity, 0.0, vz)

    # ------------------------------------------------------------------ #
    # HOLD_FORCE
    # ------------------------------------------------------------------ #

    def _step_hold_force(self) -> None:
        # Do NOT publish velocity to MAVROS during force control — the wrench_controller
        # publishes attitude+thrust via drone_interface_node -> mavros/setpoint_raw/attitude.
        # Sending velocity setpoints simultaneously causes PX4 to follow velocity instead.
        self._publish_tracking_point()
        self._publish_wrench_setpoint()
        self._set_wrench_switch(True)

        if self.contact_time is None:
            self.contact_time = self.get_clock().now()
            self._in_band_since = None

        now = self.get_clock().now()
        t_total = (now - self.contact_time).nanoseconds * 1e-9

        in_band = abs(self.last_fx - self.desired_force) <= self.force_hold_tolerance

        if in_band:
            if self._in_band_since is None:
                self._in_band_since = now
        else:
            self._in_band_since = None

        t_in_band = 0.0
        if self._in_band_since is not None:
            t_in_band = (now - self._in_band_since).nanoseconds * 1e-9

        log_msg = (
            f"Hold  t_in_band={t_in_band:.1f}/{self.hold_time:.1f}s  "
            f"t_fc={t_total:.1f}s  in_band={in_band}  "
            f"alt={self.current_alt:.2f}m  Fx={self.last_fx:.3f} N"
        )
        if self.hold_odom is not None and self.last_odom is not None:
            hp = self.hold_odom.pose.pose.position
            cp = self.last_odom.pose.pose.position
            tx = float(hp.x) + self.hold_x_offset
            ty = float(hp.y)
            tz = float(self.takeoff_alt)
            log_msg += (
                f"  target_pos=({tx:.3f},{ty:.3f},{tz:.3f})  "
                f"current_pos=({cp.x:.3f},{cp.y:.3f},{cp.z:.3f})"
            )
        if self.last_attitude_thrust is not None:
            t = self.last_attitude_thrust.thrust
            q = self.last_attitude_thrust.attitude
            log_msg += (
                f"  thrust=({t.x:.3f},{t.y:.3f},{t.z:.3f})"
                f"  att=(x={q.x:.3f},y={q.y:.3f},z={q.z:.3f},w={q.w:.3f})")
        self.get_logger().info(log_msg, throttle_duration_sec=1.0)

        if self._in_band_since is not None and t_in_band >= self.hold_time:
            self.get_logger().info(
                f"Force hold completed ({self.hold_time:.1f}s in band). Retreating.")
            self._finish_hold()
            return

        if self.max_hold_timeout > 0 and t_total >= self.max_hold_timeout:
            self.get_logger().warn(
                f"Hold timed out after {t_total:.1f}s. Aborting hold.")
            self._finish_hold()
            return

    def _finish_hold(self) -> None:
        self._set_wrench_switch(False)
        self._retreat_t0 = self.get_clock().now()
        if self.last_odom is not None:
            self._retreat_pose_x0 = float(self.last_odom.pose.pose.position.x)
        self.state = TestState.RETREAT

    # ------------------------------------------------------------------ #
    # RETREAT
    # ------------------------------------------------------------------ #

    def _step_retreat(self) -> None:
        if self._retreat_t0 is None:
            self._retreat_t0 = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._retreat_t0).nanoseconds * 1e-9

        self._publish_velocity(-self.retreat_velocity, 0.0, 0.0)
        self._publish_tracking_follow_odom()
        self._set_wrench_switch(False)

        self.get_logger().info(
            f"Retreat {elapsed:.1f}/{self.retreat_duration_sec:.1f}s  "
            f"alt={self.current_alt:.2f}m",
            throttle_duration_sec=1.0)

        if elapsed >= self.retreat_duration_sec:
            self.get_logger().info("Retreat done. Landing.")
            self._land_t0 = None
            self._land_z0 = None
            self.state = TestState.LAND

    # ------------------------------------------------------------------ #
    # LAND
    # ------------------------------------------------------------------ #

    def _step_land(self) -> None:
        if self._land_t0 is None:
            self._land_t0 = self.get_clock().now()
            self._land_z0 = float(self.last_odom.pose.pose.position.z) if self.last_odom else 1.0

        self._publish_velocity(0.0, 0.0, -self.land_velocity)
        self._publish_tracking_follow_odom()
        self._set_wrench_switch(False)

        self.get_logger().info(
            f"Landing... alt={self.current_alt:.2f}m "
            f"(threshold {self.land_alt_threshold:.2f}m)",
            throttle_duration_sec=1.0)

        if self.current_alt <= self.land_alt_threshold:
            if not self._disarm_sent and self.arming_client.service_is_ready():
                req = CommandBool.Request()
                req.value = False
                self.arming_client.call_async(req)
                self._disarm_sent = True
                self.get_logger().info("Disarm requested. Test sequence complete.")
            self.state = TestState.IDLE

    # ------------------------------------------------------------------ #
    # IDLE
    # ------------------------------------------------------------------ #

    def _step_idle(self) -> None:
        self._publish_velocity(0.0, 0.0, 0.0)
        self._publish_tracking_follow_odom()
        if not self._idle_logged:
            self._idle_logged = True
            self.get_logger().info("Idle (landed / disarmed). Ctrl+C to exit.")

    # ------------------------------------------------------------------ #
    # TF helpers
    # ------------------------------------------------------------------ #

    def _publish_map_to_base_link_tf(self) -> None:
        if self.last_odom is None:
            return
        p = self.last_odom.pose.pose
        q = p.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.base_link_frame_id
        t.transform.translation.x = float(p.position.x)
        t.transform.translation.y = float(p.position.y)
        t.transform.translation.z = float(p.position.z)
        t.transform.rotation.x = float(q.x)
        t.transform.rotation.y = float(q.y)
        t.transform.rotation.z = float(q.z)
        t.transform.rotation.w = float(q.w)
        self.tf_broadcaster.sendTransform(t)

    def _publish_base_to_sensor_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_link_frame_id
        t.child_frame_id = self.sensor_frame
        t.transform.translation.z = self.sensor_offset_z
        # Keep identity rotation so base_link X aligns with ft_sensor X.
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _publish_map_to_ned_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.map_ned_frame_id
        # Keep identity rotation so map and map_ned axes are aligned.
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _publish_base_to_frd_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_link_frame_id
        t.child_frame_id = self.frd_frame_id
        # Keep identity rotation so base_link and frd axes are aligned.
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _publish_map_to_contact_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = "contact"
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    # Tracking point helpers
    # ------------------------------------------------------------------ #

    def _publish_tracking_point(self) -> None:
        if self.hold_odom is None:
            return
        p = self.hold_odom.pose.pose.position
        msg = self._make_tracking_odometry(
            float(p.x) + self.hold_x_offset, float(p.y), float(self.takeoff_alt))
        self.tracking_point_pub.publish(msg)

    def _publish_tracking_follow_odom(self) -> None:
        if self.last_odom is None:
            return
        p = self.last_odom.pose.pose.position
        msg = self._make_tracking_odometry(float(p.x), float(p.y), float(p.z))
        self.tracking_point_pub.publish(msg)

    def _make_tracking_odometry(
        self, target_x: float, target_y: float, target_z: float
    ) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.last_odom is not None:
            msg.header.frame_id = self.last_odom.header.frame_id
            msg.child_frame_id = self.last_odom.child_frame_id
            msg.pose.pose.orientation = self.last_odom.pose.pose.orientation
        else:
            msg.header.frame_id = self.map_frame_id
            msg.child_frame_id = self.base_link_frame_id
            msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.position.x = target_x
        msg.pose.pose.position.y = target_y
        msg.pose.pose.position.z = target_z
        return msg

    # ------------------------------------------------------------------ #
    # Velocity / wrench helpers
    # ------------------------------------------------------------------ #

    def _publish_velocity(self, vx: float, vy: float, vz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def _altitude_hold_vz(self, z_ref: float) -> float:
        err = float(z_ref) - float(self.current_alt)
        vz = self.approach_altitude_kp * err
        lim = max(0.0, float(self.approach_vz_limit))
        return max(-lim, min(lim, vz))

    def _publish_wrench_setpoint(self) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_frame
        msg.wrench.force.x = float(self.desired_force)
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
