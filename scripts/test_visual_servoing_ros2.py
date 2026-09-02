#!/usr/bin/env python3
"""
Visual-servo-guided wall contact test using ROS 2 + PX4 SITL + MAVROS.

During approach, `tracking_point` stays at the hover altitude. Horizontal
position is fixed unless `approach_velocity` is non-zero: then the target
creeps in map +X at that speed only while waiting for `/visual_servo/active`
(lock). During lock confirmation and after visual servo is enabled, creep
stops (zero tracking velocity) so IBVS is not fought.

ROS parameters for approach / lock (this node):
  pre_approach_hold_sec, hover_alt_tolerance — gate entry into APPROACH.
  visual_servo_timeout_sec — max age of `/visual_servo/active` and depth msgs
    to count as valid.
  visual_servo_lock_confirm_sec — continuous lock required before enabling VS.
  visual_servo_depth_max — depth threshold to start wrench blend.
  approach_velocity — optional map +X creep (m/s) while lock is false; 0 = fixed.

Publishes `/visual_servo/enable` so IBVS can drive the vehicle once lock is
confirmed, and arms wrench control after depth enters the blend window. After
F/T contact, the task switches to force hold, then detaches, retreats, and lands.
"""

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped, Vector3Stamped, WrenchStamped
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
    DETACH = auto()
    RETREAT = auto()
    LAND = auto()
    IDLE = auto()


class WallContactTester(Node):

    def __init__(self) -> None:
        super().__init__("wall_contact_tester")

        # --------------- parameters --------------- #
        self.declare_parameter("force_topic", "ft_data_filtered")
        self.declare_parameter("force_threshold", 1.8)
        self.declare_parameter("contact_force_max", 8.0)
        self.declare_parameter("contact_confirm_sec", 0.15)
        self.declare_parameter("contact_alt_tolerance", 0.25)
        self.declare_parameter("visual_servo_timeout_sec", 0.6)
        self.declare_parameter("visual_servo_lock_confirm_sec", 0.6)
        self.declare_parameter("visual_servo_depth_max", 1.30)
        self.declare_parameter("desired_force", 5.0)
        self.declare_parameter("use_contact_force_sign_for_setpoint", True)
        self.declare_parameter("approach_velocity", 0.0)
        self.declare_parameter("approach_forward_offset", 0.12)
        self.declare_parameter("contact_push_velocity", 0.01)
        self.declare_parameter("contact_push_threshold", 0.8)
        self.declare_parameter("hold_time", 20.0)
        self.declare_parameter("force_hold_tolerance", 2.0)
        self.declare_parameter("max_hold_timeout", 90.0)
        self.declare_parameter("hold_force_nudge_vel", 0.003)
        self.declare_parameter("hold_force_nudge_deadband", 0.2)
        self.declare_parameter("hold_force_nudge_max_x_offset", 0.05)
        self.declare_parameter("hold_lateral_abort_y", 0.35)
        self.declare_parameter("hold_vertical_abort_z", 0.35)
        self.declare_parameter("sensor_frame", "ft_sensor")
        self.declare_parameter("takeoff_altitude", 1.20)
        self.declare_parameter("takeoff_velocity", 0.15)
        self.declare_parameter("takeoff_alt_tolerance", 0.05)
        self.declare_parameter("takeoff_settle_vel", 0.10)
        self.declare_parameter("takeoff_settle_sec", 0.5)
        self.declare_parameter("hover_alt_tolerance", 0.08)
        self.declare_parameter("pre_approach_hold_sec", 3.0)
        self.declare_parameter("loop_rate", 100.0)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("map_ned_frame_id", "map_ned")
        self.declare_parameter("base_link_frame_id", "base_link")
        self.declare_parameter("frd_frame_id", "base_link_frd")
        self.declare_parameter("sensor_offset_z", -0.3)
        self.declare_parameter("hold_x_offset", 0.0)
        self.declare_parameter("detach_velocity", 0.05)
        self.declare_parameter("detach_distance", 0.20)
        self.declare_parameter("retreat_velocity", 0.08)
        self.declare_parameter("retreat_duration_sec", 4.0)
        self.declare_parameter("land_velocity", 0.25)
        self.declare_parameter("land_alt_threshold", 0.12)
        self.declare_parameter("force_hard_abort_limit", 15.0)
        self.declare_parameter("odom_dropout_timeout_sec", 0.5)
        self.declare_parameter("ft_dropout_timeout_sec", 0.5)

        force_topic = self.get_parameter("force_topic").value
        self.force_threshold = float(self.get_parameter("force_threshold").value)
        self.contact_force_max = float(self.get_parameter("contact_force_max").value)
        self.contact_confirm_sec = float(self.get_parameter("contact_confirm_sec").value)
        self.contact_alt_tolerance = float(self.get_parameter("contact_alt_tolerance").value)
        self.visual_servo_timeout_sec = float(
            self.get_parameter("visual_servo_timeout_sec").value)
        self.visual_servo_lock_confirm_sec = float(
            self.get_parameter("visual_servo_lock_confirm_sec").value)
        self.visual_servo_depth_max = float(self.get_parameter("visual_servo_depth_max").value)
        self.desired_force = float(self.get_parameter("desired_force").value)
        self.use_contact_force_sign_for_setpoint = bool(
            self.get_parameter("use_contact_force_sign_for_setpoint").value)
        self.approach_velocity = float(self.get_parameter("approach_velocity").value)
        self.approach_forward_offset = float(self.get_parameter("approach_forward_offset").value)
        self.contact_push_velocity = float(self.get_parameter("contact_push_velocity").value)
        self.contact_push_threshold = float(self.get_parameter("contact_push_threshold").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.force_hold_tolerance = float(self.get_parameter("force_hold_tolerance").value)
        self.max_hold_timeout = float(self.get_parameter("max_hold_timeout").value)
        self.hold_force_nudge_vel = float(self.get_parameter("hold_force_nudge_vel").value)
        self.hold_force_nudge_deadband = float(self.get_parameter("hold_force_nudge_deadband").value)
        self.hold_force_nudge_max_x_offset = float(
            self.get_parameter("hold_force_nudge_max_x_offset").value)
        self.hold_lateral_abort_y = float(self.get_parameter("hold_lateral_abort_y").value)
        self.hold_vertical_abort_z = float(self.get_parameter("hold_vertical_abort_z").value)
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.takeoff_alt = float(self.get_parameter("takeoff_altitude").value)
        self.takeoff_vel = float(self.get_parameter("takeoff_velocity").value)
        self.takeoff_alt_tolerance = float(self.get_parameter("takeoff_alt_tolerance").value)
        self.takeoff_settle_vel = float(self.get_parameter("takeoff_settle_vel").value)
        self.takeoff_settle_sec = float(self.get_parameter("takeoff_settle_sec").value)
        self.hover_alt_tolerance = float(self.get_parameter("hover_alt_tolerance").value)
        self.pre_approach_hold_sec = float(self.get_parameter("pre_approach_hold_sec").value)
        loop_rate = self.get_parameter("loop_rate").value
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.map_ned_frame_id = self.get_parameter("map_ned_frame_id").value
        self.base_link_frame_id = self.get_parameter("base_link_frame_id").value
        self.frd_frame_id = self.get_parameter("frd_frame_id").value
        self.sensor_offset_z = float(self.get_parameter("sensor_offset_z").value)
        self.hold_x_offset = float(self.get_parameter("hold_x_offset").value)
        self.detach_velocity = float(self.get_parameter("detach_velocity").value)
        self.detach_distance = float(self.get_parameter("detach_distance").value)
        self.retreat_velocity = float(self.get_parameter("retreat_velocity").value)
        self.retreat_duration_sec = float(self.get_parameter("retreat_duration_sec").value)
        self.land_velocity = float(self.get_parameter("land_velocity").value)
        self.land_alt_threshold = float(self.get_parameter("land_alt_threshold").value)
        self.force_hard_abort_limit = float(self.get_parameter("force_hard_abort_limit").value)
        self.odom_dropout_timeout_sec = float(self.get_parameter("odom_dropout_timeout_sec").value)
        self.ft_dropout_timeout_sec = float(self.get_parameter("ft_dropout_timeout_sec").value)

        # --------------- MAVROS publishers / subscribers --------------- #
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.mavros_state: Optional[MavrosState] = None
        self.state_sub = self.create_subscription(
            MavrosState, "mavros/state", self._mavros_state_cb, state_qos)

        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        # --------------- wrench controller publishers --------------- #
        self.ft_setpoint_pub = self.create_publisher(WrenchStamped, "ft_setpoint", 10)
        self.switch_pub = self.create_publisher(Bool, "wrench_controller/switch", 10)
        self.tracking_point_pub = self.create_publisher(Odometry, "tracking_point", 10)
        self.visual_servo_enable_pub = self.create_publisher(Bool, "/visual_servo/enable", 10)

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

        self.visual_servo_active = False
        self.visual_servo_depth: Optional[float] = None
        self._visual_servo_status_time: Optional[Time] = None
        self._visual_servo_depth_time: Optional[Time] = None
        self.vs_active_sub = self.create_subscription(
            Bool, "/visual_servo/active", self._visual_servo_active_cb, 10)
        self.vs_depth_sub = self.create_subscription(
            Vector3Stamped, "/visual_servo/depth", self._visual_servo_depth_cb, 10)

        # --------------- attitude/thrust subscriber --------------- #
        self.last_attitude_thrust: Optional[AttitudeThrust] = None
        self.attitude_thrust_sub = self.create_subscription(
            AttitudeThrust, "attitude_thrust_command", self._attitude_thrust_cb, 10)

        # --------------- internal state --------------- #
        self.state = TestState.PREFLIGHT
        self.last_force_msg: Optional[WrenchStamped] = None
        self.last_fx_raw: float = 0.0
        self.last_fx_abs: float = 0.0
        self.active_force_setpoint_x: float = self.desired_force
        self.contact_time: Optional[Time] = None
        self._contact_candidate_since: Optional[Time] = None
        self._in_band_since: Optional[Time] = None
        self.hold_odom: Optional[Odometry] = None
        self.hold_x_reference: Optional[float] = None
        self._hover_t0: Optional[Time] = None
        self._vs_lock_since: Optional[Time] = None
        self._takeoff_stable_since: Optional[Time] = None
        self._arm_requested = False
        self._offboard_requested = False
        self._preflight_log_timer = 0
        self._retreat_t0: Optional[Time] = None
        self._detach_start_x: Optional[float] = None
        self._detach_target_z: Optional[float] = None
        self._land_t0: Optional[Time] = None
        self._disarm_sent = False
        self._idle_logged = False
        self._force_blend_started = False
        self._last_odom_time: Optional[Time] = None
        self._last_ft_time: Optional[Time] = None
        self._hard_abort_triggered = False
        self._approach_start_y = 0.0
        self._target_initialized = False
        self._command_target_x = 0.0
        self._command_target_y = 0.0
        self._command_target_z = 0.0
        self._command_vel_x = 0.0
        self._command_vel_y = 0.0
        self._command_vel_z = 0.0

        # --------------- main loop --------------- #
        self._loop_period = 1.0 / loop_rate if loop_rate > 0 else 0.02
        self.timer = self.create_timer(self._loop_period, self._loop)

        self.get_logger().info(
            f"WallContactTester started. force_topic='{force_topic}', "
            f"vs_lock_confirm={self.visual_servo_lock_confirm_sec:.2f}s, "
            f"vs_depth_switch={self.visual_servo_depth_max:.2f} m, "
            f"contact_window=[{self.force_threshold:.2f}, {self.contact_force_max:.2f}] N, "
            f"contact_confirm={self.contact_confirm_sec:.2f}s, "
            f"desired_Fx={self.desired_force:.1f} N, "
            f"hold: {self.hold_time:.1f}s within "
            f"{self.desired_force:.1f}±{self.force_hold_tolerance:.2f} N, "
            f"x_hold_ref_offset={self.hold_x_offset:.2f}m, "
            f"(script x-nudge disabled; wrench owns x after contact), "
            f"hold_y_abort={self.hold_lateral_abort_y:.2f}m, "
            f"hold_z_abort={self.hold_vertical_abort_z:.2f}m."
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _mavros_state_cb(self, msg: MavrosState) -> None:
        self.mavros_state = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_alt = msg.pose.pose.position.z
        self.last_odom = msg
        self._last_odom_time = self.get_clock().now()

    def _ft_callback(self, msg: WrenchStamped) -> None:
        self.last_force_msg = msg
        self.last_fx_raw = float(msg.wrench.force.x)
        self.last_fx_abs = abs(self.last_fx_raw)
        self._last_ft_time = self.get_clock().now()

    def _visual_servo_active_cb(self, msg: Bool) -> None:
        self.visual_servo_active = bool(msg.data)
        self._visual_servo_status_time = self.get_clock().now()

    def _visual_servo_depth_cb(self, msg: Vector3Stamped) -> None:
        self.visual_servo_depth = float(msg.vector.z)
        self._visual_servo_depth_time = self.get_clock().now()

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

        # --- Safety watchdogs (active during flight states) --- #
        if self.state not in (TestState.PREFLIGHT, TestState.IDLE):
            now = self.get_clock().now()

            # Hard force abort: if |Fx| exceeds a critical limit, immediately
            # disengage force control and retreat to prevent structural damage.
            if (self.force_hard_abort_limit > 0.0 and
                    self.last_fx_abs > self.force_hard_abort_limit and
                    not self._hard_abort_triggered):
                self._hard_abort_triggered = True
                self.get_logger().error(
                    f"HARD ABORT: |Fx|={self.last_fx_abs:.2f} N exceeds "
                    f"limit {self.force_hard_abort_limit:.1f} N. "
                    "Disabling wrench control and retreating.")
                self._finish_hold()
                return

            # Odometry dropout: if no odom arrives for too long, fall back to
            # wrench-off hover so the pose controller does not accumulate stale error.
            if (self._last_odom_time is not None and
                    self.state in (TestState.APPROACH, TestState.HOLD_FORCE)):
                odom_age = (now - self._last_odom_time).nanoseconds * 1e-9
                if odom_age > self.odom_dropout_timeout_sec:
                    self.get_logger().warn(
                        f"Odometry dropout ({odom_age:.2f}s). "
                        "Disabling wrench control, retreating.",
                        throttle_duration_sec=1.0)
                    self._finish_hold()
                    return

            # FT sensor dropout: if force data goes stale during force hold,
            # detach to avoid applying unmonitored force.
            if (self.state == TestState.HOLD_FORCE and
                    self._last_ft_time is not None):
                ft_age = (now - self._last_ft_time).nanoseconds * 1e-9
                if ft_age > self.ft_dropout_timeout_sec:
                    self.get_logger().warn(
                        f"FT sensor dropout ({ft_age:.2f}s) during force hold. "
                        "Aborting hold.",
                        throttle_duration_sec=1.0)
                    self._finish_hold()
                    return

        handler = {
            TestState.PREFLIGHT: self._step_preflight,
            TestState.TAKEOFF: self._step_takeoff,
            TestState.HOVER_ALT: self._step_hover_alt,
            TestState.APPROACH: self._step_approach,
            TestState.HOLD_FORCE: self._step_hold_force,
            TestState.DETACH: self._step_detach,
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

        if self.last_odom is None:
            self._preflight_log_timer += 1
            if self._preflight_log_timer % 100 == 1:
                self.get_logger().info("Waiting for local odometry...")
            return

        if not self._target_initialized:
            self._set_tracking_target_from_current_odom()

        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        self._set_wrench_switch(False)
        self._force_blend_started = False
        self._publish_tracking_point_target()

        if self.last_attitude_thrust is None:
            self._preflight_log_timer += 1
            if self._preflight_log_timer % 100 == 1:
                self.get_logger().info("Waiting for external controller setpoints...")
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

        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        self._command_vel_x = 0.0
        self._command_vel_y = 0.0
        if not self._target_initialized:
            self._set_tracking_target_from_current_odom()
        if self._command_target_z < self.takeoff_alt:
            next_target_z = min(self._command_target_z + self.takeoff_vel * self._loop_period,
                                self.takeoff_alt)
            self._command_target_z = next_target_z
            self._command_vel_z = self.takeoff_vel if next_target_z < self.takeoff_alt else 0.0
        else:
            self._command_target_z = self.takeoff_alt
            self._command_vel_z = 0.0
        self._publish_tracking_point_target()

        current_vz = self._current_vertical_velocity()
        alt_err = abs(self.current_alt - self.takeoff_alt)
        alt_ready = (
            self._command_target_z >= self.takeoff_alt and
            alt_err <= self.takeoff_alt_tolerance and
            abs(current_vz) <= self.takeoff_settle_vel
        )

        now = self.get_clock().now()
        if alt_ready:
            if self._takeoff_stable_since is None:
                self._takeoff_stable_since = now
            stable_time = (now - self._takeoff_stable_since).nanoseconds * 1e-9
            if stable_time >= self.takeoff_settle_sec:
                self.get_logger().info(
                    f"Reached altitude {self.current_alt:.2f}m with vz={current_vz:.2f}m/s "
                    f"(target {self.takeoff_alt:.1f}m). Hovering "
                    f"{self.pre_approach_hold_sec:.1f}s to stabilize before approach.")
                self.state = TestState.HOVER_ALT
                self._hover_t0 = None
                return
        else:
            self._takeoff_stable_since = None
            stable_time = 0.0

        self.get_logger().info(
            f"Taking off... alt={self.current_alt:.2f}/{self.takeoff_alt:.1f}m  "
            f"target_z={self._command_target_z:.2f}m  vz={current_vz:.2f}m/s  "
            f"settled={stable_time:.1f}/{self.takeoff_settle_sec:.1f}s",
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

        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        self._command_vel_x = 0.0
        self._command_vel_y = 0.0
        self._command_vel_z = 0.0
        self._command_target_z = self.takeoff_alt
        self._publish_tracking_point_target()

        current_vz = self._current_vertical_velocity()
        alt_err = abs(self.current_alt - self.takeoff_alt)
        hover_ready = (
            alt_err <= self.hover_alt_tolerance and
            abs(current_vz) <= self.takeoff_settle_vel
        )
        now = self.get_clock().now()
        if hover_ready:
            if self._hover_t0 is None:
                self._hover_t0 = now
            elapsed = (now - self._hover_t0).nanoseconds * 1e-9
        else:
            self._hover_t0 = None
            elapsed = 0.0

        self.get_logger().info(
            f"Pre-approach hover {elapsed:.1f}/{self.pre_approach_hold_sec:.1f}s  "
            f"alt={self.current_alt:.2f}m (target {self.takeoff_alt:.1f}m)  "
            f"vz={current_vz:.2f}m/s",
            throttle_duration_sec=1.0)

        if elapsed >= self.pre_approach_hold_sec:
            self.get_logger().info(
                "Hover done. Entering approach and waiting for stable visual lock.")
            self._force_blend_started = False
            if self.last_odom is not None:
                self._approach_start_y = float(
                    self.last_odom.pose.pose.position.y)
            self.state = TestState.APPROACH

    # ------------------------------------------------------------------ #
    # APPROACH
    # ------------------------------------------------------------------ #

    def _step_approach(self) -> None:
        vs_locked = self._visual_servo_locked()
        depth_ready = self._visual_servo_depth_ready()
        depth = self.visual_servo_depth if depth_ready else None
        now = self.get_clock().now()

        if not vs_locked:
            self._command_target_z = self.takeoff_alt
            if self.approach_velocity != 0.0:
                self._advance_tracking_target(self.approach_velocity, 0.0, 0.0)
            else:
                self._command_vel_x = 0.0
                self._command_vel_y = 0.0
                self._command_vel_z = 0.0
            self._set_visual_servo_enable(False)
            self._vs_lock_since = None
            if self._force_blend_started:
                self.get_logger().warn(
                    "Visual servo lost before contact. Disabling wrench blend and holding hover.",
                    throttle_duration_sec=1.0)
            self._set_wrench_switch(False)
            self._force_blend_started = False
            self._contact_candidate_since = None
            self._publish_tracking_point_target()
            self.get_logger().info(
                "Approach: waiting for visual target lock.",
                throttle_duration_sec=1.0)
            return

        self._command_vel_x = 0.0
        self._command_vel_y = 0.0
        self._command_vel_z = 0.0
        self._command_target_z = self.takeoff_alt
        self._publish_tracking_point_target()

        if self._vs_lock_since is None:
            self._vs_lock_since = now

        lock_dwell = (now - self._vs_lock_since).nanoseconds * 1e-9
        if lock_dwell < self.visual_servo_lock_confirm_sec:
            self._set_visual_servo_enable(False)
            self._set_wrench_switch(False)
            self.get_logger().info(
                f"Approach: target seen, confirming stable lock "
                f"{lock_dwell:.2f}/{self.visual_servo_lock_confirm_sec:.2f}s.",
                throttle_duration_sec=1.0)
            return

        if self.last_odom is not None:
            current_pos = self.last_odom.pose.pose.position
            if not self._force_blend_started:
                self._command_target_x = float(current_pos.x) + self.approach_forward_offset
            else:
                self._command_target_x = float(current_pos.x)
            self._command_target_y = self._approach_start_y
            self._command_target_z = self.takeoff_alt
            self._publish_tracking_point_target()

        self._set_visual_servo_enable(True)

        if depth_ready and not self._force_blend_started and depth <= self.visual_servo_depth_max:
            self.active_force_setpoint_x = self.desired_force
            self._publish_wrench_setpoint()
            self._set_wrench_switch(True)
            self._force_blend_started = True
            self.get_logger().info(
                f"Approach: target locked, depth={depth:.3f}m <= {self.visual_servo_depth_max:.3f}m. "
                "Starting VS/wrench blend.")
        elif self._force_blend_started:
            self._publish_wrench_setpoint()
            self._set_wrench_switch(True)
        else:
            self._set_wrench_switch(False)

        if self._force_blend_started and not depth_ready and self.last_fx_abs < self.force_threshold:
            self.get_logger().warn(
                "Approach: depth feedback stale before contact, disabling wrench blend.",
                throttle_duration_sec=1.0)
            self._set_wrench_switch(False)
            self._force_blend_started = False

        depth_text = f"{depth:.3f}m" if depth_ready and depth is not None else "n/a"
        self.get_logger().info(
            f"Approach: VS locked, depth={depth_text}, blend={'on' if self._force_blend_started else 'off'}, "
            f"|Fx|={self.last_fx_abs:.3f}N",
            throttle_duration_sec=1.0)

        if self.last_force_msg is None:
            self._contact_candidate_since = None
            return

        alt_ok = abs(self.current_alt - self.takeoff_alt) <= self.contact_alt_tolerance
        in_contact_window = self.force_threshold <= self.last_fx_abs <= self.contact_force_max

        if not alt_ok:
            self._contact_candidate_since = None
            self.get_logger().info(
                f"Contact candidate ignored due to altitude offset: "
                f"alt={self.current_alt:.2f}m target={self.takeoff_alt:.2f}m",
                throttle_duration_sec=1.0)
            return

        if self.last_fx_abs > self.contact_force_max:
            self._contact_candidate_since = None
            self.get_logger().warn(
                f"Ignoring contact spike |Fx|={self.last_fx_abs:.3f} N "
                f"(contact max={self.contact_force_max:.2f} N)",
                throttle_duration_sec=1.0)
            return

        if not in_contact_window:
            self._contact_candidate_since = None
            return

        if self._contact_candidate_since is None:
            self._contact_candidate_since = now
            return

        contact_dwell = (now - self._contact_candidate_since).nanoseconds * 1e-9
        if contact_dwell < self.contact_confirm_sec:
            return

        measured_contact_fx = self.last_fx_raw
        if self.use_contact_force_sign_for_setpoint and abs(measured_contact_fx) > 1e-6:
            self.active_force_setpoint_x = abs(self.desired_force) * (
                1.0 if measured_contact_fx >= 0.0 else -1.0)
        else:
            self.active_force_setpoint_x = self.desired_force

        self.get_logger().info(
            f"Contact detected! raw Fx={self.last_fx_raw:.3f} N  "
            f"|Fx|={self.last_fx_abs:.3f} N  "
            f"(window=[{self.force_threshold:.2f}, {self.contact_force_max:.2f}] N)  "
            f"alt={self.current_alt:.2f}m  "
            f"setpoint Fx={self.active_force_setpoint_x:.3f} N")
        self.hold_odom = self.last_odom
        if self.hold_odom is not None:
            p = self.hold_odom.pose.pose.position
            self.hold_x_reference = float(p.x) + self.hold_x_offset
            self._set_tracking_target(self.hold_x_reference, float(p.y), self.takeoff_alt)
        self._publish_tracking_point_target()
        self._publish_wrench_setpoint()
        self._set_wrench_switch(True)
        self._force_blend_started = True
        self.contact_time = now
        self._contact_candidate_since = None
        self._in_band_since = None
        self.get_logger().info(
            "Switched to force control. Wrench controller now commands attitude/thrust.")
        self.state = TestState.HOLD_FORCE

    # ------------------------------------------------------------------ #
    # HOLD_FORCE
    # ------------------------------------------------------------------ #

    def _step_hold_force(self) -> None:
        self._set_visual_servo_enable(True)
        if self.hold_odom is not None and self.last_odom is not None:
            hold_p = self.hold_odom.pose.pose.position
            cur_p = self.last_odom.pose.pose.position

            y_err = float(cur_p.y) - float(hold_p.y)
            z_err = float(cur_p.z) - float(self.takeoff_alt)
            if abs(y_err) > self.hold_lateral_abort_y:
                self.get_logger().warn(
                    f"Hold aborted due to lateral drift: dy={y_err:.3f}m "
                    f"(limit {self.hold_lateral_abort_y:.3f}m)")
                self._finish_hold()
                return
            if abs(z_err) > self.hold_vertical_abort_z:
                self.get_logger().warn(
                    f"Hold aborted due to vertical drift: dz={z_err:.3f}m "
                    f"(limit {self.hold_vertical_abort_z:.3f}m)")
                self._finish_hold()
                return

            desired_x = self.hold_x_reference if self.hold_x_reference is not None else float(hold_p.x)
            self._set_tracking_target(desired_x, float(hold_p.y), self.takeoff_alt)
            self._command_vel_x = 0.0
            self._command_vel_y = 0.0
            self._command_vel_z = 0.0

        self._publish_tracking_point_target()
        self._publish_wrench_setpoint()
        self._set_wrench_switch(True)

        if self.contact_time is None:
            self.contact_time = self.get_clock().now()
            self._in_band_since = None

        now = self.get_clock().now()
        t_total = (now - self.contact_time).nanoseconds * 1e-9

        in_band = abs(self.last_fx_raw - self.active_force_setpoint_x) <= self.force_hold_tolerance

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
            f"alt={self.current_alt:.2f}m  "
            f"Fx_raw={self.last_fx_raw:.3f} N  |Fx|={self.last_fx_abs:.3f} N  "
            f"Fx_sp={self.active_force_setpoint_x:.3f} N"
        )
        if self.hold_odom is not None and self.last_odom is not None:
            hp = self.hold_odom.pose.pose.position
            cp = self.last_odom.pose.pose.position
            tx = self.hold_x_reference if self.hold_x_reference is not None else float(hp.x)
            ty = float(cp.y)
            tz = float(self.takeoff_alt)
            log_msg += (
                f"  target_ref=({tx:.3f},{ty:.3f},{tz:.3f})  "
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
        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        self._set_wrench_switch(False)
        self._force_blend_started = False
        self._command_vel_x = 0.0
        self._command_vel_y = 0.0
        self._command_vel_z = 0.0
        self._retreat_t0 = None
        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            self._set_tracking_target(float(p.x), float(p.y), float(p.z))
            self._detach_start_x = float(p.x)
            self._detach_target_z = float(p.z)
        else:
            self._detach_start_x = self._command_target_x
            self._detach_target_z = self._command_target_z
        self.get_logger().info(
            f"Detached from force control. Switching to pose-only retreat; "
            f"backing off {self.detach_distance:.2f}m at {self.detach_velocity:.2f} m/s.")
        self.state = TestState.DETACH

    # ------------------------------------------------------------------ #
    # DETACH
    # ------------------------------------------------------------------ #

    def _step_detach(self) -> None:
        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        self._set_wrench_switch(False)
        if self._detach_start_x is None:
            self._detach_start_x = self._command_target_x
        if self._detach_target_z is None:
            self._detach_target_z = self._command_target_z

        self._advance_tracking_target(-self.detach_velocity, 0.0, 0.0)
        self._command_target_z = self._detach_target_z
        self._command_vel_z = 0.0
        self._publish_tracking_point_target()

        backed_off = max(0.0, self._detach_start_x - self._command_target_x)
        self.get_logger().info(
            f"Detach {backed_off:.2f}/{self.detach_distance:.2f}m  "
            f"alt={self.current_alt:.2f}m",
            throttle_duration_sec=1.0)

        if backed_off >= self.detach_distance:
            self.get_logger().info("Detach complete. Continuing pose-only retreat.")
            self._retreat_t0 = self.get_clock().now()
            self.state = TestState.RETREAT

    # ------------------------------------------------------------------ #
    # RETREAT
    # ------------------------------------------------------------------ #

    def _step_retreat(self) -> None:
        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        if self._retreat_t0 is None:
            self._retreat_t0 = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._retreat_t0).nanoseconds * 1e-9

        self._advance_tracking_target(-self.retreat_velocity, 0.0, 0.0)
        self._publish_tracking_point_target()
        self._set_wrench_switch(False)

        self.get_logger().info(
            f"Retreat {elapsed:.1f}/{self.retreat_duration_sec:.1f}s  "
            f"alt={self.current_alt:.2f}m",
            throttle_duration_sec=1.0)

        if elapsed >= self.retreat_duration_sec:
            self.get_logger().info("Retreat done. Landing.")
            self._land_t0 = None
            self.state = TestState.LAND

    # ------------------------------------------------------------------ #
    # LAND
    # ------------------------------------------------------------------ #

    def _step_land(self) -> None:
        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        if self._land_t0 is None:
            self._land_t0 = self.get_clock().now()

        self._advance_tracking_target(0.0, 0.0, -self.land_velocity)
        if self._command_target_z < 0.0:
            self._command_target_z = 0.0
            self._command_vel_z = 0.0
        self._publish_tracking_point_target()
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
        self._set_visual_servo_enable(False)
        self._vs_lock_since = None
        self._publish_tracking_point_target()
        self._set_wrench_switch(False)
        self._force_blend_started = False
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
        # Match control_stack_base launch: map -> contact is yaw=pi.
        t.transform.rotation.z = 1.0
        t.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    # Tracking point helpers
    # ------------------------------------------------------------------ #

    def _set_tracking_target_from_current_odom(self) -> None:
        if self.last_odom is None:
            return
        p = self.last_odom.pose.pose.position
        self._set_tracking_target(float(p.x), float(p.y), float(p.z))

    def _set_tracking_target(self, target_x: float, target_y: float, target_z: float) -> None:
        self._command_target_x = float(target_x)
        self._command_target_y = float(target_y)
        self._command_target_z = float(target_z)
        self._command_vel_x = 0.0
        self._command_vel_y = 0.0
        self._command_vel_z = 0.0
        self._target_initialized = True

    def _advance_tracking_target(self, vx: float, vy: float, vz: float) -> None:
        if not self._target_initialized:
            self._set_tracking_target_from_current_odom()
            if not self._target_initialized:
                return
        self._command_vel_x = float(vx)
        self._command_vel_y = float(vy)
        self._command_vel_z = float(vz)
        self._command_target_x += float(vx) * self._loop_period
        self._command_target_y += float(vy) * self._loop_period
        self._command_target_z += float(vz) * self._loop_period

    def _publish_tracking_point_target(self) -> None:
        if not self._target_initialized:
            self._set_tracking_target_from_current_odom()
            if not self._target_initialized:
                return
        msg = self._make_tracking_odometry(
            self._command_target_x,
            self._command_target_y,
            self._command_target_z,
            self._command_vel_x,
            self._command_vel_y,
            self._command_vel_z,
        )
        self.tracking_point_pub.publish(msg)

    def _make_tracking_odometry(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        vel_x: float = 0.0,
        vel_y: float = 0.0,
        vel_z: float = 0.0,
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
        msg.twist.twist.linear.x = float(vel_x)
        msg.twist.twist.linear.y = float(vel_y)
        msg.twist.twist.linear.z = float(vel_z)
        return msg

    # ------------------------------------------------------------------ #
    # Wrench helpers
    # ------------------------------------------------------------------ #

    def _publish_wrench_setpoint(self) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_frame
        msg.wrench.force.x = float(self.active_force_setpoint_x)
        self.ft_setpoint_pub.publish(msg)

    def _set_wrench_switch(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.switch_pub.publish(msg)

    def _set_visual_servo_enable(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.visual_servo_enable_pub.publish(msg)

    def _stamp_is_recent(self, stamp: Optional[Time]) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= self.visual_servo_timeout_sec

    def _visual_servo_locked(self) -> bool:
        return self.visual_servo_active and self._stamp_is_recent(self._visual_servo_status_time)

    def _visual_servo_depth_ready(self) -> bool:
        return self.visual_servo_depth is not None and self._stamp_is_recent(
            self._visual_servo_depth_time)

    def _current_vertical_velocity(self) -> float:
        if self.last_odom is None:
            return 0.0
        return float(self.last_odom.twist.twist.linear.z)


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
