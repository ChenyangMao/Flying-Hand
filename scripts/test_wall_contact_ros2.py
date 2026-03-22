#!/usr/bin/env python3
"""
Wall contact test using ROS 2 + PX4 SITL + MAVROS.

This script arms the drone, enters OFFBOARD mode, approaches a wall, then
switches the wrench controller to force-control mode and holds a constant
desired force for a configurable duration. After that it retreats (-X),
descends, and disarms (see ``retreat_*`` / ``land_*`` parameters).

By default it uses MAVROS velocity setpoints for takeoff and approach; after
contact, OFFBOARD is fed by ``attitude_thrust_command`` (wrench →
drone_interface → MAVROS).  Set ``use_attitude_offboard_only`` to true to use
``tracking_point`` + wrench pose control for takeoff/approach as well, so PX4
only receives attitude/thrust setpoints after arming (preflight still sends
zero velocity setpoints until OFFBOARD is engaged — PX4 requirement).

Tuning (aligned with ``control_stack_base`` spirit: gentle contact, avoid
saturation): keep ``desired_force`` modest (e.g. 1–3 N), ``approach_velocity``
small (e.g. 0.05–0.12 m/s), and optionally ``takeoff_altitude`` near 1.5 m
like ``scripts/test_wall_contact.py`` (``CONTACT_PUSH_SPEED`` there is 0.05 m/s
for post-contact motion). Raise aggressiveness only after stable holds.

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
from typing import Optional, Tuple

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
    APPROACH = auto()
    HOLD_FORCE = auto()
    RETREAT = auto()  # 后退离墙
    LAND = auto()  # 下降
    IDLE = auto()  # 已降落/解锁结束


class WallContactTester(Node):

    def __init__(self) -> None:
        super().__init__("wall_contact_tester")

        # --------------- parameters --------------- #
        self.declare_parameter("force_topic", "ft_data_filtered")
        self.declare_parameter("force_threshold", 1.0)
        # Start low; raise via -p desired_force:=... once stable (5 N is often too aggressive in sim).
        self.declare_parameter("desired_force", 1.0)
        self.declare_parameter("desired_force_axis", "x")
        self.declare_parameter("approach_velocity", 0.05)
        self.declare_parameter("hold_time", 10.0)
        self.declare_parameter("sensor_frame", "ft_sensor")
        self.declare_parameter("takeoff_altitude", 1.5)
        self.declare_parameter("takeoff_velocity", 0.5)
        self.declare_parameter("loop_rate", 50.0)
        # Pose controller uses TF to compute thrust. In some setups, `map` and
        # `base_link` might be in different TF trees; bridge them here.
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_link_frame_id", "base_link")
        # Also publish base_link -> ft_sensor so map and ft_sensor become connected.
        # Defaults match the ROS1 launch static TF used in control_stack_base.
        self.declare_parameter("sensor_offset_x", 0.0)
        self.declare_parameter("sensor_offset_y", 0.0)
        self.declare_parameter("sensor_offset_z", -0.3)
        self.declare_parameter("sensor_roll", 0.0)
        self.declare_parameter("sensor_pitch", 1.57079632679)
        self.declare_parameter("sensor_yaw", 0.0)
        # If true: after arming, do not publish mavros/setpoint_velocity/cmd_vel;
        # use tracking_point + core_wrench_controller pose loop for takeoff/approach.
        # Preflight still publishes zero velocity until OFFBOARD (PX4 requirement).
        self.declare_parameter("use_attitude_offboard_only", False)
        # 力控结束后：先沿 -X 退离墙面，再下降着陆
        self.declare_parameter("retreat_velocity", 0.08)
        self.declare_parameter("retreat_duration_sec", 4.0)
        self.declare_parameter("land_velocity", 0.25)
        self.declare_parameter("land_alt_threshold", 0.12)

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
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.base_link_frame_id = self.get_parameter("base_link_frame_id").value
        self.sensor_offset_x = self.get_parameter("sensor_offset_x").value
        self.sensor_offset_y = self.get_parameter("sensor_offset_y").value
        self.sensor_offset_z = self.get_parameter("sensor_offset_z").value
        self.sensor_roll = self.get_parameter("sensor_roll").value
        self.sensor_pitch = self.get_parameter("sensor_pitch").value
        self.sensor_yaw = self.get_parameter("sensor_yaw").value
        self.use_attitude_offboard_only = self.get_parameter(
            "use_attitude_offboard_only"
        ).value
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

        # TF broadcaster (repair missing TF connection)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --------------- FT subscriber --------------- #
        self.ft_sub = self.create_subscription(
            WrenchStamped, force_topic, self._ft_callback, 10)

        # --------------- attitude/thrust subscriber (from wrench controller) --------------- #
        self.last_attitude_thrust: Optional[AttitudeThrust] = None
        self.attitude_thrust_sub = self.create_subscription(
            AttitudeThrust, "attitude_thrust_command", self._attitude_thrust_cb, 10)

        # --------------- internal state --------------- #
        self.state = TestState.PREFLIGHT
        self.last_force_msg: Optional[WrenchStamped] = None
        self.last_force_norm: float = 0.0
        self.contact_time: Optional[Time] = None
        self.hold_odom: Optional[Odometry] = None  # snapshot of odom at contact
        self._takeoff_xy_ref: Optional[Tuple[float, float]] = None
        self._approach_t0: Optional[Time] = None
        self._approach_x0: float = 0.0
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
            f"threshold={self.force_threshold:.1f} N, "
            f"desired_force={self.desired_force:.1f} N ({self.desired_force_axis.upper()}), "
            f"use_attitude_offboard_only={self.use_attitude_offboard_only}."
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
        self._publish_map_to_base_link_tf()
        self._publish_base_to_sensor_tf()
        if self.state == TestState.PREFLIGHT:
            self._step_preflight()
        elif self.state == TestState.TAKEOFF:
            self._step_takeoff()
        elif self.state == TestState.APPROACH:
            self._step_approach()
        elif self.state == TestState.HOLD_FORCE:
            self._step_hold_force()
        elif self.state == TestState.RETREAT:
            self._step_retreat()
        elif self.state == TestState.LAND:
            self._step_land()
        elif self.state == TestState.IDLE:
            self._step_idle()

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
        if self.last_odom is not None:
            self._takeoff_xy_ref = (
                float(self.last_odom.pose.pose.position.x),
                float(self.last_odom.pose.pose.position.y),
            )

    # ------------------------------------------------------------------ #
    # TAKEOFF: climb to target altitude
    # ------------------------------------------------------------------ #

    def _step_takeoff(self) -> None:
        if not self.mavros_state or not self.mavros_state.armed:
            self.get_logger().warn("Lost arm during takeoff, re-arming...")
            self._arm_requested = False
            self.state = TestState.PREFLIGHT
            return

        if self.use_attitude_offboard_only:
            self._publish_tracking_takeoff()
        else:
            self._publish_velocity(0.0, 0.0, self.takeoff_vel)

        if self.current_alt >= self.takeoff_alt * 0.95:
            self.get_logger().info(
                f"Reached altitude {self.current_alt:.2f}m (target {self.takeoff_alt:.1f}m). "
                "Starting approach.")
            self.state = TestState.APPROACH
            self._approach_t0 = self.get_clock().now()
            if self.last_odom is not None:
                self._approach_x0 = float(self.last_odom.pose.pose.position.x)
        else:
            self.get_logger().info(
                f"Taking off... alt={self.current_alt:.2f}/{self.takeoff_alt:.1f}m",
                throttle_duration_sec=1.0,
            )

    # ------------------------------------------------------------------ #
    # APPROACH: fly forward until contact
    # ------------------------------------------------------------------ #

    def _step_approach(self) -> None:
        if self.use_attitude_offboard_only:
            self._publish_tracking_approach()
        else:
            self._publish_velocity(self.approach_velocity, 0.0, 0.0)

        if self.last_force_msg is None:
            return

        if self.last_force_norm > self.force_threshold:
            self.get_logger().info(
                f"Contact detected! |F|={self.last_force_norm:.3f} N "
                f"(threshold={self.force_threshold:.1f} N)  "
                f"alt={self.current_alt:.2f}m")
            # Snapshot current pose as the hold position for pose controller
            self.hold_odom = self.last_odom
            self._publish_wrench_setpoint(self.desired_force, self.desired_force_axis)
            self._set_wrench_switch(True)
            self.contact_time = self.get_clock().now()
            self.get_logger().info(
                "Switched to force control. Wrench controller now commands attitude/thrust.")
            self._log_attitude_thrust()
            self.state = TestState.HOLD_FORCE

    # ------------------------------------------------------------------ #
    # HOLD_FORCE: maintain desired contact force
    # ------------------------------------------------------------------ #

    def _step_hold_force(self) -> None:
        # Velocity OFFBOARD: must keep publishing setpoints. Approach was sending vx>0;
        # if we stop here, PX4 can keep blending old velocity with wrench attitude/thrust
        # and altitude collapses. Zero velocity every tick while wrench commands thrust.
        if not self.use_attitude_offboard_only:
            self._publish_velocity(0.0, 0.0, 0.0)

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
            log_msg = (
                f"Hold {elapsed:.1f}/{self.hold_time:.1f}s  "
                f"alt={self.current_alt:.2f}m"
            )
            if self.hold_odom is not None:
                z_ref = float(self.hold_odom.pose.pose.position.z)
                log_msg += f"  z_sp={z_ref:.2f}m"
            log_msg += f"  |F|={self.last_force_norm:.3f} N"
            if self.last_attitude_thrust is not None:
                t = self.last_attitude_thrust.thrust
                q = self.last_attitude_thrust.attitude
                thrust_mag = math.sqrt(t.x**2 + t.y**2 + t.z**2)
                log_msg += (
                    f"  thrust=({t.x:.3f},{t.y:.3f},{t.z:.3f}) |T|={thrust_mag:.3f}"
                    f"  att=(x={q.x:.3f},y={q.y:.3f},z={q.z:.3f},w={q.w:.3f})")
            self.get_logger().info(log_msg, throttle_duration_sec=1.0)

        if elapsed >= self.hold_time:
            self.get_logger().info(
                f"Force hold completed ({self.hold_time:.1f}s). Releasing, then retreat from wall."
            )
            self._set_wrench_switch(False)
            self._publish_wrench_setpoint(0.0, self.desired_force_axis)
            self._retreat_t0 = self.get_clock().now()
            if self.last_odom is not None:
                self._retreat_pose_x0 = float(self.last_odom.pose.pose.position.x)
            self.state = TestState.RETREAT

    def _step_retreat(self) -> None:
        """沿地图 -X 后退（接近墙时为 +X）；纯姿态模式用 tracking 斜坡，速度模式用 cmd_vel。"""
        if self._retreat_t0 is None:
            self._retreat_t0 = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._retreat_t0).nanoseconds * 1e-9

        if self.use_attitude_offboard_only:
            if self.last_odom is not None:
                px = self._retreat_pose_x0 - self.retreat_velocity * elapsed
                py = float(self.last_odom.pose.pose.position.y)
                pz = float(self.last_odom.pose.pose.position.z)
                msg = self._make_tracking_odometry(px, py, pz)
                self.tracking_point_pub.publish(msg)
        else:
            self._publish_velocity(-self.retreat_velocity, 0.0, 0.0)
            self._publish_tracking_follow_odom()

        self._set_wrench_switch(False)
        self._publish_wrench_setpoint(0.0, self.desired_force_axis)

        self.get_logger().info(
            f"Retreat {elapsed:.1f}/{self.retreat_duration_sec:.1f}s  alt={self.current_alt:.2f}m",
            throttle_duration_sec=1.0,
        )
        if elapsed >= self.retreat_duration_sec:
            self.get_logger().info("Retreat done. Landing.")
            self._land_t0 = None
            self._land_z0 = None
            self.state = TestState.LAND

    def _step_land(self) -> None:
        """竖直下降直到近地，然后请求解锁。"""
        if self._land_t0 is None:
            self._land_t0 = self.get_clock().now()
            if self.last_odom is not None:
                self._land_z0 = float(self.last_odom.pose.pose.position.z)
            else:
                self._land_z0 = 1.0

        elapsed_land = (self.get_clock().now() - self._land_t0).nanoseconds * 1e-9

        if self.use_attitude_offboard_only:
            if self.last_odom is not None and self._land_z0 is not None:
                p = self.last_odom.pose.pose.position
                target_z = self._land_z0 - self.land_velocity * elapsed_land
                target_z = max(0.05, target_z)
                msg = self._make_tracking_odometry(
                    float(p.x), float(p.y), float(target_z))
                self.tracking_point_pub.publish(msg)
        else:
            self._publish_velocity(0.0, 0.0, -self.land_velocity)
            self._publish_tracking_follow_odom()

        self._set_wrench_switch(False)
        self._publish_wrench_setpoint(0.0, self.desired_force_axis)

        self.get_logger().info(
            f"Landing... alt={self.current_alt:.2f}m (threshold {self.land_alt_threshold:.2f}m)",
            throttle_duration_sec=1.0,
        )

        if self.current_alt <= self.land_alt_threshold:
            if not self._disarm_sent and self.arming_client.service_is_ready():
                req = CommandBool.Request()
                req.value = False
                self.arming_client.call_async(req)
                self._disarm_sent = True
                self.get_logger().info("Disarm requested. Test sequence complete.")
            self.state = TestState.IDLE

    def _step_idle(self) -> None:
        self._publish_velocity(0.0, 0.0, 0.0)
        self._publish_tracking_follow_odom()
        if not self._idle_logged:
            self._idle_logged = True
            self.get_logger().info(
                "Idle (landed / disarmed). Ctrl+C to exit."
            )

    def _publish_tracking_follow_odom(self) -> None:
        """tracking 与当前里程计一致，减小与速度指令打架时的位姿误差。"""
        if self.last_odom is None:
            return
        p = self.last_odom.pose.pose.position
        msg = self._make_tracking_odometry(
            float(p.x), float(p.y), float(p.z))
        self.tracking_point_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _publish_map_to_base_link_tf(self) -> None:
        """Broadcast `map -> base_link` based on MAVROS odometry.

        The internal pose controller does TF lookups with `target_frame=map`
        and `odom.child_frame_id=base_link`. If those are disconnected in TF,
        thrust/attitude computation fails and no `attitude_thrust_command`
        will be published.
        """
        if self.last_odom is None:
            return

        # Use the pose from mavros/local_position/odom.
        odom_pose = self.last_odom.pose.pose

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.base_link_frame_id

        t.transform.translation.x = float(odom_pose.position.x)
        t.transform.translation.y = float(odom_pose.position.y)
        t.transform.translation.z = float(odom_pose.position.z)

        t.transform.rotation.x = float(odom_pose.orientation.x)
        t.transform.rotation.y = float(odom_pose.orientation.y)
        t.transform.rotation.z = float(odom_pose.orientation.z)
        t.transform.rotation.w = float(odom_pose.orientation.w)

        # If quaternion is invalid (all zeros), skip broadcasting.
        if (
            t.transform.rotation.x == 0.0
            and t.transform.rotation.y == 0.0
            and t.transform.rotation.z == 0.0
            and t.transform.rotation.w == 0.0
        ):
            return

        self.tf_broadcaster.sendTransform(t)

    def _publish_base_to_sensor_tf(self) -> None:
        """Broadcast static `base_link -> ft_sensor` transform.

        Wrench controller requires map<-ft_sensor lookup. With map->base_link
        and this static link, the TF chain becomes connected.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_link_frame_id
        t.child_frame_id = self.sensor_frame

        t.transform.translation.x = float(self.sensor_offset_x)
        t.transform.translation.y = float(self.sensor_offset_y)
        t.transform.translation.z = float(self.sensor_offset_z)

        # Convert RPY to quaternion.
        cr = math.cos(self.sensor_roll * 0.5)
        sr = math.sin(self.sensor_roll * 0.5)
        cp = math.cos(self.sensor_pitch * 0.5)
        sp = math.sin(self.sensor_pitch * 0.5)
        cy = math.cos(self.sensor_yaw * 0.5)
        sy = math.sin(self.sensor_yaw * 0.5)

        t.transform.rotation.w = float(cr * cp * cy + sr * sp * sy)
        t.transform.rotation.x = float(sr * cp * cy - cr * sp * sy)
        t.transform.rotation.y = float(cr * sp * cy + sr * cp * sy)
        t.transform.rotation.z = float(cr * cp * sy - sr * sp * cy)

        self.tf_broadcaster.sendTransform(t)

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

    def _publish_tracking_takeoff(self) -> None:
        """Pose-based takeoff: climb to takeoff_alt at fixed XY (snapshot at TAKEOFF)."""
        if self.last_odom is None:
            return
        if self._takeoff_xy_ref is None:
            self._takeoff_xy_ref = (
                float(self.last_odom.pose.pose.position.x),
                float(self.last_odom.pose.pose.position.y),
            )
        tx, ty = self._takeoff_xy_ref
        msg = self._make_tracking_odometry(tx, ty, float(self.takeoff_alt))
        self.tracking_point_pub.publish(msg)

    def _publish_tracking_approach(self) -> None:
        """Pose-based approach: ramp target X forward at approach_velocity."""
        if self.last_odom is None or self._approach_t0 is None:
            return
        elapsed = (self.get_clock().now() - self._approach_t0).nanoseconds * 1e-9
        target_x = self._approach_x0 + self.approach_velocity * elapsed
        py = float(self.last_odom.pose.pose.position.y)
        msg = self._make_tracking_odometry(target_x, py, float(self.takeoff_alt))
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

    def _attitude_thrust_cb(self, msg: AttitudeThrust) -> None:
        self.last_attitude_thrust = msg

    def _log_attitude_thrust(self) -> None:
        if self.last_attitude_thrust is None:
            self.get_logger().info("  (no attitude/thrust received yet)")
            return
        t = self.last_attitude_thrust.thrust
        q = self.last_attitude_thrust.attitude
        thrust_mag = math.sqrt(t.x**2 + t.y**2 + t.z**2)
        self.get_logger().info(
            f"  thrust=({t.x:.3f},{t.y:.3f},{t.z:.3f}) |T|={thrust_mag:.3f}  "
            f"att=(x={q.x:.3f},y={q.y:.3f},z={q.z:.3f},w={q.w:.3f})")


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
