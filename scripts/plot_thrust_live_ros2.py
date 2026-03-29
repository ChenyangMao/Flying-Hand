#!/usr/bin/env python3
"""
Live monitoring dashboard for hybrid force/position control.

The 5 subplots correspond to the main tuning concerns:
  1. Force Fx tracking  - force PI tuning: measured force vs desired force vs error
  2. X position         - position PD damping tuning: current x vs target x vs x error
  3. Thrust output      - thrust commands: wrench x / pose y,z / ||T||
  4. YZ position hold   - pose-controller hold quality: y error, z error
  5. Attitude           - overall stability: pitch / roll

Usage:
  python3 scripts/plot_thrust_live_ros2.py

Dependency: matplotlib
"""
from __future__ import annotations

import argparse
import math
import threading
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import WrenchStamped
from mav_msgs.msg import AttitudeThrust
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--topic", default="attitude_thrust_command",
                    help="AttitudeThrust topic")
    p.add_argument("--force-topic", default="ft_data_filtered",
                    help="Measured force topic (WrenchStamped)")
    p.add_argument("--force-setpoint-topic", default="ft_setpoint",
                    help="Desired force topic (WrenchStamped)")
    p.add_argument("--odom-topic", default="mavros/local_position/odom",
                    help="Odometry topic (Odometry)")
    p.add_argument("--tracking-topic", default="tracking_point",
                    help="Tracking target topic (Odometry)")
    p.add_argument("--switch-topic", default="wrench_controller/switch",
                    help="Mode switch topic (Bool)")
    p.add_argument("--history-sec", type=float, default=45.0,
                    help="Sliding time window on the x-axis (seconds)")
    p.add_argument("--max-points", type=int, default=6000,
                    help="Maximum number of retained samples")
    p.add_argument("--interval-ms", type=int, default=50,
                    help="Matplotlib refresh interval (milliseconds)")
    p.add_argument("--sample-hz", type=float, default=40.0,
                    help="Internal sampling frequency (Hz)")
    p.add_argument("--no-magnitude", action="store_true",
                    help="Do not plot the ||T|| curve")
    return p.parse_args()


def _import_matplotlib():
    try:
        import matplotlib.pyplot as plt
        from matplotlib import animation
        return plt, animation
    except Exception as e:
        hint = (
            "\nCommon cause: the system-provided python3-matplotlib was built "
            "against NumPy 1.x, while the current environment uses NumPy 2.x.\n"
            "Fix: pip install --user --upgrade matplotlib\n"
        )
        raise SystemExit(
            f"Failed to import matplotlib.{hint}\nOriginal error: {type(e).__name__}: {e}"
        ) from e


class HybridControlMonitor(Node):
    """Subscribe to all control-related topics and sample them into time series."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hybrid_control_monitor")
        self._lock = threading.Lock()
        self._t0: Optional[float] = None
        self._has_data = False
        n = args.max_points
        self._history_sec = args.history_sec

        # ---- time series deques (filled by sampling timer) ---- #
        self._t: Deque[float] = deque(maxlen=n)
        self._tx: Deque[float] = deque(maxlen=n)
        self._ty: Deque[float] = deque(maxlen=n)
        self._tz: Deque[float] = deque(maxlen=n)
        self._pitch_deg: Deque[float] = deque(maxlen=n)
        self._roll_deg: Deque[float] = deque(maxlen=n)
        self._f_meas_x: Deque[float] = deque(maxlen=n)
        self._f_des_x: Deque[float] = deque(maxlen=n)
        self._pos_x: Deque[float] = deque(maxlen=n)
        self._pos_y: Deque[float] = deque(maxlen=n)
        self._pos_z: Deque[float] = deque(maxlen=n)
        self._tgt_x: Deque[float] = deque(maxlen=n)
        self._tgt_y: Deque[float] = deque(maxlen=n)
        self._tgt_z: Deque[float] = deque(maxlen=n)
        self._mode_val: Deque[float] = deque(maxlen=n)

        # ---- latest values (updated asynchronously by callbacks) ---- #
        self._lat_thrust = [0.0, 0.0, 0.0]
        self._lat_att = [0.0, 0.0, 0.0, 1.0]
        self._lat_f_meas = 0.0
        self._lat_f_des = 0.0
        self._lat_pos = [0.0, 0.0, 0.0]
        self._lat_tgt = [0.0, 0.0, 0.0]
        self._lat_mode = False

        # ---- subscriptions ---- #
        self.create_subscription(
            AttitudeThrust, args.topic, self._thrust_cb, 10)
        self.create_subscription(
            WrenchStamped, args.force_topic, self._force_cb, 10)
        self.create_subscription(
            WrenchStamped, args.force_setpoint_topic, self._setpoint_cb, 10)

        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            Odometry, args.odom_topic, self._odom_cb, odom_qos)
        self.create_subscription(
            Odometry, args.tracking_topic, self._tracking_cb, 10)
        self.create_subscription(
            Bool, args.switch_topic, self._switch_cb, 10)

        # ---- uniform sampling timer ---- #
        self.create_timer(1.0 / args.sample_hz, self._sample)

        self.get_logger().info(
            f"HybridControlMonitor: sample={args.sample_hz:.0f}Hz, "
            f"history={self._history_sec:.0f}s, max_points={n}")

    # ---------- callbacks (just store latest) ---------- #

    def _thrust_cb(self, msg: AttitudeThrust) -> None:
        self._lat_thrust = [
            float(msg.thrust.x), float(msg.thrust.y), float(msg.thrust.z)]
        self._lat_att = [
            float(msg.attitude.x), float(msg.attitude.y),
            float(msg.attitude.z), float(msg.attitude.w)]
        self._has_data = True

    def _force_cb(self, msg: WrenchStamped) -> None:
        self._lat_f_meas = float(msg.wrench.force.x)
        self._has_data = True

    def _setpoint_cb(self, msg: WrenchStamped) -> None:
        self._lat_f_des = float(msg.wrench.force.x)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._lat_pos = [float(p.x), float(p.y), float(p.z)]
        self._has_data = True

    def _tracking_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._lat_tgt = [float(p.x), float(p.y), float(p.z)]

    def _switch_cb(self, msg: Bool) -> None:
        self._lat_mode = bool(msg.data)

    # ---------- uniform sampling ---------- #

    def _sample(self) -> None:
        if not self._has_data:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        qx, qy, qz, qw = self._lat_att
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(sinp)
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            self._t.append(now - self._t0)
            self._tx.append(self._lat_thrust[0])
            self._ty.append(self._lat_thrust[1])
            self._tz.append(self._lat_thrust[2])
            self._pitch_deg.append(math.degrees(pitch))
            self._roll_deg.append(math.degrees(roll))
            self._f_meas_x.append(self._lat_f_meas)
            self._f_des_x.append(self._lat_f_des)
            self._pos_x.append(self._lat_pos[0])
            self._pos_y.append(self._lat_pos[1])
            self._pos_z.append(self._lat_pos[2])
            self._tgt_x.append(self._lat_tgt[0])
            self._tgt_y.append(self._lat_tgt[1])
            self._tgt_z.append(self._lat_tgt[2])
            self._mode_val.append(1.0 if self._lat_mode else 0.0)

    # ---------- snapshot for plotting ---------- #

    def snapshot(self) -> Dict[str, List[float]]:
        with self._lock:
            if not self._t:
                return {}
            raw: Dict[str, List[float]] = {
                "t": list(self._t),
                "tx": list(self._tx), "ty": list(self._ty), "tz": list(self._tz),
                "pitch": list(self._pitch_deg), "roll": list(self._roll_deg),
                "f_meas": list(self._f_meas_x), "f_des": list(self._f_des_x),
                "px": list(self._pos_x), "py": list(self._pos_y), "pz": list(self._pos_z),
                "gx": list(self._tgt_x), "gy": list(self._tgt_y), "gz": list(self._tgt_z),
                "mode": list(self._mode_val),
            }

        # windowing
        t = raw["t"]
        if t and self._history_sec > 0:
            t_min = t[-1] - self._history_sec
            start = 0
            for i, ti in enumerate(t):
                if ti >= t_min:
                    start = i
                    break
            if start > 0:
                for k in raw:
                    raw[k] = raw[k][start:]

        # derived signals
        raw["f_err"] = [d - m for d, m in zip(raw["f_des"], raw["f_meas"])]
        raw["x_err"] = [g - p for g, p in zip(raw["gx"], raw["px"])]
        raw["y_err"] = [g - p for g, p in zip(raw["gy"], raw["py"])]
        raw["z_err"] = [g - p for g, p in zip(raw["gz"], raw["pz"])]
        raw["mag"] = [
            math.sqrt(x * x + y * y + z * z)
            for x, y, z in zip(raw["tx"], raw["ty"], raw["tz"])]
        return raw


def main() -> None:
    args = _parse_args()
    plt, animation = _import_matplotlib()

    rclpy.init()
    node = HybridControlMonitor(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    fig, axes = plt.subplots(
        5, 1, figsize=(13, 12), sharex=True,
        gridspec_kw={"height_ratios": [1.0, 0.8, 0.8, 0.7, 0.6]})
    ax_force, ax_xpos, ax_thrust, ax_yz, ax_att = axes
    try:
        fig.canvas.manager.set_window_title("Hybrid Force/Position Control Monitor")
    except (AttributeError, TypeError):
        pass

    # ---- Panel 1: Force Fx tracking (force PI tuning) ---- #
    ln_f_des, = ax_force.plot([], [], color="#d97706", lw=1.6, label="Fx desired")
    ln_f_meas, = ax_force.plot([], [], color="#7c3aed", lw=1.5, label="Fx measured")
    ln_f_err, = ax_force.plot([], [], color="#dc2626", lw=1.2, alpha=0.85, label="Fx error")
    ax_force.axhline(0, color="#94a3b8", lw=0.7, ls="--", alpha=0.5)
    ax_force.set_ylabel("Force x (N)")
    ax_force.set_title("Hybrid Force/Position Control — Tuning Dashboard")
    ax_force.legend(loc="upper right", fontsize=8)
    ax_force.grid(True, alpha=0.25)

    status_text = ax_force.text(
        0.01, 0.97, "", transform=ax_force.transAxes, va="top", ha="left",
        fontsize=9, family="monospace",
        bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#cbd5e1", "alpha": 0.92})

    # ---- Panel 2: X position (x_hold PD tuning) ---- #
    ln_px, = ax_xpos.plot([], [], color="#2563eb", lw=1.5, label="current x")
    ln_gx, = ax_xpos.plot([], [], color="#ea580c", lw=1.4, ls="--", label="target x")
    ax_xpos_r = ax_xpos.twinx()
    ln_xerr, = ax_xpos_r.plot([], [], color="#dc2626", lw=1.1, alpha=0.8, label="x error")
    ax_xpos.set_ylabel("X position (m)")
    ax_xpos_r.set_ylabel("X error (m)", color="#dc2626")
    ax_xpos_r.tick_params(axis="y", labelcolor="#dc2626")
    h1, l1 = ax_xpos.get_legend_handles_labels()
    h2, l2 = ax_xpos_r.get_legend_handles_labels()
    ax_xpos.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8)
    ax_xpos.grid(True, alpha=0.25)

    # ---- Panel 3: Thrust output ---- #
    ln_tx, = ax_thrust.plot([], [], "r-", lw=1.6, label="thrust.x (wrench)")
    ln_ty, = ax_thrust.plot([], [], "g-", lw=1.0, alpha=0.7, label="thrust.y (pose)")
    ln_tz, = ax_thrust.plot([], [], "b-", lw=1.0, alpha=0.7, label="thrust.z (pose)")
    ln_mag = None
    if not args.no_magnitude:
        ln_mag, = ax_thrust.plot([], [], "k--", lw=0.9, alpha=0.7, label="||T||")
    ax_thrust.set_ylabel("Thrust")
    all_thrust_h = [ln_tx, ln_ty, ln_tz] + ([ln_mag] if ln_mag else [])
    ax_thrust.legend(all_thrust_h, [h.get_label() for h in all_thrust_h],
                     loc="upper right", fontsize=8)
    ax_thrust.grid(True, alpha=0.25)

    # ---- Panel 4: YZ position hold (pose controller) ---- #
    ln_yerr, = ax_yz.plot([], [], color="#059669", lw=1.4, label="y error")
    ln_zerr, = ax_yz.plot([], [], color="#2563eb", lw=1.4, label="z error")
    ax_yz.axhline(0, color="#94a3b8", lw=0.7, ls="--", alpha=0.5)
    ax_yz.set_ylabel("Position error (m)")
    ax_yz.legend(loc="upper right", fontsize=8)
    ax_yz.grid(True, alpha=0.25)

    # ---- Panel 5: Attitude ---- #
    ln_pitch, = ax_att.plot([], [], color="#0f766e", lw=1.4, label="pitch (deg)")
    ln_roll, = ax_att.plot([], [], color="#7c2d12", lw=1.1, alpha=0.75, label="roll (deg)")
    ax_att.axhline(0, color="#94a3b8", lw=0.7, ls="--", alpha=0.5)
    ax_att.set_xlabel("time (s)")
    ax_att.set_ylabel("Attitude (deg)")
    ax_att.legend(loc="upper right", fontsize=8)
    ax_att.grid(True, alpha=0.25)

    # ---- mode background shading state ---- #
    mode_spans: List[Any] = []

    def update(_frame: int) -> List:
        nonlocal mode_spans
        d = node.snapshot()
        if not d:
            return []

        t = d["t"]
        ln_f_des.set_data(t, d["f_des"])
        ln_f_meas.set_data(t, d["f_meas"])
        ln_f_err.set_data(t, d["f_err"])

        ln_px.set_data(t, d["px"])
        ln_gx.set_data(t, d["gx"])
        ln_xerr.set_data(t, d["x_err"])

        ln_tx.set_data(t, d["tx"])
        ln_ty.set_data(t, d["ty"])
        ln_tz.set_data(t, d["tz"])
        if ln_mag is not None:
            ln_mag.set_data(t, d["mag"])

        ln_yerr.set_data(t, d["y_err"])
        ln_zerr.set_data(t, d["z_err"])

        ln_pitch.set_data(t, d["pitch"])
        ln_roll.set_data(t, d["roll"])

        # mode shading: light red background when wrench active
        for sp in mode_spans:
            sp.remove()
        mode_spans.clear()
        mode = d["mode"]
        if mode and any(m > 0.5 for m in mode):
            i = 0
            while i < len(mode):
                if mode[i] > 0.5:
                    j = i
                    while j < len(mode) and mode[j] > 0.5:
                        j += 1
                    t_start = t[i]
                    t_end = t[min(j, len(t) - 1)]
                    for ax in axes:
                        sp = ax.axvspan(t_start, t_end,
                                        alpha=0.08, color="#ef4444", zorder=0)
                        mode_spans.append(sp)
                    i = j
                else:
                    i += 1

        # status text
        mode_str = "HYBRID (wrench x + pose yz)" if d["mode"][-1] > 0.5 else "POSE ONLY (xyz)"
        fx_m = d["f_meas"][-1]
        fx_d = d["f_des"][-1]
        fx_e = d["f_err"][-1]
        xe = d["x_err"][-1]
        ye = d["y_err"][-1]
        ze = d["z_err"][-1]
        thr_x = d["tx"][-1]
        status_text.set_text(
            f"MODE: {mode_str}\n"
            f"Fx: meas={fx_m:+.2f}  des={fx_d:+.2f}  err={fx_e:+.2f} N\n"
            f"Pos err: x={xe:+.3f}  y={ye:+.3f}  z={ze:+.3f} m\n"
            f"thrust.x={thr_x:+.4f}")

        for ax in axes:
            ax.relim()
            ax.autoscale_view()
        ax_xpos_r.relim()
        ax_xpos_r.autoscale_view()
        return []

    _ = animation.FuncAnimation(
        fig, update, interval=args.interval_ms,
        blit=False, cache_frame_data=False)

    plt.tight_layout()
    try:
        plt.show()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
