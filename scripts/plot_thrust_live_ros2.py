#!/usr/bin/env python3
"""
实时绘制 wrench_controller 发布的 map 系期望推力 (AttitudeThrust.thrust)。

与 test_wall_contact_ros2.py 并行使用：
  终端1: ros2 launch ...   # wrench + drone_interface
  终端2: python3 scripts/test_wall_contact_ros2.py
  终端3: python3 scripts/plot_thrust_live_ros2.py

依赖: matplotlib（纯 Python 列表绘图，不依赖本脚本 import numpy）。
若遇 NumPy 2.x 与系统 matplotlib 二进制不兼容，见 main() 内提示。
"""
from __future__ import annotations

import argparse
import math
import threading
from collections import deque
from typing import Deque, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from mav_msgs.msg import AttitudeThrust


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--topic",
        default="attitude_thrust_command",
        help="mav_msgs/AttitudeThrust 话题（与 wrench_controller 发布一致）",
    )
    p.add_argument(
        "--force-topic",
        default="ft_data_filtered",
        help="实际感受到的力话题（geometry_msgs/WrenchStamped，默认取 force.x）",
    )
    p.add_argument(
        "--force-setpoint-topic",
        default="ft_setpoint",
        help="期望力话题（geometry_msgs/WrenchStamped，默认取 force.x）",
    )
    p.add_argument(
        "--history-sec",
        type=float,
        default=45.0,
        help="横轴滑动窗口长度（秒）",
    )
    p.add_argument(
        "--max-points",
        type=int,
        default=12000,
        help="最多保留的样本数（防止内存增长）",
    )
    p.add_argument(
        "--interval-ms",
        type=int,
        default=50,
        help="matplotlib 刷新间隔（毫秒）",
    )
    p.add_argument(
        "--no-magnitude",
        action="store_true",
        help="不绘制 ‖T‖ 曲线",
    )
    return p.parse_args()


def _import_matplotlib():
    """尽量在友好报错里说明 NumPy2 / 系统 matplotlib 冲突的常见修法。"""
    try:
        import matplotlib.pyplot as plt
        from matplotlib import animation

        return plt, animation
    except Exception as e:
        hint = (
            "\n常见原因：系统自带的 python3-matplotlib 按 NumPy 1.x 编译，"
            "而当前环境为 NumPy 2.x（_ARRAY_API / AttributeError）。\n"
            "任选其一修复后重试：\n"
            "  pip install --user --upgrade matplotlib\n"
            "  pip install --user 'numpy<2'\n"
            "  或使用 venv：python3 -m venv .venv && source .venv/bin/activate && pip install matplotlib numpy rclpy ...\n"
        )
        raise SystemExit(
            f"无法加载 matplotlib。{hint}\n原始错误: {type(e).__name__}: {e}"
        ) from e


class ThrustLivePlotNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("thrust_live_plotter")
        self._lock = threading.Lock()
        self._t0: Optional[float] = None
        n = args.max_points
        self._t: Deque[float] = deque(maxlen=n)
        self._tx: Deque[float] = deque(maxlen=n)
        self._ty: Deque[float] = deque(maxlen=n)
        self._tz: Deque[float] = deque(maxlen=n)
        self._f_meas_x: Deque[float] = deque(maxlen=n)
        self._f_des_x: Deque[float] = deque(maxlen=n)
        self._history_sec = args.history_sec

        self._thrust_sub = self.create_subscription(
            AttitudeThrust,
            args.topic,
            self._thrust_cb,
            10,
        )
        self._force_sub = self.create_subscription(
            WrenchStamped,
            args.force_topic,
            self._force_cb,
            10,
        )
        self._force_setpoint_sub = self.create_subscription(
            WrenchStamped,
            args.force_setpoint_topic,
            self._force_setpoint_cb,
            10,
        )
        self.get_logger().info(
            f"Subscribing AttitudeThrust on '{args.topic}' "
            f"(history={self._history_sec:.1f}s, max_points={n})"
        )
        self.get_logger().info(
            f"Subscribing desired force on '{args.force_setpoint_topic}' "
            f"and measured force on '{args.force_topic}' (using wrench.force.x)"
        )

    def _append_time_locked(self, now: float) -> None:
        if self._t0 is None:
            self._t0 = now
        self._t.append(now - self._t0)

    def _thrust_cb(self, msg: AttitudeThrust) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._append_time_locked(now)
            self._tx.append(float(msg.thrust.x))
            self._ty.append(float(msg.thrust.y))
            self._tz.append(float(msg.thrust.z))
            self._f_meas_x.append(self._f_meas_x[-1] if self._f_meas_x else 0.0)
            self._f_des_x.append(self._f_des_x[-1] if self._f_des_x else 0.0)

    def _force_cb(self, msg: WrenchStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._append_time_locked(now)
            self._tx.append(self._tx[-1] if self._tx else 0.0)
            self._ty.append(self._ty[-1] if self._ty else 0.0)
            self._tz.append(self._tz[-1] if self._tz else 0.0)
            self._f_meas_x.append(float(msg.wrench.force.x))
            self._f_des_x.append(self._f_des_x[-1] if self._f_des_x else 0.0)

    def _force_setpoint_cb(self, msg: WrenchStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._append_time_locked(now)
            self._tx.append(self._tx[-1] if self._tx else 0.0)
            self._ty.append(self._ty[-1] if self._ty else 0.0)
            self._tz.append(self._tz[-1] if self._tz else 0.0)
            self._f_meas_x.append(self._f_meas_x[-1] if self._f_meas_x else 0.0)
            self._f_des_x.append(float(msg.wrench.force.x))

    def snapshot(
        self,
    ) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float], List[float]]:
        with self._lock:
            if not self._t:
                return [], [], [], [], [], [], []
            t = list(self._t)
            tx = list(self._tx)
            ty = list(self._ty)
            tz = list(self._tz)
            f_meas_x = list(self._f_meas_x)
            f_des_x = list(self._f_des_x)
        mag = [
            math.sqrt(ax * ax + ay * ay + az * az) for ax, ay, az in zip(tx, ty, tz)
        ]
        if t and self._history_sec > 0:
            t_end = t[-1]
            t_min = t_end - self._history_sec
            t_f, tx_f, ty_f, tz_f, mag_f, f_meas_f, f_des_f = [], [], [], [], [], [], []
            for i, ti in enumerate(t):
                if ti >= t_min:
                    t_f.append(ti)
                    tx_f.append(tx[i])
                    ty_f.append(ty[i])
                    tz_f.append(tz[i])
                    mag_f.append(mag[i])
                    f_meas_f.append(f_meas_x[i])
                    f_des_f.append(f_des_x[i])
            return t_f, tx_f, ty_f, tz_f, mag_f, f_meas_f, f_des_f
        return t, tx, ty, tz, mag, f_meas_x, f_des_x


def main() -> None:
    args = _parse_args()
    plt, animation = _import_matplotlib()

    rclpy.init()
    node = ThrustLivePlotNode(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=executor.spin, daemon=True).start()

    fig, ax = plt.subplots(figsize=(11, 5.5))
    ax_force = ax.twinx()
    try:
        fig.canvas.manager.set_window_title("AttitudeThrust.thrust (live)")
    except (AttributeError, TypeError):
        pass
    (line_x,) = ax.plot([], [], "r-", lw=1.2, label="thrust.x")
    (line_y,) = ax.plot([], [], "g-", lw=1.2, label="thrust.y")
    (line_z,) = ax.plot([], [], "b-", lw=1.2, label="thrust.z")
    (line_f_des,) = ax_force.plot([], [], color="#d97706", lw=1.6, label="desired F.x")
    (line_f_meas,) = ax_force.plot([], [], color="#7c3aed", lw=1.4, alpha=0.9, label="measured F.x")
    line_m = None
    if not args.no_magnitude:
        (line_m,) = ax.plot([], [], "k--", lw=1.0, alpha=0.85, label="‖T‖")
    ax.set_xlabel("time since first message (s)")
    ax.set_ylabel("thrust components (controller units)")
    ax_force.set_ylabel("force x (N)")
    ax.set_title("wrench_controller → attitude_thrust_command (map-frame thrust vector)")
    ax.grid(True, alpha=0.3)
    lines = [line_x, line_y, line_z, line_f_des, line_f_meas]
    if line_m is not None:
        lines.append(line_m)
    ax.legend(lines, [line.get_label() for line in lines], loc="upper right")

    def update(_frame: int) -> List:
        t, tx, ty, tz, mag, f_meas_x, f_des_x = node.snapshot()
        if not t:
            out = [line_x, line_y, line_z, line_f_des, line_f_meas]
            if line_m is not None:
                out.append(line_m)
            return out

        line_x.set_data(t, tx)
        line_y.set_data(t, ty)
        line_z.set_data(t, tz)
        line_f_des.set_data(t, f_des_x)
        line_f_meas.set_data(t, f_meas_x)
        if line_m is not None:
            line_m.set_data(t, mag)

        ax.relim()
        ax.autoscale_view()
        ax_force.relim()
        ax_force.autoscale_view()
        out = [line_x, line_y, line_z, line_f_des, line_f_meas]
        if line_m is not None:
            out.append(line_m)
        return out

    _ = animation.FuncAnimation(
        fig,
        update,
        interval=args.interval_ms,
        blit=False,
        cache_frame_data=False,
    )

    plt.tight_layout()
    try:
        plt.show()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
