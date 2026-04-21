#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass
class _Vec3:
    x: float
    y: float
    z: float


class WrenchLowPassFilter(Node):
    def __init__(self) -> None:
        super().__init__("ft_wrench_filter")

        self.declare_parameter("input_topic", "ft_data")
        self.declare_parameter("output_topic", "ft_data_filtered")
        self.declare_parameter("alpha", 0.2)  # 0..1 ; higher = less smoothing
        self.declare_parameter("frame_id", "")  # if empty, keep input frame_id
        self.declare_parameter("filter_torque", False)

        self._in_topic = str(self.get_parameter("input_topic").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._alpha = float(self.get_parameter("alpha").value)
        self._alpha = _clamp(self._alpha, 0.0, 1.0)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._filter_torque = bool(self.get_parameter("filter_torque").value)

        self._pub = self.create_publisher(WrenchStamped, self._out_topic, 10)
        self._sub = self.create_subscription(WrenchStamped, self._in_topic, self._cb, 10)

        self._f: Optional[_Vec3] = None
        self._t: Optional[_Vec3] = None

        self.get_logger().info(
            f"Filtering WrenchStamped '{self._in_topic}' -> '{self._out_topic}' "
            f"(alpha={self._alpha:.3f}, filter_torque={self._filter_torque})"
        )

    def _lpf(self, prev: float, x: float) -> float:
        a = self._alpha
        return a * x + (1.0 - a) * prev

    def _cb(self, msg: WrenchStamped) -> None:
        fx = float(msg.wrench.force.x)
        fy = float(msg.wrench.force.y)
        fz = float(msg.wrench.force.z)

        if self._f is None:
            self._f = _Vec3(fx, fy, fz)
        else:
            self._f.x = self._lpf(self._f.x, fx)
            self._f.y = self._lpf(self._f.y, fy)
            self._f.z = self._lpf(self._f.z, fz)

        tx = float(msg.wrench.torque.x)
        ty = float(msg.wrench.torque.y)
        tz = float(msg.wrench.torque.z)

        if self._filter_torque:
            if self._t is None:
                self._t = _Vec3(tx, ty, tz)
            else:
                self._t.x = self._lpf(self._t.x, tx)
                self._t.y = self._lpf(self._t.y, ty)
                self._t.z = self._lpf(self._t.z, tz)

        out = WrenchStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id if self._frame_id else msg.header.frame_id
        out.wrench.force.x = self._f.x
        out.wrench.force.y = self._f.y
        out.wrench.force.z = self._f.z

        if self._filter_torque and self._t is not None:
            out.wrench.torque.x = self._t.x
            out.wrench.torque.y = self._t.y
            out.wrench.torque.z = self._t.z
        else:
            out.wrench.torque.x = tx
            out.wrench.torque.y = ty
            out.wrench.torque.z = tz

        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node: Optional[WrenchLowPassFilter] = None
    try:
        node = WrenchLowPassFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

