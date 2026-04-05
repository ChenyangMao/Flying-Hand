#!/usr/bin/env python3
"""UDP text Fz (N) -> geometry_msgs/WrenchStamped on ft_data (replaces scripts/sim_ft_sensor for hardware).

Expects newline-terminated UTF-8 datagrams parsable as a single float (same as the prior Float64 bridge).
Other wrench components are set to zero; use ``force_axis`` to place the scalar on x/y/z in the message
(physical Fz is usually ``z``; the sim historically used ``x`` for Fx-style tuning).
"""

from __future__ import annotations

import socket
from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


class UdpFzBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("udp_fz_bridge")

        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("output_topic", "ft_data")
        self.declare_parameter("frame_id", "ft_sensor")
        self.declare_parameter("force_axis", "z")
        self.declare_parameter("force_sign", 1.0)
        self.declare_parameter("poll_period_sec", 0.002)

        port = int(self.get_parameter("udp_port").value)
        topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        axis = str(self.get_parameter("force_axis").value).lower().strip()
        if axis not in ("x", "y", "z"):
            raise ValueError(f"force_axis must be x, y, or z; got {axis!r}")
        self._axis = axis
        self._force_sign = float(self.get_parameter("force_sign").value)
        poll = float(self.get_parameter("poll_period_sec").value)
        poll = max(poll, 0.0005)

        self._pub = self.create_publisher(WrenchStamped, topic, 10)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.setblocking(False)

        self.create_timer(poll, self._poll_udp)

        self.get_logger().info(
            f"UDP :{port} -> WrenchStamped '{topic}' "
            f"(frame_id={self._frame_id!r}, scalar -> force.{self._axis}, "
            f"force_sign={self._force_sign})"
        )

    def _poll_udp(self) -> None:
        while rclpy.ok():
            try:
                data, addr = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as e:
                self.get_logger().error(f"recv error: {e}")
                break
            try:
                text = data.decode("utf-8").strip()
                first_line = text.splitlines()[0] if text else ""
                fz = float(first_line) * self._force_sign
            except (ValueError, UnicodeDecodeError):
                self.get_logger().warning("bad datagram from %s: %r" % (addr, data[:64]))
                continue

            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            if self._axis == "x":
                msg.wrench.force.x = fz
            elif self._axis == "y":
                msg.wrench.force.y = fz
            else:
                msg.wrench.force.z = fz
            self._pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._sock.close()
        except OSError:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[UdpFzBridgeNode] = None
    try:
        node = UdpFzBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
