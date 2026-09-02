#!/usr/bin/env python3
"""
Quick bench-test for the F/T sensor pipeline.

Subscribes to both raw (/ft_data) and filtered (/ft_data_filtered) topics
and prints live force/torque readings so you can verify the sensor is
working, the frame_id is correct, and the filter is behaving well.

Usage:
    python3 scripts/test_ft_sensor.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class FtSensorTester(Node):
    def __init__(self) -> None:
        super().__init__("ft_sensor_tester")
        self._count_raw = 0
        self._count_filtered = 0

        self.create_subscription(WrenchStamped, "/ft_data", self._raw_cb, 10)
        self.create_subscription(
            WrenchStamped, "ft_data_filtered", self._filtered_cb, 10
        )
        self.create_timer(1.0, self._summary)
        self.get_logger().info(
            "FT sensor tester started. Apply known forces and verify readings."
        )

    def _raw_cb(self, msg: WrenchStamped) -> None:
        self._count_raw += 1
        f = msg.wrench.force
        t = msg.wrench.torque
        self.get_logger().info(
            f"[RAW  ] frame={msg.header.frame_id}  "
            f"F=({f.x:+7.3f}, {f.y:+7.3f}, {f.z:+7.3f})  "
            f"T=({t.x:+7.3f}, {t.y:+7.3f}, {t.z:+7.3f})",
            throttle_duration_sec=0.2,
        )

    def _filtered_cb(self, msg: WrenchStamped) -> None:
        self._count_filtered += 1
        f = msg.wrench.force
        self.get_logger().info(
            f"[FILT ] frame={msg.header.frame_id}  "
            f"F=({f.x:+7.3f}, {f.y:+7.3f}, {f.z:+7.3f})",
            throttle_duration_sec=0.2,
        )

    def _summary(self) -> None:
        self.get_logger().info(
            f"Messages received: raw={self._count_raw}, filtered={self._count_filtered}"
        )


def main() -> None:
    rclpy.init()
    node = FtSensorTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
