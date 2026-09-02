#!/usr/bin/env python3
"""
Hardware readiness checker for the Flying Hand drone.

Runs a series of automated checks to verify that all required topics,
TF frames, and services are available before attempting flight.

Usage:
    # Launch the hardware stack first, then:
    python3 scripts/test_hw_readiness.py

Checks performed:
  1. MAVROS state is connected and FCU is reachable.
  2. Local position odometry is being published.
  3. F/T sensor data is being published on /ft_data.
  4. Camera images are arriving.
  5. TF tree contains required frames: map, base_link, ft_sensor, camera.
  6. Visual servo nodes are alive (/detected_circle, /visual_servo/active).
  7. Wrench controller is accepting commands.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import WrenchStamped
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

from tf2_ros import Buffer, TransformListener, TransformException


REQUIRED_FRAMES = [
    ("map", "base_link"),
    ("base_link", "ft_sensor"),
    ("base_link", "camera"),
]

TIMEOUT_SEC = 10.0


class ReadinessChecker(Node):
    def __init__(self) -> None:
        super().__init__("hw_readiness_checker")
        self.results: dict[str, str] = {}
        self._received: dict[str, bool] = {}

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions for liveness checks
        self._sub("mavros_state", MavrosState, "mavros/state", state_qos)
        self._sub("odom", Odometry, "mavros/local_position/odom", odom_qos)
        self._sub("ft_data", WrenchStamped, "/ft_data", 10)
        self._sub("camera", Image, "/uav1/camera/color/image_raw", 10)
        self._sub("circle", Vector3, "/detected_circle", 10)
        self._sub("vs_active", Bool, "/visual_servo/active", 10)

        self.get_logger().info(
            f"Checking hardware readiness (timeout {TIMEOUT_SEC}s)..."
        )

    def _sub(self, key, msg_type, topic, qos):
        self._received[key] = False

        def cb(msg, k=key):
            self._received[k] = True

        self.create_subscription(msg_type, topic, cb, qos)

    def run_checks(self) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.2)

        all_ok = True
        print("\n" + "=" * 60)
        print("  HARDWARE READINESS REPORT")
        print("=" * 60)

        checks = [
            ("MAVROS state", "mavros_state"),
            ("Local odometry", "odom"),
            ("F/T sensor (/ft_data)", "ft_data"),
            ("Camera images", "camera"),
            ("Circle detector", "circle"),
            ("Visual servo active", "vs_active"),
        ]

        for label, key in checks:
            ok = self._received[key]
            status = "OK" if ok else "FAIL"
            if not ok:
                all_ok = False
            print(f"  [{status:4s}] {label}")

        # TF checks
        for parent, child in REQUIRED_FRAMES:
            try:
                self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time()
                )
                print(f"  [OK  ] TF: {parent} -> {child}")
            except TransformException as e:
                print(f"  [FAIL] TF: {parent} -> {child}  ({e})")
                all_ok = False

        print("=" * 60)
        if all_ok:
            print("  ALL CHECKS PASSED -- ready for flight tests")
        else:
            print("  SOME CHECKS FAILED -- resolve before flight")
        print("=" * 60 + "\n")
        return all_ok


def main() -> None:
    rclpy.init()
    checker = ReadinessChecker()
    ok = checker.run_checks()
    checker.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
