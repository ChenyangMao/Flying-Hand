#!/usr/bin/env python3
"""
Bench-test for camera + target detection pipeline.

Subscribes to the detection output and the debug image, prints detection
rate and circle parameters so you can verify:
  - The camera is publishing images at the expected rate.
  - The color detector is finding the target reliably.
  - The detected circle center and radius are stable (EMA smoothing working).
  - Depth estimates are reasonable for the known target distance.

Usage:
    python3 scripts/test_camera_detection.py

Place the target at known distances (0.5, 1.0, 1.5 m) and check the
reported depth against the actual distance.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Vector3Stamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class CameraDetectionTester(Node):
    def __init__(self) -> None:
        super().__init__("camera_detection_tester")
        self._det_count = 0
        self._img_count = 0
        self._active = False
        self._last_depth = None
        self._last_circle = None

        self.create_subscription(
            Vector3, "/detected_circle", self._circle_cb, 10
        )
        self.create_subscription(
            Vector3Stamped, "/visual_servo/depth", self._depth_cb, 10
        )
        self.create_subscription(
            Bool, "/visual_servo/active", self._active_cb, 10
        )
        self.create_subscription(
            Image, "/visual_servo/debug_image", self._img_cb, 10
        )
        self.create_timer(2.0, self._summary)
        self.get_logger().info(
            "Camera detection tester started. "
            "Place the target in front of the camera."
        )

    def _circle_cb(self, msg: Vector3) -> None:
        self._det_count += 1
        self._last_circle = (msg.x, msg.y, msg.z)

    def _depth_cb(self, msg: Vector3Stamped) -> None:
        self._last_depth = msg.vector.z

    def _active_cb(self, msg: Bool) -> None:
        self._active = msg.data

    def _img_cb(self, msg: Image) -> None:
        self._img_count += 1

    def _summary(self) -> None:
        circle_str = "none"
        if self._last_circle is not None:
            u, v, r = self._last_circle
            circle_str = f"u={u:.0f} v={v:.0f} r={r:.0f}"
        depth_str = f"{self._last_depth:.3f} m" if self._last_depth else "n/a"
        self.get_logger().info(
            f"detections={self._det_count}  images={self._img_count}  "
            f"active={self._active}  circle=[{circle_str}]  "
            f"depth={depth_str}"
        )


def main() -> None:
    rclpy.init()
    node = CameraDetectionTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
