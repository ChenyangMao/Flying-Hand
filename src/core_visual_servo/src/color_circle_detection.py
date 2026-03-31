#!/usr/bin/env python3
# pure perception node
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge


class ColorCircleDetector(Node):
    def __init__(self):
        super().__init__("circle_detector")

        self.declare_parameter("image_topic", "/uav1/camera/color/image_raw")
        self.declare_parameter("circle_topic", "/detected_circle")
        self.declare_parameter("debug_image_topic", "/visual_servo/debug_image")
        self.declare_parameter("show_window", True)
        self.declare_parameter("desired_color", "red")
        self.declare_parameter("resize_scale", 1.0)
        self.declare_parameter("min_radius_px", 20.0)
        self.declare_parameter("red_min_saturation", 100)
        self.declare_parameter("red_min_value", 60)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.circle_topic = self.get_parameter("circle_topic").get_parameter_value().string_value
        self.debug_image_topic = (
            self.get_parameter("debug_image_topic").get_parameter_value().string_value
        )
        self.show_window = self.get_parameter("show_window").get_parameter_value().bool_value
        self.resize_scale = float(self.get_parameter("resize_scale").value)
        self.min_radius_px = float(self.get_parameter("min_radius_px").value)
        self.red_min_saturation = int(self.get_parameter("red_min_saturation").value)
        self.red_min_value = int(self.get_parameter("red_min_value").value)
        self.desired_color = (
            self.get_parameter("desired_color").get_parameter_value().string_value or "red"
        ).lower()

        self.get_logger().info(f"Detecting color: {self.desired_color}")
        self.get_logger().info(f"show_window: {self.show_window}")
        self.get_logger().info(
            f"image_topic: {self.image_topic}, resize_scale: {self.resize_scale}, "
            f"min_radius_px: {self.min_radius_px}"
        )
        self.get_logger().info(
            f"red threshold: S>={self.red_min_saturation}, V>={self.red_min_value}"
        )
        self.get_logger().info(f"debug_image_topic: {self.debug_image_topic}")

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.circle_pub = self.create_publisher(Vector3, self.circle_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.window_enabled = self.show_window
        self._received_first_image = False

        if self.window_enabled:
            try:
                cv2.namedWindow("Detected Circles", cv2.WINDOW_NORMAL)
            except cv2.error as exc:
                self.window_enabled = False
                self.get_logger().warning(
                    f"OpenCV display window is unavailable, disabling show_window: {exc}"
                )

    def get_color_mask(self, hsv):
        """Return mask for the desired color in HSV space."""
        if self.desired_color == "red":
            lower_red1 = np.array([0, self.red_min_saturation, self.red_min_value])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, self.red_min_saturation, self.red_min_value])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

        elif self.desired_color == "blue":
            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

        else:
            self.get_logger().warning(
                f"Unknown color '{self.desired_color}', mask will be empty"
            )
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        return cv2.medianBlur(mask, 5)

    def image_callback(self, msg):
        if not self._received_first_image:
            self._received_first_image = True
            self.get_logger().info(f"First image received on {self.image_topic}")

        scale = max(0.1, min(self.resize_scale, 1.0))
        frame_full = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame_full.shape[:2]
        frame = cv2.resize(frame_full, (int(w * scale), int(h * scale)))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = self.get_color_mask(hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_circle = None
        best_score = -1.0
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if radius > self.min_radius_px:
                score = float(cv2.contourArea(cnt))
                if score > best_score:
                    best_score = score
                    best_circle = (x, y, radius)

        if best_circle is not None:
            x, y, radius = best_circle
            center = (int(x / scale), int(y / scale))
            radius_int = int(radius / scale)
            out = Vector3()
            out.x = float(center[0])
            out.y = float(center[1])
            out.z = float(radius_int)
            self.circle_pub.publish(out)
            cv2.circle(frame_full, center, radius_int, (0, 255, 0), 2)
            cv2.circle(frame_full, center, 2, (0, 0, 255), 3)
            cv2.putText(
                frame_full,
                f"circle: u={center[0]} v={center[1]} r={radius_int}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                frame_full,
                "circle: none",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(frame_full, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

        if self.window_enabled:
            try:
                cv2.imshow("Detected Circles", frame_full)
                cv2.waitKey(1)
            except cv2.error as exc:
                self.window_enabled = False
                self.get_logger().warning(
                    f"OpenCV display failed during update, disabling show_window: {exc}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ColorCircleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
