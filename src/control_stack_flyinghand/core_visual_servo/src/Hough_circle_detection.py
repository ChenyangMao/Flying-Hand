#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CircleDetector(Node):
    def __init__(self):
        super().__init__("hough_circle_detector")
        self.declare_parameter("image_topic", "/uav1/camera/color/image_raw")
        topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, topic, self.image_callback, 10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=100,
            param2=30,
            minRadius=1,
            maxRadius=50,
        )

        if circles is not None:
            circles = circles[0, :].astype(int)
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)

        cv2.imshow("Detected Circles", frame)
        cv2.waitKey(100)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
