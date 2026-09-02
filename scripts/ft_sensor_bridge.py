#!/usr/bin/env python3
"""
Hardware F/T sensor bridge for the Flying Hand drone.

Subscribes to the raw force-torque topic published by the hardware sensor
driver (e.g. ATI Nano, OnRobot HEX, or similar) and republishes as a
geometry_msgs/WrenchStamped on /ft_data with the correct frame_id for
the wrench controller.

If the hardware driver already publishes geometry_msgs/WrenchStamped on
a different topic, this node simply remaps and optionally re-stamps.  If
the driver uses a different message type, extend the subscriber below.

Usage:
    python3 scripts/ft_sensor_bridge.py

    # Override the input topic:
    python3 scripts/ft_sensor_bridge.py --input-topic /ati_sensor/ft_data

    # Override the output frame_id:
    python3 scripts/ft_sensor_bridge.py --frame-id my_ft_frame
"""

import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class FtSensorBridge(Node):
    """Re-publish a hardware FT sensor topic as /ft_data with correct frame_id."""

    def __init__(self, input_topic: str, output_topic: str, frame_id: str) -> None:
        super().__init__("ft_sensor_bridge")
        self.frame_id = frame_id
        self.pub = self.create_publisher(WrenchStamped, output_topic, 10)
        self.sub = self.create_subscription(
            WrenchStamped, input_topic, self._cb, 10
        )
        self.get_logger().info(
            f"FT bridge: '{input_topic}' -> '{output_topic}' "
            f"(frame_id='{frame_id}')"
        )

    def _cb(self, msg: WrenchStamped) -> None:
        out = WrenchStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.wrench = msg.wrench
        self.pub.publish(out)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hardware FT sensor -> /ft_data bridge"
    )
    parser.add_argument(
        "--input-topic",
        type=str,
        default="/ft_sensor/wrench",
        help="Raw FT topic from the hardware driver (default: /ft_sensor/wrench)",
    )
    parser.add_argument(
        "--output-topic",
        type=str,
        default="/ft_data",
        help="ROS 2 output topic for the wrench controller (default: /ft_data)",
    )
    parser.add_argument(
        "--frame-id",
        type=str,
        default="ft_sensor",
        help="frame_id for WrenchStamped header (must match the TF tree, default: ft_sensor)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = FtSensorBridge(args.input_topic, args.output_topic, args.frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
