#!/usr/bin/env python3
"""
Minimal bridge: streams Gazebo forcetorque sensor data into a ROS 2
geometry_msgs/WrenchStamped topic.

Usage:
    # Auto-discover the gz forcetorque topic:
    python3 scripts/gz_ft_bridge.py

    # Or specify the gz topic explicitly:
    python3 scripts/gz_ft_bridge.py --gz-topic /world/default/model/.../forcetorque

    # Change the ROS 2 output topic (default: /ft_data):
    python3 scripts/gz_ft_bridge.py --ros-topic /ft_data
"""

import argparse
import subprocess
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


def discover_ft_topic() -> str | None:
    """Auto-discover the forcetorque gz transport topic."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.splitlines():
            if "forcetorque" in line.lower():
                return line.strip()
    except Exception:
        pass
    return None


class GzFtBridge(Node):
    """Reads gz transport forcetorque data via subprocess and publishes WrenchStamped."""

    def __init__(self, gz_topic: str, ros_topic: str, frame_id: str) -> None:
        super().__init__("gz_ft_bridge")
        self.pub = self.create_publisher(WrenchStamped, ros_topic, 10)
        self.frame_id = frame_id
        self.gz_topic = gz_topic

        self._force = [0.0, 0.0, 0.0]
        self._torque = [0.0, 0.0, 0.0]

        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", gz_topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()

        self._timer = self.create_timer(0.01, self._publish)  # 100 Hz
        self.get_logger().info(
            f"Bridging gz topic '{gz_topic}' -> ROS 2 '{ros_topic}' "
            f"(frame_id='{frame_id}', 100 Hz)"
        )

    def _reader(self) -> None:
        """Parse the text-protobuf output of `gz topic -e`."""
        section = None
        idx_map = {"x": 0, "y": 1, "z": 2}
        for line in iter(self._proc.stdout.readline, ""):
            s = line.strip()
            if s.startswith("force"):
                section = "f"
            elif s.startswith("torque"):
                section = "t"
            elif s == "}":
                section = None
            elif section and ":" in s:
                key, _, val = s.partition(":")
                key = key.strip()
                if key in idx_map:
                    try:
                        v = float(val.strip())
                        if section == "f":
                            self._force[idx_map[key]] = v
                        else:
                            self._torque[idx_map[key]] = v
                    except ValueError:
                        pass

    def _publish(self) -> None:
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = self._force[0]
        msg.wrench.force.y = self._force[1]
        msg.wrench.force.z = self._force[2]
        msg.wrench.torque.x = self._torque[0]
        msg.wrench.torque.y = self._torque[1]
        msg.wrench.torque.z = self._torque[2]
        self.pub.publish(msg)

    def destroy_node(self) -> None:
        if self._proc:
            self._proc.terminate()
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="Gazebo FT -> ROS 2 WrenchStamped bridge")
    parser.add_argument("--gz-topic", type=str, default=None,
                        help="Gazebo transport forcetorque topic (auto-discovered if omitted)")
    parser.add_argument("--ros-topic", type=str, default="/ft_data",
                        help="ROS 2 output topic (default: /ft_data)")
    parser.add_argument("--frame-id", type=str, default="sim_ft_sensor",
                        help="frame_id for WrenchStamped header (default: sim_ft_sensor)")
    args = parser.parse_args()

    gz_topic = args.gz_topic
    if gz_topic is None:
        gz_topic = discover_ft_topic()
        if gz_topic is None:
            print("ERROR: Could not auto-discover a forcetorque gz topic. "
                  "Is the Gazebo simulation running? Use --gz-topic to specify manually.")
            return

    rclpy.init()
    node = GzFtBridge(gz_topic, args.ros_topic, args.frame_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
