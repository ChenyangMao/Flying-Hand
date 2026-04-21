#!/usr/bin/env python3
"""
Publish simulated force-sensor data as geometry_msgs/WrenchStamped.

Default behavior:
  - publish to /ft_data
  - frame_id = ft_sensor
  - force.x fluctuates around 5 N with high-frequency noise (about ±3–4 N)
  - other force/torque axes stay near 0 with small noise

Example:
  python3 scripts/sim_ft_sensor.py
  python3 scripts/sim_ft_sensor.py --mean-x 5.0 --rate 100 --noise-std 0.08
"""

from __future__ import annotations

import argparse
import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/ft_data",
                        help="ROS 2 topic to publish (default: /ft_data)")
    parser.add_argument("--frame-id", default="ft_sensor",
                        help="WrenchStamped frame_id (default: ft_sensor)")
    parser.add_argument("--rate", type=float, default=250.0,
                        help="Publish rate in Hz (default: 250)")
    parser.add_argument("--mean-x", type=float, default=5.0,
                        help="Mean force on x axis in N (default: 5.0)")
    parser.add_argument("--hf-amp", type=float, default=3.5,
                        help="High-frequency noise amplitude on force.x in N "
                             "(uniform in [-amp, +amp], default: 3.5)")
    parser.add_argument("--hf-freq", type=float, default=25.0,
                        help="High-frequency component frequency in Hz "
                             "(sine wave, default: 25.0)")
    parser.add_argument("--sine-amp", type=float, default=0.0,
                        help="Main oscillation amplitude in N (default: 0.35)")
    parser.add_argument("--sine-freq", type=float, default=0.6,
                        help="Main oscillation frequency in Hz (default: 0.6)")
    parser.add_argument("--drift-amp", type=float, default=0.0,
                        help="Slow drift amplitude in N (default: 0.12)")
    parser.add_argument("--drift-freq", type=float, default=0.11,
                        help="Slow drift frequency in Hz (default: 0.11)")
    parser.add_argument("--noise-std", type=float, default=0.15,
                        help="Gaussian noise std-dev in N (default: 0.10)")
    parser.add_argument("--y-noise-std", type=float, default=0.03,
                        help="Noise std-dev for force.y in N (default: 0.03)")
    parser.add_argument("--z-noise-std", type=float, default=0.03,
                        help="Noise std-dev for force.z in N (default: 0.03)")
    parser.add_argument("--torque-noise-std", type=float, default=0.01,
                        help="Noise std-dev for each torque axis (default: 0.01)")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for repeatable output")
    return parser.parse_args()


class SimFtSensor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sim_ft_sensor")
        self._args = args
        self._publisher = self.create_publisher(WrenchStamped, args.topic, 10)
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._rng = random.Random(args.seed)
        self._hf_phase = self._rng.uniform(0.0, 2.0 * math.pi)

        period = 1.0 / max(args.rate, 1e-3)
        self.create_timer(period, self._publish_sample)

        self.get_logger().info(
            f"Publishing simulated FT data to '{args.topic}' at {args.rate:.1f} Hz "
            f"(frame_id='{args.frame_id}', mean_x={args.mean_x:.2f} N)",
        )

    def _noise(self, std: float) -> float:
        if std <= 0.0:
            return 0.0
        return self._rng.gauss(0.0, std)

    def _publish_sample(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self._t0
        args = self._args

        # High-frequency component: deterministic sine + a small random "jitter" per sample
        hf_wave = (
            float(args.hf_amp)
            * math.sin(2.0 * math.pi * float(args.hf_freq) * t + self._hf_phase)
            if args.hf_amp and args.hf_freq
            else 0.0
        )
        hf_jitter = self._rng.uniform(-0.25, 0.25) * abs(args.hf_amp) if args.hf_amp else 0.0
        fx = (
            args.mean_x +
            hf_wave +
            hf_jitter +
            args.sine_amp * math.sin(2.0 * math.pi * args.sine_freq * t) +
            args.drift_amp * math.sin(2.0 * math.pi * args.drift_freq * t + 0.7) +
            self._noise(args.noise_std)
        )

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = args.frame_id
        msg.wrench.force.x = fx
        msg.wrench.force.y = self._noise(args.y_noise_std)
        msg.wrench.force.z = self._noise(args.z_noise_std)
        msg.wrench.torque.x = self._noise(args.torque_noise_std)
        msg.wrench.torque.y = self._noise(args.torque_noise_std)
        msg.wrench.torque.z = self._noise(args.torque_noise_std)
        self._publisher.publish(msg)


def main() -> None:
    args = parse_args()
    rclpy.init()
    node: Optional[SimFtSensor] = None
    try:
        node = SimFtSensor(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
