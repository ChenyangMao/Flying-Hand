#!/usr/bin/env python3
"""
Staged flight test script for hardware validation.

Runs progressively more complex flight tests to validate each subsystem
before attempting the full visual-servoing contact sequence.

Stages:
  1. pose_only  — Take off, hover at target altitude, land. Validates
                   PX4 interface, MAVROS, offboard mode, and pose controller.
  2. vs_track   — Take off, hover, enable visual servo, approach target
                   WITHOUT force control. Validates camera, detection, IBVS.
                   Stays at safe depth (no contact).
  3. light_contact — Full sequence with reduced force (desired_force=2.0 N).
                   Validates F/T sensor, wrench blend, contact detection.

Usage:
    python3 scripts/test_flight_stages.py --stage pose_only
    python3 scripts/test_flight_stages.py --stage vs_track
    python3 scripts/test_flight_stages.py --stage light_contact

SAFETY: Always run in a tethered or caged indoor environment first.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Staged flight tests for hardware validation"
    )
    parser.add_argument(
        "--stage",
        choices=["pose_only", "vs_track", "light_contact"],
        required=True,
        help="Which test stage to run.",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=1.2,
        help="Takeoff altitude in meters (default: 1.2).",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=10.0,
        help="How long to hold pose/force before landing (default: 10).",
    )
    args = parser.parse_args()

    rclpy.init()

    # Build parameter overrides based on stage
    overrides: list[Parameter] = [
        Parameter("takeoff_altitude", value=args.altitude),
        Parameter("hold_time", value=args.hold_sec),
        Parameter("force_hard_abort_limit", value=12.0),
    ]

    if args.stage == "pose_only":
        # Disable visual servo and wrench; just hover then land.
        overrides += [
            Parameter("visual_servo_timeout_sec", value=999.0),
            Parameter("visual_servo_lock_confirm_sec", value=999.0),
            Parameter("pre_approach_hold_sec", value=args.hold_sec),
            Parameter("desired_force", value=0.0),
            Parameter("approach_velocity", value=0.0),
        ]
        print(f"\n=== STAGE: POSE ONLY (hover at {args.altitude}m for {args.hold_sec}s) ===\n")

    elif args.stage == "vs_track":
        # Enable visual servo but keep depth_max very small so wrench never
        # engages (visual approach only, no contact).
        overrides += [
            Parameter("visual_servo_depth_max", value=0.10),
            Parameter("desired_force", value=0.0),
            Parameter("pre_approach_hold_sec", value=3.0),
            Parameter("approach_velocity", value=0.0),
            Parameter("max_hold_timeout", value=args.hold_sec),
        ]
        print(f"\n=== STAGE: VS TRACKING (approach without contact, {args.hold_sec}s) ===\n")

    elif args.stage == "light_contact":
        overrides += [
            Parameter("desired_force", value=2.0),
            Parameter("contact_force_max", value=6.0),
            Parameter("force_hard_abort_limit", value=10.0),
            Parameter("pre_approach_hold_sec", value=3.0),
        ]
        print(f"\n=== STAGE: LIGHT CONTACT (desired_force=2.0 N, {args.hold_sec}s) ===\n")

    # Import and instantiate the real WallContactTester with overridden params
    sys.path.insert(0, ".")
    from scripts.test_visual_servoing_ros2 import WallContactTester

    node = WallContactTester()

    # Apply parameter overrides
    for p in overrides:
        if node.has_parameter(p.name):
            node.set_parameters([p])
        else:
            node.get_logger().warn(f"Parameter '{p.name}' not declared, skipping.")

    # For pose_only: override the state machine to skip approach entirely.
    # The test will hover for pre_approach_hold_sec and then land.
    if args.stage == "pose_only":
        original_step_hover = node._step_hover_alt

        def _pose_only_hover():
            original_step_hover()
            # After hover completes, skip approach and go straight to land.
            from scripts.test_visual_servoing_ros2 import TestState
            if node.state == node.__class__.__dict__.get("APPROACH", None):
                pass
            # Check if approach state was entered by the original hover handler
            if hasattr(node, 'state'):
                from scripts.test_visual_servoing_ros2 import TestState
                if node.state == TestState.APPROACH:
                    node.get_logger().info(
                        "Pose-only test: skipping approach, landing."
                    )
                    node._land_t0 = None
                    node.state = TestState.LAND

        node._step_hover_alt = _pose_only_hover

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n=== Test complete ===\n")


if __name__ == "__main__":
    main()
