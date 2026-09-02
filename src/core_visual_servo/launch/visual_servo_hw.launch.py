#!/usr/bin/env python3
"""
Launch the visual-servo perception + IBVS control stack for real hardware.

Uses vs_exp.yaml (experiment/hardware parameters) and the real camera driver
topic. No Gazebo bridge, no GUI windows by default.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("core_visual_servo")
    params_file = os.path.join(pkg_share, "config", "vs_exp.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_debug_view",
                default_value="false",
                description="Start rqt_image_view on the visual servo debug image topic.",
            ),
            DeclareLaunchArgument(
                "debug_view_topic",
                default_value="/visual_servo/debug_image",
                description="Image topic displayed by rqt_image_view.",
            ),
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),

            # Circle detector (perception)
            Node(
                package="core_visual_servo",
                executable="color_circle_detection.py",
                name="circle_detector",
                output="screen",
                parameters=[params_file],
            ),

            # IBVS controller
            Node(
                package="core_visual_servo",
                executable="IBVS.py",
                name="visual_servo_controller",
                output="screen",
                parameters=[params_file],
            ),

            # Optional debug viewer (off by default on headless onboard computer)
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="visual_servo_debug_view",
                output="screen",
                arguments=[LaunchConfiguration("debug_view_topic")],
                condition=IfCondition(LaunchConfiguration("show_debug_view")),
            ),
        ]
    )
