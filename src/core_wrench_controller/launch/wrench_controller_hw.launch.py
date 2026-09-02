#!/usr/bin/env python3
"""
Launch wrench_controller for real hardware with PX4 + MAVROS.

Loads the base wrench params then overlays hardware-specific tuning.
Publishes the real camera and FT sensor TFs (adjust arguments below to
match the actual mount on the drone).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wrench = get_package_share_directory('core_wrench_controller')
    pkg_px4_if = get_package_share_directory('core_px4_interface')
    px4_bridge_launch = os.path.join(pkg_px4_if, 'launch', 'px4_interface_node.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'include_px4_bridge',
            default_value='true',
            description='Start PX4Interface bridge for MAVROS attitude commands.',
        ),

        # ---------- Real camera TF ----------
        # ADJUST these values to match the actual camera mount on your drone.
        # Arguments: x y z yaw pitch roll (meters / radians)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '0.12', '0', '0.12',
                '-1.57079632679', '0', '-1.57079632679',
                'base_link', 'camera',
            ],
        ),

        # ---------- Real FT sensor TF ----------
        # ADJUST these values to match the actual force-torque sensor mount.
        # On hardware, the FT driver publishes with its own frame_id (e.g.
        # "ft_sensor"); this TF should place that frame relative to base_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_ft_sensor',
            arguments=[
                '0.15', '0', '-0.10',
                '0', '0', '0',
                'base_link', 'ft_sensor',
            ],
        ),

        # ---------- PX4 interface ----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(px4_bridge_launch),
            condition=IfCondition(LaunchConfiguration('include_px4_bridge')),
        ),

        # ---------- Wrench controller ----------
        Node(
            package='core_wrench_controller',
            executable='wrench_controller',
            name='wrench_controller',
            parameters=[
                os.path.join(pkg_wrench, 'config', 'wrench_px4_params.yaml'),
                os.path.join(pkg_wrench, 'config', 'wrench_px4_hw_params.yaml'),
            ],
            remappings=[
                ('odometry', 'mavros/local_position/odom'),
            ],
        ),
    ])
