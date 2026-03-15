#!/usr/bin/env python3
"""Launch drone_interface_node with configurable plugin (default: GazeboInterface)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_interface',
            default_value='GazeboInterface',
            description='DroneInterface plugin name (e.g. PX4Interface, GazeboInterface)',
        ),
        Node(
            package='core_drone_interface',
            executable='drone_interface_node',
            name='drone_interface_node',
            parameters=[
                {'execute_target': 20.0},
                {'drone_interface': LaunchConfiguration('drone_interface')},
            ],
        ),
    ])
