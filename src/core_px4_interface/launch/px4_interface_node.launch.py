#!/usr/bin/env python3
"""Launch drone_interface_node with PX4Interface plugin (attitude_thrust_command -> MAVROS)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='core_drone_interface',
            executable='drone_interface_node',
            name='drone_interface_node',
            parameters=[
                {'execute_target': 20.0},
                {'drone_interface': 'PX4Interface'},
                {'setpoint_rate': 50.0},
            ],
        ),
    ])
