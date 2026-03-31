#!/usr/bin/env python3
# Launch wrench_controller with Gazebo simulation parameters, static TF for sim, and (by default)
# the PX4 MAVROS bridge so AttitudeThrust reaches mavros/setpoint_raw/attitude when in force-control mode.

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
            description=(
                'If true, start drone_interface_node with PX4Interface so '
                'attitude_thrust_command is subscribed and republished to '
                'mavros/setpoint_raw/attitude for PX4 OFFBOARD.'
            ),
        ),
        # Static TF used in sim: ft_sensor -> sim_ft_sensor (x y z yaw pitch roll).
        # Keep identity rotation so axes map directly: x->x, y->y, z->z.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_to_sim',
            arguments=['0', '0', '0', '0', '0', '0', 'ft_sensor', 'sim_ft_sensor'],
        ),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(px4_bridge_launch),
            condition=IfCondition(LaunchConfiguration('include_px4_bridge')),
        ),
        Node(
            package='core_wrench_controller',
            executable='wrench_controller',
            name='wrench_controller',
            parameters=[
                os.path.join(pkg_wrench, 'config', 'wrench_px4_params.yaml'),
                os.path.join(pkg_wrench, 'config', 'wrench_px4_gazebo_params.yaml'),
            ],
            remappings=[
                ('odometry', 'mavros/local_position/odom'),
            ],
        ),
    ])
