#!/usr/bin/env python3
# Launch wrench_controller with Gazebo simulation parameters and static TF for sim.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_wrench = get_package_share_directory('core_wrench_controller')
    return LaunchDescription([
        # Static TF used in sim: ft_sensor -> sim_ft_sensor (x y z yaw pitch roll).
        # Keep identity rotation so axes map directly: x->x, y->y, z->z.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_to_sim',
            arguments=['0', '0', '0', '0', '0', '0', 'ft_sensor', 'sim_ft_sensor'],
        ),
        # map -> contact (required by combine_motion_and_force, ref: control_stack_base)
        # x y z yaw pitch roll: yaw=pi for wall facing -X direction
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_contact',
            arguments=['0', '0', '0', '3.14159265359', '0', '0', 'map', 'contact'],
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
