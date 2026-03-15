#!/usr/bin/env python3
# Launch pose_controller with Gazebo simulation parameters (base + gazebo overrides).

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('core_pose_controller')
    return LaunchDescription([
        Node(
            package='core_pose_controller',
            executable='pose_controller',
            name='pose_controller',
            parameters=[
                os.path.join(pkg_dir, 'config', 'pose_px4_params.yaml'),
                os.path.join(pkg_dir, 'config', 'pose_px4_gazebo_params.yaml'),
            ],
        ),
    ])
