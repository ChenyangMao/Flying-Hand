from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="core_visual_servo",
                executable="tracking_point_sender.py",
                name="tracking_point_sender",
                output="screen",
            ),
        ]
    )
