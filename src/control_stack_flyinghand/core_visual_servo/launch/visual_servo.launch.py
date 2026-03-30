from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("core_visual_servo")
    params_file = os.path.join(pkg_share, "config", "vs_sim_redtarget.yaml")

    return LaunchDescription(
        [
            Node(
                package="core_visual_servo",
                executable="color_circle_detection.py",
                name="circle_detector",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="core_visual_servo",
                executable="IBVS.py",
                name="visual_servo_controller",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
