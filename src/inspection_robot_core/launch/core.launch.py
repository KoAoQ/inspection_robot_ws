from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    p = os.path.join(get_package_share_directory("inspection_robot_core"), "config", "core.yaml")
    return LaunchDescription(
        [
            Node(
                package="inspection_robot_core",
                executable="velocity_manager_node",
                parameters=[p],
                output="screen",
            )
        ]
    )
