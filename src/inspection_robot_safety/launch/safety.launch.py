from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    p = os.path.join(
        get_package_share_directory("inspection_robot_safety"), "config", "safety.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="inspection_robot_safety",
                executable="safety_node",
                parameters=[p],
                output="screen",
            )
        ]
    )
