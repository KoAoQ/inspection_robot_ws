import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("inspection_robot_slam"),
        "config",
        "mapper.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_params,
                description="SLAM Toolbox parameter file",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
