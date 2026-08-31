import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("inspection_robot_navigation"),
        "rviz",
        "navigation.rviz",
    )

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_config,
                description="Full path to the navigation RViz configuration",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="inspection_robot_navigation_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
