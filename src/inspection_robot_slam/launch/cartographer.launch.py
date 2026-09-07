import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    configuration_directory = os.path.join(
        get_package_share_directory("inspection_robot_slam"),
        "config",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                )
            }
        ],
        arguments=[
            "-configuration_directory",
            configuration_directory,
            "-configuration_basename",
            "cartographer_s200.lua",
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
            ("odom", "/odometry/filtered"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                )
            }
        ],
        arguments=[
            "-resolution",
            "0.05",
            "-publish_period_sec",
            "1.0",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            cartographer_node,
            occupancy_grid_node,
        ]
    )