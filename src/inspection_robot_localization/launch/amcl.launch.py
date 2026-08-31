import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("inspection_robot_localization"),
        "config",
        "amcl.yaml",
    )
    default_map_file = os.path.join(
        get_package_share_directory("inspection_robot_maps"),
        "maps",
        "inspection_area_clean.yaml",
    )

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description="Full path to the map YAML file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="AMCL parameter file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),

            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": map_file,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),

            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),

            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "amcl",
                        ],
                    }
                ],
            ),
        ]
    )