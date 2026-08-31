from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(package),
                    "launch",
                    filename,
                ]
            )
        ),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    enable_camera_color = LaunchConfiguration("enable_camera_color")

    platform = GroupAction(
        scoped=True,
        actions=[
            package_launch(
                "inspection_robot_bringup",
                "robot.launch.py",
                {
                    "enable_localization": "false",
                    "enable_slam": "false",
                    "enable_amcl": "true",
                    "enable_camera": "true",
                    "enable_camera_color": enable_camera_color,
                    "use_sim_time": use_sim_time,
                    "map": map_file,
                    "params_file": amcl_params_file,
                },
            )
        ],
    )

    navigation = GroupAction(
        scoped=True,
        actions=[
            package_launch(
                "inspection_robot_navigation",
                "navigation.launch.py",
                {
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                },
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("inspection_robot_maps"),
                        "maps",
                        "inspection_area_clean.yaml",
                    ]
                ),
                description="Full path to the occupancy map YAML file",
            ),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("inspection_robot_localization"),
                        "config",
                        "amcl.yaml",
                    ]
                ),
                description="Full path to the AMCL parameter file",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("inspection_robot_navigation"),
                        "config",
                        "nav2.yaml",
                    ]
                ),
                description="Full path to the Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically configure and activate Nav2 nodes",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="false",
                description="Respawn a Nav2 process if it exits unexpectedly",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Nav2 logging level",
            ),
            DeclareLaunchArgument(
                "enable_camera_color",
                default_value="false",
                description="Publish color images in navigation mode",
            ),
            platform,
            navigation,
        ]
    )
