from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("inspection_robot_bringup"),
                    "launch",
                    "robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "enable_localization": "false",
            "enable_slam": "true",
            "enable_amcl": "false",
        }.items(),
    )

    return LaunchDescription([robot_launch])
