from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def include(pkg, file, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", file])
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    localization = LaunchConfiguration("enable_localization")
    slam = LaunchConfiguration("enable_slam")
    amcl = LaunchConfiguration("enable_amcl")
    camera = LaunchConfiguration("enable_camera")
    camera_color = LaunchConfiguration("enable_camera_color")

    # EKF supplies odom -> base_footprint for standalone odometry, SLAM and AMCL.
    ekf_required = IfCondition(
        PythonExpression(
            [
                "'",
                localization,
                "'.lower() == 'true' or '",
                slam,
                "'.lower() == 'true' or '",
                amcl,
                "'.lower() == 'true'",
            ]
        )
    )

    # SLAM and AMCL both own map -> odom. If both are requested, SLAM wins.
    amcl_enabled = IfCondition(
        PythonExpression(
            [
                "'",
                amcl,
                "'.lower() == 'true' and '",
                slam,
                "'.lower() != 'true'",
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_localization",
                default_value="false",
                description="Start EKF explicitly; SLAM and AMCL start it automatically",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Start SLAM Toolbox mapping",
            ),
            DeclareLaunchArgument(
                "enable_amcl",
                default_value="true",
                description="Start map server and AMCL localization",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description="Start Astra depth camera and navigation cloud filter",
            ),
            DeclareLaunchArgument(
                "enable_camera_color",
                default_value="false",
                description="Publish color images while navigation camera is enabled",
            ),
            include("inspection_robot_description", "description.launch.py"),
            include("inspection_robot_lidar", "lidar.launch.py"),
            include(
                "inspection_robot_camera",
                "camera.launch.py",
                IfCondition(camera),
                {
                    "enable_color": camera_color,
                    "enable_ir": "false",
                    "enable_point_cloud": "true",
                    "enable_navigation_filter": "true",
                    "publish_tf": "true",
                },
            ),
            include("inspection_robot_core", "core.launch.py"),
            include("inspection_robot_safety", "safety.launch.py"),
            include("inspection_robot_base", "base.launch.py"),
            include(
                "inspection_robot_localization",
                "localization.launch.py",
                ekf_required,
            ),
            include("inspection_robot_slam", "slam.launch.py", IfCondition(slam)),
            include(
                "inspection_robot_localization",
                "amcl.launch.py",
                amcl_enabled,
            ),
        ]
    )
