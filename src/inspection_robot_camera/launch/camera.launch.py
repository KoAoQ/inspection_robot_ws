import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    depth_fps = LaunchConfiguration("depth_fps")
    color_fps = LaunchConfiguration("color_fps")
    enable_color = LaunchConfiguration("enable_color")
    enable_ir = LaunchConfiguration("enable_ir")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_navigation_filter = LaunchConfiguration("enable_navigation_filter")
    publish_tf = LaunchConfiguration("publish_tf")

    astra_launch = os.path.join(
        get_package_share_directory("astra_camera"),
        "launch",
        "astra.launch.xml",
    )
    filter_params = os.path.join(
        get_package_share_directory("inspection_robot_camera"),
        "config",
        "camera.yaml",
    )

    driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch),
        launch_arguments={
            "camera_name": camera_name,
            "depth_registration": "false",
            "enable_point_cloud": enable_point_cloud,
            "enable_colored_point_cloud": "false",
            "color_width": "640",
            "color_height": "480",
            "color_fps": color_fps,
            "enable_color": enable_color,
            "depth_width": "640",
            "depth_height": "480",
            "depth_fps": depth_fps,
            "enable_depth": "true",
            "ir_width": "640",
            "ir_height": "480",
            "ir_fps": depth_fps,
            "enable_ir": enable_ir,
            "publish_tf": publish_tf,
            # The camera-to-optical-frame transforms are rigid; publish them on /tf_static.
            # This avoids timestamp races with the 30 Hz depth stream.
            "tf_publish_rate": "0.0",
            "enable_d2c_viewer": "false",
            "enable_publish_extrinsic": "false",
        }.items(),
    )

    navigation_filter = Node(
        package="inspection_robot_camera",
        executable="depth_cloud_filter_node",
        name="depth_cloud_filter",
        output="screen",
        parameters=[filter_params],
        condition=IfCondition(enable_navigation_filter),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera",
                description="Astra ROS namespace and frame prefix",
            ),
            DeclareLaunchArgument(
                "depth_fps",
                default_value="30",
                description="Raw depth stream rate; filtering is applied downstream",
            ),
            DeclareLaunchArgument(
                "color_fps",
                default_value="30",
                description="Raw color stream rate",
            ),
            DeclareLaunchArgument(
                "enable_color",
                default_value="true",
                description="Publish the Astra color image",
            ),
            DeclareLaunchArgument(
                "enable_ir",
                default_value="false",
                description="Publish the separate infrared image stream",
            ),
            DeclareLaunchArgument(
                "enable_point_cloud",
                default_value="true",
                description="Publish /camera/depth/points for navigation",
            ),
            DeclareLaunchArgument(
                "enable_navigation_filter",
                default_value="true",
                description="Publish filtered navigation cloud in base_footprint",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish camera_link to optical-frame transforms",
            ),
            driver,
            navigation_filter,
        ]
    )
