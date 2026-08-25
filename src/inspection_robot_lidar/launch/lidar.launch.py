import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('inspection_robot_lidar'),
        'config',
        'lidar.yaml',
    )
    serial_port = LaunchConfiguration('serial_port')

    lidar_node = Node(
        package = 'lslidar_driver',
        executable = 'lslidar_driver_node',
        namespace = 'x10',
        name = 'lslidar_driver_node',
        output = 'screen',
        parameters = [
            config_file,
            {
                'serial_port' : serial_port,
                'frame_id' : 'laser_link',
                'laserscan_topic' : '/scan'
            },
        ],

    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value = '/dev/wheeltec_lidar',
                description = 'LS M10P serial device',
            ),
            lidar_node,
        ]
    )