from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params=os.path.join(get_package_share_directory('inspection_robot_base'),'config','base_driver.yaml')
    return LaunchDescription([Node(package='inspection_robot_base',executable='base_driver_node',name='inspection_robot_base',output='screen',parameters=[params])])
