from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def include(pkg,file,condition=None):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg),'launch',file])),condition=condition)
def generate_launch_description():
    loc=LaunchConfiguration('enable_localization')
    return LaunchDescription([
      DeclareLaunchArgument('enable_localization',default_value='false'),
      include('inspection_robot_description','description.launch.py'),
      include('inspection_robot_core','core.launch.py'),
      include('inspection_robot_safety','safety.launch.py'),
      include('inspection_robot_base','base.launch.py'),
      include('inspection_robot_localization','localization.launch.py',IfCondition(loc)),
    ])
