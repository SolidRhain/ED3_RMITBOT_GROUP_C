from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='position_manager',
            executable='position_manager_node',
            name='position_manager',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])