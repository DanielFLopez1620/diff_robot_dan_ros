from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    slam_package = 'slam_toolbox'
    diff_robot_dan_slam = 'diff_robot_dan_slam'
    slam_config = os.path.join(
                get_package_share_directory(diff_robot_dan_slam),'config','online_async_mapper_config.yaml')
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(slam_package),'launch','online_async_launch.py')]), 
                launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_config}.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        
        slam_toolbox    
    ])