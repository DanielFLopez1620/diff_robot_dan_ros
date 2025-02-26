#!/usr/bin/env python3

# Author: DanielFLopez1620
# Description: Launch for using the slam_toolbox in localization mode.

# ------------------------ PYTHON DEPENDENCIES --------------------------------
import os
from ament_index_python.packages import get_package_share_directory

# -----------------------  LAUNCH DEPENDENCIES --------------------------------
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

# ------------------------ LAUNCH DESCRIPTION --------------------------------

def generate_launch_description():
    """
    Script for using slam_toolbox for generation for localization with an
    already created map by using the Diff Robot Dan.
    """
    # Set configuration for sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Add the corresponding argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value='true',
        description="Wheter or nto to use sim time Gazebo"
    )

    # Declare important names
    slam_package = 'slam_toolbox'
    diff_robot_dan_slam = 'diff_robot_dan_slam'

    # Get package path to obtain the corresponding .yaml file
    slam_config = os.path.join(
        get_package_share_directory(diff_robot_dan_slam),'config',
                'online_async_localization_config.yaml')
    
    # Include launch for the orginal slam_toolbox package
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(slam_package),'launch',
                'online_async_launch.py')]), 
        launch_arguments={'use_sim_time': use_sim_time, 
                          'slam_params_file': slam_config}.items()
    )
    
    # Declare arguments and nodes declared
    return LaunchDescription([
        use_sim_time_arg,        
        slam_toolbox    
    ])