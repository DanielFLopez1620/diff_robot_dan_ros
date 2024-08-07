#!/usr/bin/env python3

# Original Author: Josh Newans | Articulated Robotics
# Source: https://github.com/joshnewans/articubot_one/blob/main/launch/rsp.launch.py
# Modified by: DanielFLopez1620
# Description: Launch the robot description of a robot, to make it available.

# ---------------------- PYTHON DEPENDENCIES ----------------------------------
import os
import xacro
from ament_index_python.packages import get_package_share_directory

# --------------------- LAUNCH DEPENDENCIES -----------------------------------
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """
    Script oriented to launch the robot description of the diff_robot_dan without
    visualization, by just considering the pass of the Xacro file model.
    """

    # Configurations declared 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Provide the package path
    pkg_path = os.path.join(get_package_share_directory('diff_robot_dan_description'))
    xacro_file = os.path.join(pkg_path,'urdf','diff_robot_dan.xacro')

    # Process the URDF file from your description package
    # It also considers the conversion from Xacro to URDF
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # Create a robot_state_publisher node, related with the state of the joints depending on their type.
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch the nodes configured with the params that were set.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control in XML descriptions'),

        node_robot_state_publisher
    ])