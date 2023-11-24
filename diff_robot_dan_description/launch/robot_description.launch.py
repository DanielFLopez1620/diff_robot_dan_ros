# Original Author: Josh Newans | Articulated Robotics
# Modified by: DanielFLopez1620
# Description: Launch the robot description of a robot, to make it available.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check for 'use_sim_time' param
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file from your description package (this includes the traslation of xacro->urdf)
    pkg_path = os.path.join(get_package_share_directory('diff_robot_dan_description'))
    xacro_file = os.path.join(pkg_path,'urdf','diff_robot_dan.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])

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