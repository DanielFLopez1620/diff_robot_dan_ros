#!/usr/bin/env python3

# Based on the ROS2 launch from teleop_joy package: 
#   https://github.com/ros2/teleop_twist_joy/blob/rolling/launch/teleop-launch.py
# Based on the ROS2 articubot_one joysticks launch:
#   https://github.com/joshnewans/articubot_one/blob/humble/launch/joystick.launch.py
# Made by : DanielFLopez1620
# Description: Run joystick configuration for Xbox Controller to publish vel.

# ------------------------------ PYTHON DEPENDENCIES -------------------------- 
import os
from ament_index_python.packages import get_package_share_directory

# ------------------------------ LAUNCH DEPENDENCIES --------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# ------------------------------ LAUNCH DESCRIPTION ---------------------------

def generate_launch_description():
    """
    Script oriented to launch the joy_node and teleop_twist_joy node, to play
    with any robot that has /cmd_vel topic, in this case it is renamed to be
    set as a deafult to /diff_robot_dan_controller/cmd_vel_unstamped.
    """

    # Consider configurations for sim_time (simulation) and cmd_vel_config(
    # real topic of the robot)
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_config = LaunchConfiguration('cmd_vel_config')

    # Add launch arguments oriented to the previous configurations defined
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    cmd_vel_config_launch_arg = DeclareLaunchArgument(
        'cmd_vel_config',
        default_value='/diff_robot_dan_controller/cmd_vel_unstamped',
        description='Topic for cmd_vel according control, nav2 or sim'
    )
    
    # Consider .yaml file for configuration params
    joy_params = os.path.join(
        get_package_share_directory('diff_robot_dan_teleop'),'config',
            'joystick_config.yaml')

    # Joy node for recieving info of the controller
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    # Node that traslates the lecture of the joystick to publish it as cmd_vel
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_config)]
         )

    # Launch the considered args and nodes
    return LaunchDescription([
        use_sim_time_launch_arg,
        cmd_vel_config_launch_arg,
        joy_node,
        teleop_twist_joy_node,    
    ])