#!/usr/bin/env python3

# Based on the mux implemented in the launch robot from the Articubot One:
#   https://github.com/joshnewans/articubot_one/blob/humble/launch/launch_robot.launch.py
# Made by: DanielFLopez1620
# Description: Twist mux for combining cmd_vel from joy and nav2

# -------------------------------- PYTHON DEPENDENCIES ------------------------
import os
from ament_index_python.packages import get_package_share_directory

# ------------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# ------------------------------ LAUNCH DESCRIPTION ---------------------------

def generate_launch_description():
    """
    Script oriented to implement a twist mux for cmd_vel considerations over
    the Diff Robot Dan, in case of Nav2 and joy info.
    """ 

    # Consider configuration in case of simulation time
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Add the proper argument for the simulation time
    use_sim_time_launch_arg = DeclareLaunchArgument(
        "cmd_vel_config",
        default_value='False',
        description="Current topic of the robot to move / cmd_vel with joystick"
    )
    
    # Confirm info of the teleop package and the corresponding .yaml file
    teleop_package = 'diff_robot_dan_teleop'
    twist_mux_config = 'twist_mux_config.yaml'
    twist_mux_path = os.path.join(get_package_share_directory(teleop_package), 
        'config', twist_mux_config)

    # Invoke node and remap output to the proper cmd_vel
    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_path, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diff_robot_dan_controller/cmd_vel_unstamped')]
        )
    
    # Launch arguments and nodes
    return LaunchDescription([
        use_sim_time_launch_arg,
        twist_mux_node
    ])