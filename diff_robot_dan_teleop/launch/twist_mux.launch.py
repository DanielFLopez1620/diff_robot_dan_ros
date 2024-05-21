from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    teleop_package = 'diff_robot_dan_teleop'
    twist_mux_config = 'twist_mux_config.yaml'
    twist_mux_path = os.path.join(get_package_share_directory(teleop_package), 'config', twist_mux_config)

    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_path],
            remappings=[('/cmd_vel_out','/diff_dan_robot_controller/cmd_vel_unstamped')]
        )
    
    return LaunchDescription([
        twist_mux_node
    ])