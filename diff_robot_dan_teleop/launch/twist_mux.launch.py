from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    use_sim_time_launch_arg = DeclareLaunchArgument(
        "cmd_vel_config",
        default_value='False',
        description="Current topic of the robot to move / cmd_vel with joystick"
    )
    
    teleop_package = 'diff_robot_dan_teleop'
    twist_mux_config = 'twist_mux_config.yaml'
    twist_mux_path = os.path.join(get_package_share_directory(teleop_package), 'config', twist_mux_config)

    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_path, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diff_robot_dan_controller/cmd_vel_unstamped')]
        )
    
    return LaunchDescription([
        use_sim_time_launch_arg,
        twist_mux_node
    ])