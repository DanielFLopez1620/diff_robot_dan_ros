from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_config = LaunchConfiguration('cmd_vel_config')

    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    
    cmd_vel_config_launch_arg = DeclareLaunchArgument(
        'cmd_vel_config',
        default_value='/diff_dan_robot_controller/cmd_vel_unstamped',
        description='Topic for cmd_vel according control, nav2 or sim'
    )
    

    joy_params = os.path.join(get_package_share_directory('diff_robot_dan_teleop'),'config','joystick_config.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_config)]
         )

    return LaunchDescription([
        use_sim_time_launch_arg,
        cmd_vel_config_launch_arg,
        joy_node,
        teleop_twist_joy_node,    
    ])