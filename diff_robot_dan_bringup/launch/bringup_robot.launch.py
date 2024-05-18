import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node



def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz with the launch file"
        )
    )

    gui = LaunchConfiguration("gui")
    cmd_vel_config = LaunchConfiguration("cmd_vel_config")
    
    cmd_vel_config_launch_arg = DeclareLaunchArgument(
        "cmd_vel_config",
        default_value='/cmd_vel_joy'
        description="Current topic of the robot to move / cmd_vel"
    )

    description_package = 'diff_robot_dan_description'
    bringup_package = 'diff_robot_dan_bringup'
    teleop_package = 'diff_robot_dan_teleop'

    diff_robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','robot_description.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_params_file = os.path.join(
                get_package_share_directory(description_package),'config','robot_controllers.yaml')
   
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
	    remappings=[ ('~/robot_description','robot_description')]
    )
    
    delayed_controller_manager_spawner = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_dan_robot_controller",]
    )
    
    delayed_diff_drive_spawner = RegisterEventHandler(
        
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",]
    )
    
    delayed_joint_broad_spawner = RegisterEventHandler(
        
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    ld06_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(bringup_package), 'launch', "ld06_lidar.launch.py"
        )])
    )
    
    joystick_teleop = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(teleop_package),'launch','joystick_teleop.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'cmd_vel_config': cmd_vel_config}.items()
    )
    
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(teleop_package), 'launch', "twist_mux.launch.py"
        )])
    )
    
    return LaunchDescription([
        cmd_vel_config_launch_arg,
        diff_robot_description,
        delayed_controller_manager_spawner,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        ld06_lidar,
        joystick_teleop
    ])
