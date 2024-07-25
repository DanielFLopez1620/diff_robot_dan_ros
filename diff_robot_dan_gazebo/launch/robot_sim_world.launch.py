# Based on the ROS2_Control launch from example 9 of Humble tutorials and
# https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz with the launch file"
        )
    )

    world = LaunchConfiguration('world')
    cmd_vel_config = LaunchConfiguration('cmd_vel_config')
    
    description_package = 'diff_robot_dan_description'
    gazebo_package = 'diff_robot_dan_gazebo'
    teleop_package = 'diff_robot_dan_teleop'
    
    world_name = 'gazebo_laberynth.world'
    world_path = os.path.join(get_package_share_directory(gazebo_package),'worlds', world_name)
    
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to world where the robot will be spawned')
    
    cmd_vel_config_launch_arg = DeclareLaunchArgument(
        name='cmd_vel_config',
        default_value='/cmd_vel_joy',
        description="Current topic of the robot to move /cmd_vel with"
    )



    diff_robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','robot_description.launch.py')]), 
                    launch_arguments={
                        'use_sim_time': 'true',
                        'use_ros2_control': 'true'
                    }.items()
    )

    gazebo_params_file = os.path.join(
                get_package_share_directory(gazebo_package),'config','gazebo_params.yaml')


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments = {
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                        'world': world,
                        'verbose' : 'false'
                    }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diff_robot_dan',
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.08'],
                        output='screen')

    delayed_controller_manager_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_robot_dan_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster",]
            )
        ],
    )
    
    joystick_teleop = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(teleop_package),'launch','joystick_teleop.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'cmd_vel_config': cmd_vel_config}.items()
    )
    
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(teleop_package), 'launch', "twist_mux.launch.py"
        )]), launch_arguments={'use_sim_time':'true'}.items()
    )
    
    # Launch them all!
    return LaunchDescription([
        world_launch_arg,
        cmd_vel_config_launch_arg,
        diff_robot_description,
        gazebo,
        spawn_entity,
        delayed_controller_manager_spawner,
        joystick_teleop,
        twist_mux
    ])