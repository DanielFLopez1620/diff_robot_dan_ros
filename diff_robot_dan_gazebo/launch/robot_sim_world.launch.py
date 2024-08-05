#!/usr/bin/env python3

# Original Author: Automatic Addison
# Source: https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/
# Based on the ROS2_Control launch from example 9 of Humble tutorials
# Modified, adapted and commented by: DanielFLopez1620
# Description: Launch the robot simulation in a previously created gazebo maze.

# ---------------------------- PYTHON DEPENDENICES ----------------------------
import os
from ament_index_python.packages import get_package_share_directory

# ----------------------------- LAUNCH DEPENDENCIES ---------------------------
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, 
    TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Script oriented to spawn the Diff Robot Dan in a Gazebo generated maze,
    ready to run to explore the usage of the robot like SLAM and Navigation.
    """

    # Consider configurations for the launch arguments that will be declared
    gui_config = LaunchConfiguration("gui_config")
    world_config = LaunchConfiguration("world_config")
    cmd_vel_config = LaunchConfiguration("cmd_vel_config")
    
    # Names of share directories for the robot description, sim and teleop
    description_package = 'diff_robot_dan_description'
    gazebo_package = 'diff_robot_dan_gazebo'
    teleop_package = 'diff_robot_dan_teleop'

    # World selection
    world_name = 'gazebo_laberynth.world'
    world_path = os.path.join(
        get_package_share_directory(gazebo_package),'worlds', world_name)

    # Adding the launch arguments for gui (wheter to use Rviz2), 
    # world (path to the world to load) and cmd_vel_config (topic for cmd_vel)
    gui_arg = DeclareLaunchArgument(
            "gui_config",
            default_value="false",
            description="Start RViz with the launch file"
        )
    world_arg = DeclareLaunchArgument(
            "world_config",
            default_value=world_path,
            description="Full path to world where the robot will be spawned"
        )
    cmd_vel_config_arg = DeclareLaunchArgument(
            "cmd_vel_config",
            default_value="/cmd_vel_joy",
            description="Current topic of the robot to move /cmd_vel with"
        )



    # Include launch to load the robot_description
    diff_robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','robot_description.launch.py')]), 
                    launch_arguments={
                        'use_sim_time': 'true',
                        'use_ros2_control': 'true'
                    }.items()
    )

    # Consider additional arguments for the gazebo simulation
    gazebo_params_file = os.path.join(
        get_package_share_directory(gazebo_package), 'config', 'gazebo_params.yaml')


    # Include launch to start the simulator, but passing additional args like
    # world, verbose and additional params
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments = {
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                        'world': world_config,
                        'verbose' : 'true'
                    }.items()
             )

    # Spawn robot based on the robot description topic in the given position
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diff_robot_dan',
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.08'],
                        output='screen')

    # Add delayed action to spawn the controller while giving time to the model
    # for loading.
    delayed_controller_manager_spawner = TimerAction(
        period=10.0,
        actions=[
            # Consider both controllers (differential and broadcaster)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_robot_dan_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"]
            )
        ],
    )

    # Add teleoperation when a controller is connected, by default the params
    # are optimized for Xbox Controller.
    joystick_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(teleop_package), 'launch', 
                'joystick_teleop.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 
            'cmd_vel_config': cmd_vel_config}.items()
    )

    # It is a need to add a mux as we will have different inputs from the
    # cmd_vel topic like the joystick and nav2 package
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(teleop_package), 'launch', 
                "twist_mux.launch.py")]), 
            launch_arguments={'use_sim_time':'true'}.items()
    )

    # Consider a path for the Rviz2 visualization config
    rviz_config_path = os.path.join(
        get_package_share_directory(description_package), 'rviz', 
            'rviz_robot_model.rviz')
    
    # Conditionally add RViz2 if respective argument is set to True
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(gui_config)
    )

    # Launch arguments and nodes declared
    return LaunchDescription([
        gui_arg,
        world_arg,
        cmd_vel_config_arg,
        diff_robot_description,
        gazebo,
        spawn_entity,
        delayed_controller_manager_spawner,
        joystick_teleop,
        twist_mux,
        rviz_node
    ])