#!/usr/bin/env python3

# Original Author: Josh Newans | Articulated Robotics
# Source: https://github.com/joshnewans/articubot_one/blob/main/launch/launch_robot.launch.py
# Modified, adapted and commented by: DanielFLopez1620
# Description: Launch set up of the robot to run it in real life.


# --------------------- PYTHON RELATED DEPENDENCIES ---------------------------
import os
from ament_index_python.packages import get_package_share_directory

# -------------------------- LAUNCH DEPENDENCIES ------------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition

# ------------------------ LAUNCH STRUCTURE -----------------------------------
def generate_launch_description():
    """
    Oriented to set up the robot controller, model and sensors for the
    Diff Robot Dan.
    """
    # List for launch arguments
    declared_arguments = []

    # Add gui arg to check wether or not launch RVIZ2
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz with the launch file"
        )
    )
    
    # Add console arg to specify if outputs are screen or log type
    declared_arguments.append(
        DeclareLaunchArgument(
            "output",
            default_value="screen",
            description="Output Option (screen or log)"
        )
    )

    # Add lidar port arg for specifying the port of the LIDAR (and change it
    # if it is different)
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Port where the LIDAR is connected'
        )
    )

    # Consider the configurations for the previously defined launch arguments
    gui_config = LaunchConfiguration("gui")
    output_config = LaunchConfiguration("output")
    lidar_port_config = LaunchConfiguration('lidar_port')
    
    # Define name of the package of interest (Description for URDF/Xacro file,
    # Bringup for start and set up info and teleop for moving the robot with
    # a controller)
    description_package = 'diff_robot_dan_description'
    bringup_package = 'diff_robot_dan_bringup'
    teleop_package = 'diff_robot_dan_teleop'

    # Include launch to load robot_model topic with use_sim_time set to false
    # and using ros2_control
    diff_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ os.path.join( get_package_share_directory(description_package),
                'launch', 'robot_description.launch.py')]),
        launch_arguments={'use_sim_time': 'false', 
                          'use_ros2_control': 'true'}.items()
    )

    # TODO: Delete?
    robot_description = Command(
        ['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # Consider the description package to access the controller params that are
    # related with the model construction and real robot controllers
    controller_params_file = os.path.join(
        get_package_share_directory(description_package),
        'config','robot_controllers.yaml')
   
   # Load controller manager (from ros2_control) by considreing obtainer params
   # from .yaml description file.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
	    remappings=[ ('~/robot_description','robot_description')]
    )
    
    # Add a delay to wait and allow the robot_model to load
    delayed_controller_manager_spawner = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )
    
    # Load the spawner of the differential controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_robot_dan_controller",]
    )
    
    # On start of the process of the controller manager, spawn the 
    # differential controller
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )
    
    # Also load the spawner of the joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",]
    )
    
    # On start of the control manager, spawn the joint broadcaster
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    # Include the luanch of the LIDAR, and consider for now the default params
    # of the port and logs display.
    ld06_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(bringup_package),
            'launch', "ld06_lidar.launch.py")]),
        launch_arguments={
            'output' : output_config,
            'lidar_port' : lidar_port_config
        }.items()
    )
    
    # Twist mux is required to receive velocitie commands from different
    # sources with different priorities, in this case, from the joy node
    # and the slam/navigation nodes.
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(teleop_package), 
            'launch', "twist_mux.launch.py")])
    )

    # Conditionally consider RVIZ2 for visualiazation if it is requested
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            description_package, 'rviz', 'rviz_robot_model.rviz'))],
        output=output_config,
        condition=IfCondition(gui_config)
    )
    
    # TODO: Add action when car is ready?

    # List and launch the considered processes, configs and params.
    return LaunchDescription(
        declared_arguments + [
        diff_robot_description,
        delayed_controller_manager_spawner,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        ld06_lidar,
        twist_mux,
        rviz2_node
        ]
    )
