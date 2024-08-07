#!/usr/bin/env python3

# Original Author: Josh Newans | Articulated Robotics
# Source: https://github.com/joshnewans/articubot_one/blob/humble/launch/launch_sim.launch.py
# Modified, adapted and commented by: DanielFLopez1620
# Description: Run base gazebo simulation with robot spawned in empty world

# ------------------------------- PYTHON DEPENDENCIES -------------------------
import os
from ament_index_python.packages import get_package_share_directory

# ------------------------------- LAUNCH DEPENDENCIES -------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ------------------------------- LAUNCH DESCRIPTION -------------------------

def generate_launch_description():
    """
    Script oriented to launch the base simulation with visualization of the 
    Diff Robot Dan robot model in a empty world as a simple demostration
    considering gazebo controllers. 
    """

    # Name of description package
    description_package='diff_robot_dan_description'

    # Launch robot description and consider simulation time
    diff_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(description_package),
            'launch','display_model.launch.py' )]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 
                'gazebo.launch.py')]),
    )

    # Run the spawner node from the gazebo_ros package. 
    # The entity name doesn't really matter if you only have a single robot. 
    # But if you want to run multiple robots, you may consider better names.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diff_robot_dan',
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.06'],
                        output='screen')

    # Launch the node and configurations defined
    return LaunchDescription([
        diff_robot_description,
        gazebo,
        spawn_entity,
    ])