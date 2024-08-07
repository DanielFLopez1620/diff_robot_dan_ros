#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Taken from https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/localization_launch.py
# Additional comments by: DanielFLopez1620
# Description: Launch to use localization with Nav2

# ----------------------------- PYTHON DEPENDENCIES ---------------------------
import os
from ament_index_python.packages import get_package_share_directory

# --------------------------- LAUNCH DEPENDENCIES -----------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# -------------------------- ADDITIONAL ROS2 DEPENDENCIES ---------------------
from nav2_common.launch import RewrittenYaml

# -------------------------- LAUNCH DESCRIPTION -------------------------------
def generate_launch_description():
    """
    Launch nav2 bringup for loading planner, smoothers, waypoing follower, 
    controller and behavior trees that works for a robot with /scan topic and
    a odemtry block oriented to localization
    """
    # Get the launch directory
    nav_package = get_package_share_directory('diff_robot_dan_navigation')
    slam_package = get_package_share_directory('diff_robot_dan_slam')

    # Consider configurations for namespace, use_sime_time (simulation time),
    # autostart, params_file (.yaml file for the nodes and components),
    # use_composition (to use components), container_name, use_respawn (come
    # again if something goes wrong) and log level (info results).
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['map_server', 'amcl']

    # Required remmapings 
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Params oriented for future substitution
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    # Create our own temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Consider the next launch arguments and nodes
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Argument for namespace, by default it is empty
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        # Argument for map, default is the gazebo laberythn from the Diff
        # Robot Dan package
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(slam_package, 'maps', 
                'gazebo_laberynth_map.yaml'),
            description='Full path to map yaml file to load'),

        # Argument for using simulation time, by default it is false
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Argument for startup of the nav2 stack
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        # Argument for .yaml file
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_package, 'config', 'nav2_config.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        # Load node for map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        # Load the Adaptative Monte Carlo Localization for Nav2
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        # Load lifecycle checker for nav2 nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])