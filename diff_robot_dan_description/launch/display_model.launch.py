#!/usr/bin/env python3

# Original Author: Addison Sears-Collins | Automatic Addison
# Source: https://automaticaddison.com/how-to-load-a-urdf-file-into-rviz-ros-2/
# Modified by: DanielFLopez1620
# Description: Launch a differential robot for the visualization model.
 
# --------------------------- PYTHON DEPENDENCIES -----------------------------
import os

# --------------------------- LAUNCH DEPENDENICES -----------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
	"""
	Script oriented to load the Xacro model of the robot and display it in 
	RVIZ2 with a defualt config for TF, URDF visualization while experimenting
	with a simple publisher.
	"""
	# Set the path to this package.
	pkg_share = FindPackageShare(
		package='diff_robot_dan_description').find('diff_robot_dan_description')
	
	# Set the path to the RViz configuration settings
	default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_robot_model.rviz')
	
	# Set the path to the URDF file
	default_urdf_model_path = os.path.join(pkg_share, 'urdf/diff_robot_dan.xacro')
	 
	# Launch configuration variables specific to simulation
	gui = LaunchConfiguration('gui')
	urdf_model = LaunchConfiguration('urdf_model')
	rviz_config_file = LaunchConfiguration('rviz_config_file')
	use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
	use_rviz = LaunchConfiguration('use_rviz')
	use_sim_time = LaunchConfiguration('use_sim_time')
	
	# Add argument oriented to the path (absolute path) to the URDF/Xacro model  
	declare_urdf_model_path_cmd = DeclareLaunchArgument(
		name='urdf_model', 
		default_value=default_urdf_model_path, 
		description='Absolute path to robot urdf file')
		
	# Add argument to pass the path to a rviz config file (absolute path)
	declare_rviz_config_file_cmd = DeclareLaunchArgument(
		name='rviz_config_file',
		default_value=default_rviz_config_path,
		description='Full path to the RVIZ config file to use')
	
	# Add argument for gui (boolean) to display or not joint_publisher_gui
	declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
		name='gui',
		default_value='True',
		description='Flag to enable joint_state_publisher_gui')
	
	# Add argument to check whether or not consider the joint_state_publisher
	declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
		name='use_robot_state_pub',
		default_value='True',
		description='Whether to start the robot state publisher')
	
	# Add argument for launching or not RVIZ2
	declare_use_rviz_cmd = DeclareLaunchArgument(
		name='use_rviz',
		default_value='True',
		description='Whether to start RVIZ')
		
	# Add argument to check for sim_time (in case of simulation)
	declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='True',
		description='Use simulation (Gazebo) clock if true')
	
	# Consider node for the state_publisher (as initially we do not receive any
	# inputs to the joint/wheels of the robot, then we need a reference for the
	# continuous joints).
	start_joint_state_publisher_cmd = Node(
		condition=UnlessCondition(gui),
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher')
	
	# Consider the node gui for manipulating the joints/wheel with bard
	# indicators in an intuitive way.
	start_joint_state_publisher_gui_node = Node(
		condition=IfCondition(gui),
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		name='joint_state_publisher_gui')
	
	# Consider the node robot_state_publisher to add the model and consider the
	# information of the URDF/Xacro robot model.
	start_robot_state_publisher_cmd = Node(
		condition=IfCondition(use_robot_state_pub),
		package='robot_state_publisher',
		executable='robot_state_publisher',
		# Include parse from Xacro to URDF and consider sim_time
		parameters=[{'use_sim_time': use_sim_time, 
		'robot_description': Command(['xacro ', urdf_model, ' sim_mode:=true'])}],
		arguments=[default_urdf_model_path])
	
	# Launch RViz if it is specified
	start_rviz_cmd = Node(
		condition=IfCondition(use_rviz),
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_file])
	
	# Create the launch description and populate
	ld = LaunchDescription()
	
	# Declare the launch options
	ld.add_action(declare_urdf_model_path_cmd)
	ld.add_action(declare_rviz_config_file_cmd)
	ld.add_action(declare_use_joint_state_publisher_cmd)
	ld.add_action(declare_use_robot_state_pub_cmd)  
	ld.add_action(declare_use_rviz_cmd) 
	ld.add_action(declare_use_sim_time_cmd)
	
	# Add any actions
	ld.add_action(start_joint_state_publisher_cmd)
	ld.add_action(start_joint_state_publisher_gui_node)
	ld.add_action(start_robot_state_publisher_cmd)
	ld.add_action(start_rviz_cmd)
	
	# Final return of the proposed actions.
	return ld