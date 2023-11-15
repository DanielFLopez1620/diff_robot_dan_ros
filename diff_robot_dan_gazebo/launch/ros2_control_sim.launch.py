# Based on the ROS2_Control launch from example 9 of Humble tutorials
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

    gui = LaunchConfiguration("gui")

    description_package='diff_robot_dan_description'

    diff_robot_description = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(description_package),'launch','robot_description.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diff_robot_dan',
                                    '-x', '0',
                                    '-y', '0',
                                    '-z', '0.15'],
                        output='screen')
    """
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_dan_robot_controller", "--controller-manager", "/controller_manager"],
    )

    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster" , "--controller-manager", "/controller_manager"],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(gui),
    )
    """
    delayed_controller_manager_spawner = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_dan_robot_controller", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster" , "--controller-manager", "/controller_manager"],
            )
        ],
    )

    # Launch them all!
    return LaunchDescription([
        diff_robot_description,
        gazebo,
        spawn_entity,
        delayed_controller_manager_spawner
        #diff_drive_spawner,
        #joint_broad_spawner,
        #rviz_node
    ])