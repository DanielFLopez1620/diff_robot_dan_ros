#!/usr/bin/env python3

# -------------------------- LAUNCH DEPENDENCIES ------------------------------
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# -------------------------- LAUNCH STRUCTURE ---------------------------------
def generate_launch_description():
    """
    LDLidar LD06 bringup for publishing /scan info.

    Based on the package: ldlidar_ros2
    https://github.com/ldrobotSensorTeam/ldlidar_ros2
    """

    # Declare output argument 
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output options (screen or log)'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Port where the LIDAR is connected'
    )


    # Consider LIDAR node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output=LaunchConfiguration('output'),
        parameters=[
            # Base parameter for LD06 LIDAR Model (considering manufacturer 
            # data), but keep an eye on the port depending your connections.
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'laser_frame'},
            {'port_name': LaunchConfiguration('lidar_port')},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
      ]
    )
    
    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add argument declarations
    ld.add_action(output_arg)
    ld.add_action(lidar_port_arg)

    # Add executable nodes
    ld.add_action(ldlidar_node)

    # Return LaunchDescription
    return ld