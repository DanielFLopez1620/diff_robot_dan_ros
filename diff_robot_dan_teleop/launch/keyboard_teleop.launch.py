from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    print("Not working as launch doesn't read keys...\For now, just use:")
    print("ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_dan_robot_controller/cmd_vel_unstamped")

    teleop_keyboard_node = Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            output='screen',
            #prefix = 'xterm -e',
            name="teleop_twist_keyboard",
            remappings=[('cmd_vel','/diff_dan_robot_controller/cmd_vel_unstamped')]
         )
    
    teleop_keyboard_cpp = Node(
            package="teleop_cpp_ros2",
            executable="teleop",
            output='screen',
            emulate_tty=True,
            #prefix = 'xterm -e',
            name="teleop_key",
            remappings=[('cmd_vel','/diff_dan_robot_controller/cmd_vel_unstamped')]
         )
    
    #teleop_process = ExecuteProcess(
    #    cmd=[["ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_dan_robot_controller/cmd_vel_unstamped"]]
    #)

    return LaunchDescription([        
        teleop_keyboard_node,
        # teleop_process,
    ])