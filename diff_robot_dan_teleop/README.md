# Diff Robot Dan Teleoperation Package

![teleop_showcase](/diff_robot_dan_teleop/resources/)

## Overview

Intented to use the Diff Robot Dan with a Xbox Controller for easier and faster tests and actions related with the movement of the robot in both real and simulated scenarios.

## Usage

### Joystick Teleop:

The [joystick_teleop.launch.py](/diff_robot_dan_teleop/launch/joystick_teleop.launch.py) incoportates the set up for the Xbox controller, you will need to use the joystick and press RB to generate the move.

```bash
ros2 launch diff_robot_dan_teleop_joystick joystick_teleop.launch.py
```

If you are using the real robot, you will need to launch the previous command from the computer you are running the visualizations, slam and navigation algorithms. Do not forget to run the bringup in the raspberry pi of the robot previously. 

If you run any simulation with ros2_controller activate, by defualt, the joy teleop will be launched too.

**NOTE:** Ensure that the controller has the proper permissions in the /dev dir, and also configure the correct port detected in your machine at the .yaml file for the controller.

### Twist Mux:

Use it when you can to incorporate multiple velocity commands (Twists) with the robot, for example, when using Xbox controller and Nav2 at the same time, by default it is called in an the bringup of the robot and the simulations with ros2_control activate.

```bash
ros2 launch diff_robot_dan_teleop_joystick joystick_teleop.launch.py
```

