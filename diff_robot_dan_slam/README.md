# Diff Robot Dan SLAM (Simultatineous Localization and Mapping) Package


## Overview

Oriented to use the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) package with the Diff Robot Dan. 

## Usage

### Joystick Teleop:

The [joystick_teleop.launch.py](/diff_robot_dan_teleop/launch/joystick_teleop.launch.py) incoportates the set up for the Xbox controller, you will need to use the joystick and press RB to generate the move.

```bash
ros2 launch diff_robot_dan_teleop_joystick joystick_teleop.launch.py
```

If you are using the real robot, you will need to launch the previous command from the computer you are running the visualizations, slam and navigation algorithms. Do not forget to run the bringup in the raspberry pi of the robot previously. 

If you run any simulation with ros2_controller activate, by defualt, the joy teleop will be launched too.This readme will be updated in the future to have more informaction on the package diff_robot_dan_slam.