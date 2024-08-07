# Diff Robot Dan

![robot_video](/resources/diff_robot_dan_showcase.gif)

## Overview

This is a collection of packages for using the Diff Robot Dan, a ROS2 low-cost differential robot that can create 2D maps, explore and navigate in flat surfaces in both real and simulated worlds. It was originally developed with the mission of learning and exploring ROS2 with mobile robots, while using components for other projects I made in the past at college.

**Keywords:** ROS2, differential robot, nav2, slam, ros2_control, simulation, DIY

## Construction

The list of the components used to build this robot can be found below:

- Raspberry Pi 4b.
- Arduino Uno R3.
- Expansion Shield for Arduino Uno R3.
- Motor Driver L298N
- Motor DC (100 rpms) with quadrature encoder  kit (with neumatic, ring, base and coupling) x 2.
- Powerbank with 2.1 A Output.
- 18650 Batteries
- 18650 Battery holder.
- LD06 LIDAR Sensor.
- Microsoft LifeCam HD 3000
- Caster Wheel.
- M3 x 30 spacers
- M3 nuts and screws.

#### Disclaimer:READMEhis is the reason why the robot counts with 2 electrical sources (the powerbank and the 18650 batteries) as the output current from the powerbank was drained very quickly by the combination of all the elements, then I decided to give the Arduino and the motors independent power that comes from the other batteries. 

Maybe in the future I will buy a better power source for the robot, but for now, if you have a better powerbank than mine and can supply more than 3.5 Amps, you may like to modify the model so you can have only one power source.


## License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Author:** Daniel Felipe López E

**Maintainer:** Daniel Felipe López E, dfelipe.lopez@gmail.com**

The Diff Robot Dan packages has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on respectively Ubuntu 22.04 and Docker Images for Ubuntu 22.04 or properly set up for the distro specified.


## Installation

### Installation from Packages

Create a workspace if you do not have one:

```bash
    cd ~
    mkdir -p ros2_ws/src
    cd ~/ros2_ws/
    colcon build
```

After that, clone the following repositories by using **https** or **SSH**, to include the serial connection and the LIDAR package:

```bash
cd src/
https://github.com/joshnewans/serial.git
https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
```

Also, clone this repository:

```bash
git clone https://github.com/DanielFLopez1620/diff_robot_dan_ros.git
```

Install and check dependencies with **rosdep**:

```bash
cd ..
rosdep install --from-paths src --ignore-src -i -y
```

#### Dependencies

Let's make a brief mention on the dependencies used for the package:

- [Robot Operating System 2 (ROS 2):](https://docs.ros.org/en/humble/Releases.html): Set of software libraries and tools for building robot applications.
- [Gazebo Classic:](https://classic.gazebosim.org/) Robotic simulator compatible with ROS/ROS2 
- [ros2_control](https://control.ros.org/humble/index.html): Framework for (real-time) control of robots using ROS2.
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox): Set of tools and capabilities for 2D SLAM in ROS2.
- [nav2](https://nav2.org/): Provides perception, planning, control, localization, visualization and behaviors for autonomous systems.
- [URDF](https://wiki.ros.org/urdf) Unified Robot Description Format. 
- [Xacro](https://wiki.ros.org/xacro) For macro usage in XML language.

If you prefer a manual installation on the dependencies, install the next packages:

    sudo apt install ros-humble-urdf ros-humble-urdf-launch ros-humble-xacro
    sudo apt install ros-humble-slam-toolbox
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
    sudo apt install gazebo-ros gazebo-ros2-control
    sudo apt install ros-humble-nav2*

### Building from Source

After you have made the installation of the required dependencies, build the package:

```bash
colcon build
```

And source the packages (and if you use bash) by running:

```bash
source install/setup.bash
```

### Running in Docker

Docker support will come in the future, also will come with support for [Flatboat](https://juancsucoder.github.io/flatboat-docs/)

## Usage

To get started you can use the simulation in gazebo:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch diff_robot_dan_gazebo robot_sim_world.launch.py
```

You can use a game controller to move the robot (node for joy is launched when simulation is displayed) or use the keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Packages

The packages present in this repository are:

- **[diff_robot_dan:](/diff_robot_dan/README.md)** Base metapackage of the project.
- **[diff_robot_dan_base:](/diff_robot_dan_base/README.md)** Contains the Arduino and Platformio Codes for the Arduino Uno R3 mounted on the robot that control the DC motors.
- **[diff_robot_dan_bringup:](/diff_robot_dan_bringup/README.md)** Has the launches and configs for getting ready with the real robot. 
- **[diff_robot_dan_controller:](/diff_robot_dan_controller/)** In the future it will have the proper controller of the robot.
- **[diff_robot_dan_description:](/diff_robot_dan_description/)** Contains the URDF and Xacro files for the description of the robot for visualization, simulation, sensors and control.
- **[diff_robot_dan_gazebo:](/diff_robot_dan_gazebo/README.md)** Present the launches and configs for using the simulation of the robot in a empty world, a laberynth or a custom world using ros2 control or standard gazebo controllers.
- **[diff_robot_dan_navigation:](/diff_robot_dan_navigation/)** Oriented to the usage of nav2 with the car in simulation or with the real robot, after the creation of maps with the slam_toolbox.
- **[diff_robot_dan_slam:](/diff_robot_dan_slam/)** For using the slam_toolbox (and the proper configuration for the robot) to create maps or localize itself.
- **[diff_robot_dan_teleop:](/diff_robot_dan_teleop/)** To control the robot with a game controller and implementing a mux when having different subscriptions at the cmd_vel of the car.
- **[diffdrive_arduino:](https://github.com/Buzzology/diffdrive_arduino/tree/3883c00479e2eeaa844ad8ae14fe147ee742ea7d)** A cloned version of a fork made by Buzzology of the diffdrive_arduino from Josh Newans as a ros2_control implementation of the robot with additional notes for the usage with this robot. Diff Robot Dan currently uses this interface but another one is being developed.

If you want more detail and learn about the executables and launches, each package has its own **README** file with usage notes.

## Acknowledgement:

This repo was made during the process of learning and experimenting with ROS2, then it was based and has content related to the [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html), the [Articubot One Tutorials](https://www.youtube.com/watch?v=OWeLUSzxMsw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&pp=iAQB) from [Josh Newans](https://github.com/joshnewans) and the [Automatic Addison](https://automaticaddison.com/)'s Tutorials from Addison Sears-Collins. Further references and licenses can be found on the codes from the different packages.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/DanielFLopez1620/diff_robot_dan_ros/issuess).
