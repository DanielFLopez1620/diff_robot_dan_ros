# diffdrive_arduino

## Overview
This node is designed to provide an interface between a `diff_drive_controller` from `ros_control` and an Arduino running firmware from `ros_arduino_bridge`.

It was originally developed by Josh Newans for the Articubot One, then forken by buzzology with make it compatible with ROS2 Humble. 

This is based on the commit 3883c00, which was cloned using:

```bash
git clone http://github.com/buzzology/diffdrive_arduino
git checkout 3883c00
```

## Content

This package contains the plugin for the differential controller that is compatible with the robot, the following definitions are present:

- **Controllers:**
    - **[robot_controller_example.yaml](/diffdrive_arduino/controllers/robot_controller_example.yaml)** Configuration file example for the differential controller.
- **Include:**
    - **[arduino_comms.h](/diffdrive_arduino/include/diffdrive_arduino/arduino_comms.h)** Definition for the serial connection and prototypes for methods to connect with the Arduino, read the encoders and update motor PWM/controller.
    - **[config.h](/diffdrive_arduino/include/diffdrive_arduino/config.h)** Set parameters for the joint, port, baudrate and encoder count.
    - **[diffdrive_arduino.h](/diffdrive_arduino/include/diffdrive_arduino/diffdrive_arduino.h)** Create the controller class and defines the prototypes required to be compatible with the ros2_control and ros2_controller definitions for the Differential Controller.
    - **[wheel.h](/diffdrive_arduino/include/diffdrive_arduino/wheel.h)** Define a wheel object for considering pos, velocity, effor and counts and add prototypes for the set up and encoder routine.
- **Source:**
    - **[arduino_comms.cpp](/diffdrive_arduino/src/arduino_comms.cpp)** Implementations to connect via serial with the Arduino and obtain encoder lecture, set PWM/reference for motors and related.
    - **[diffdrive_arduino.cpp](/diffdrive_arduino/src/diffdrive_arduino.cpp)** Plugin creation that implements the required function to be compatible with the Differential Controller while using the other source definitions for reading/writting data to the Arduino and the components attached to it.
    - **[wheel.cpp](/diffdrive_arduino/src/wheel.cpp)** Implements the set up and reading of the encoders.

