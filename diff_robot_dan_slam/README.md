# Diff Robot Dan SLAM (Simultatineous Localization and Mapping) Package

![slam_sim_gif](/diff_robot_dan_slam/resources/gazebo_slam_toolbox_run.gif)

![slam_real_gif](/diff_robot_dan_slam/resources/room_slam.gif)

## Overview

Oriented to use the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) package with the Diff Robot Dan to generate maps and locate inside them. 

## Usage

### Online Asynchronous Mapping:

By considering the params present in the [onlin_async_mapper_config.yaml](/diff_robot_dan_slam/config/online_async_mapper_config.yaml) launches the Online Asynchronous Mode of the **slam_toolbox**, so the Diff Robot Dan can generate maps in real life and simulated environments, here do not forget to set the **use_sim_time** argument in the corresponding case, if you want to try it in a Gazebo world, you can use the laberynth in the *diff_robot_dan_gazebo* package:

```bash
ros2 launch diff_robot_dan_gazebo robot_in_world.launch.py # Terminal 1
ros2 launch diff_robot_dan_slam online_async_mapper.launch.py use_sim_time:=true # Terminal 2
```

If you want to use it with the real robot, make sure you have run the bringup in the robot pi, and in your PC run:

```bash
ros2 launch diff_robot_dan_slam online_async_mapper.launch.py
```


