# mujoco_sim
ROS interface for Mujoco simulator

## Overview
[![Watch the video](img/img.png)](https://user-images.githubusercontent.com/64316740/159088750-e9e4d239-81d0-4d99-bae5-8b5e348bfe07.mp4)

## Features
- Advanced physics engine from https://mujoco.org/
- Import and export of **URDF** and **MJCF**
- Integration of controller interfaces, controller managers and hardware interfaces from http://wiki.ros.org/ros_control
- Integrate PD computed-torque control to ensure stability
- Spawn objects in run-time
- Synchronize simulation time and real time (the simulation time can also be set to speed up or slow down)
- Visualize everything from Mujoco to rviz
