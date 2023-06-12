# mujoco_sim
ROS interface for MuJoCo simulator

## Overview

[![Watch the video]()](https://user-images.githubusercontent.com/64316740/159088750-e9e4d239-81d0-4d99-bae5-8b5e348bfe07.mp4)

[![Watch the video]()](https://user-images.githubusercontent.com/64316740/216793375-0a9a7e2e-0f4e-4d19-b8ce-2a7f5fc23a6c.mp4)

## Key Features
- Incorporates an advanced physics engine sourced from https://mujoco.org/
- Supports import and export of URDF and MJCF formats
- Enables exporting of simulation results in the USD format
- Integrates controller interfaces, controller managers, and hardware interfaces sourced from http://wiki.ros.org/ros_control
- Implements PD computed-torque control for enhanced stability
- Enables spawning and destruction of objects during run-time via rosservice
- Synchronizes simulation time with real time, with the ability to adjust simulation time for faster or slower execution
- Provides comprehensive visualization of MuJoCo elements within rviz
- Offers velocity control for the base
- Supports mimic joints from URDF format

## Troubleshooting
1) `ERROR: gladLoadGL error`
- Solution: `sudo apt install nvidia-driver-515`

## Software architecture
![Picture](docs/html/mj__main_8cpp__incl.png)
