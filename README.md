# mujoco_sim
ROS interface for MuJoCo simulator

## Overview

[![Watch the video]()](https://user-images.githubusercontent.com/64316740/159088750-e9e4d239-81d0-4d99-bae5-8b5e348bfe07.mp4)

[![Watch the video]()](https://user-images.githubusercontent.com/64316740/216793375-0a9a7e2e-0f4e-4d19-b8ce-2a7f5fc23a6c.mp4)

## Features
- Advanced physics engine from https://mujoco.org/
- Import and export of **URDF** and **MJCF**
- Integration of controller interfaces, controller managers and hardware interfaces from http://wiki.ros.org/ros_control
- Integrate PD computed-torque control to ensure stability
- Spawn objects and destroy objects in run-time using rosservice
- Synchronize simulation time and real time (the simulation time can also be set to speed up or slow down)
- Visualize everything from MuJoCo to rviz
- Provide velocity controller for the base
- Support mimic joints from **URDF**

## Installation
1) Create a workspace
```
source /opt/ros/<ros-version>/setup.bash    # source ROS
mkdir -p ~/mujoco_ws/src                    # create directory for workspace
```
2) Initialize the workspace from this [file](https://raw.githubusercontent.com/HoangGiang93/mujoco_ws/main/noetic.rosinstall) and update the workspace
```
wstool init ~/mujoco_ws/src                 # initialize .rosinstall
wstool merge -t ~/mujoco_ws/src https://raw.githubusercontent.com/HoangGiang93/mujoco_ws/main/noetic.rosinstall
wstool update -t ~/mujoco_ws/src            # pull the repositories
```
3) Install dependency of all packages in the workspace
```
rosdep install --ignore-src --from-paths ~/mujoco_ws/src/mujoco_sim/ ~/mujoco_ws/src/mujoco_msgs/ ~/mujoco_ws/src/mujoco_world/  # install dependencies available through apt
```
4) Build packages
```
cd ~/mujoco_ws                              # go to the workspace directory
catkin_make                                 # build packages (or catkin build)
source ~/mujoco_ws/devel/setup.bash         # source new overlay
```

## Troubleshooting
1) `ERROR: gladLoadGL error`
- Solution: `sudo apt install nvidia-driver-515`

## Software architecture
![Picture](docs/html/mj__main_8cpp__incl.png)
