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

## Installation
1) Create a workspace
```
source /opt/ros/<ros-version>/setup.bash    # source ROS
mkdir -p ~/mujoco_ws/src                    # create directory for workspace
```
2) Initialize the workspace from this [file](https://raw.githubusercontent.com/HoangGiang93/mujoco_ws/main/noetic.rosinstall) and update the workspace
```
wstool init ~/mujoco_ws/src                 # initialize .rosinstall
wstool merge -t ~/mujoco_ws/src https://raw.githubusercontent.com/HoangGiang93/mujoco_ws/main/hackathon.rosinstall
wstool update -t ~/mujoco_ws/src            # pull the repositories
```
3) Install dependency of all packages in the workspace
```
rosdep install --ignore-src --from-paths ~/mujoco_ws/src/mujoco_sim/ ~/mujoco_ws/src/mujoco_msgs/ # install dependencies available through apt
```
4) Build packages
```
cd ~/mujoco_ws                              # go to the workspace directory
catkin_make                                 # build packages (or catkin build)
source ~/mujoco_ws/devel/setup.bash         # source new overlay
```
5) Install some useful ROS packages for control and GUI
```
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-joint-trajectory-controller
sudo apt install ros-noetic-rqt-joint-trajectory-controller
sudo apt install ros-noetic-rqt-robot-steering
```

## Troubleshooting
1) `ERROR: gladLoadGL error`
- Solution: `sudo apt install nvidia-driver-515`

## Software architecture
![Picture](docs/html/mj__main_8cpp__incl.png)
