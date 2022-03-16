#pragma once

#include "mj_sim.h"

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

class MjHWInterface : public hardware_interface::RobotHW
{
public:
    MjHWInterface();

    ~MjHWInterface();

public:
    void read();

    void write();

private:
    ros::NodeHandle n;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    // hardware_interface::PositionJointInterface position_joint_interface;
    // hardware_interface::VelocityJointInterface velocity_joint_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

private:
    // States
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    // Commands
    // std::vector<double> joint_positions_command;
    // std::vector<double> joint_velocities_command;
    std::vector<double> joint_efforts_command;
};