#pragma once

#include "mj_sim.h"

// ROS Controls
#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

class MjHWInterface : public hardware_interface::RobotHW
{
public:
    MjHWInterface();

    ~MjHWInterface();

public:
    void read(const ros::Time &, const ros::Duration &period) override;

    void write(const ros::Time &, const ros::Duration &period) override;

private:
    ros::NodeHandle n;

    // Hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    std::map<std::string, hardware_interface::EffortJointInterface> effort_joint_interfaces;

private:
    std::size_t num_joints;

    // States
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    // Commands
    std::vector<double> joint_positions_command;
    std::vector<double> joint_velocities_command;
    std::vector<double> joint_efforts_command;
};