#pragma once

#include "mj_sim.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class MjRos
{
public:
    MjRos() = default;

public:
    void init();

    void update();

private:
    ros::NodeHandle n;

    tf2_ros::TransformBroadcaster br;

    // Sensor data
    std::vector<std::string> link_names;
};