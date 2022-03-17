#pragma once

#include "mj_sim.h"
#include "mujoco_msgs/ModelState.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

class MjRos
{
public:
    MjRos() = default;

public:
    void init();

    void update();

private:
    void object_gen_callback(const mujoco_msgs::ModelState& msg);

    void publish_markers(int body_idx, std::string object_name);

    void publish_tf(int body_idx, std::string object_name);

private:
    ros::NodeHandle n;

    ros::Subscriber object_gen_sub;

    visualization_msgs::Marker marker;

    ros::Publisher vis_pub;

    geometry_msgs::TransformStamped transform;

    tf2_ros::TransformBroadcaster br;
};