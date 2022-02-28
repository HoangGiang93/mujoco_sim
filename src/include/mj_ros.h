#pragma once

#include "mj_model.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "mj_sim.h"

class MjRos
{
public:
    MjRos();

public:
    void init(ros::NodeHandle &n);

    void publish_joint_state();

    void publish_follow_joint_traj_feedback();

private:
    void set_joint_trajectory_point_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);

public:
    static int n_dof;
    static ros::Time ros_start;

private:
    ros::Subscriber joint_trajectory_points_sub;

    ros::Publisher joint_state_pub;
    double joint_state_pub_rate;

    ros::Publisher follow_joint_traj_feedback_pub;
    double follow_joint_traj_feedback_pub_rate;

    sensor_msgs::JointState joint_state_msg;
    control_msgs::FollowJointTrajectoryFeedback follow_joint_traj_feedback_msg;
};