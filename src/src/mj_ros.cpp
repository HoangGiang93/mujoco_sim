#include "mj_ros.h"

int MjRos::n_dof = 0;
ros::Time MjRos::ros_start;

MjRos::MjRos()
{
}

void MjRos::init(ros::NodeHandle &n)
{
    ros_start = ros::Time().now();

    if (!n.getParam("/mujoco/n_dof", n_dof))
    {
        n_dof = 7;
    }

    std::vector<std::string> joint_names;
    if (!n.getParam("/mujoco/joint_trajectory_controller/joints", joint_names))
    {
        joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    }
    for (std::string joint_name : joint_names)
    {
        MjSim::q_idx.push_back(mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str()));
    }

    if (!n.getParam("/mujoco/joint_trajectory_controller/controller_gains/Kp", MjSim::Kp))
    {
        MjSim::Kp = std::vector<double>(n_dof, 2000);
    }
    if (!n.getParam("/mujoco/joint_trajectory_controller/controller_gains/Kv", MjSim::Kv))
    {
        MjSim::Kv = std::vector<double>(n_dof, 50);
    }
    if (!n.getParam("/mujoco/joint_trajectory_controller/controller_gains/Ki", MjSim::Ki))
    {
        MjSim::Ki = std::vector<double>(n_dof, 100);
    }
    

    joint_trajectory_points_sub = n.subscribe("mujoco/joint_trajectory_points", 10, &MjRos::set_joint_trajectory_point_callback, this);
    joint_state_pub = n.advertise<sensor_msgs::JointState>("mujoco/joint_states", 10);
    follow_joint_traj_feedback_pub = n.advertise<control_msgs::FollowJointTrajectoryFeedback>("mujoco/follow_joint_trajectory/feedback", 10);

    joint_state_msg = sensor_msgs::JointState();
    
    if (!n.getParam("/mujoco/joint_state_publisher/joints", joint_names))
    {
        joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    }
    for (std::string joint_name : joint_names)
    {
        joint_state_pub_idx.push_back(mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str()));
    }
    joint_state_msg.name = joint_names;
    joint_state_msg.position = std::vector<double>(joint_names.size(), 0.);
    joint_state_msg.velocity = std::vector<double>(joint_names.size(), 0.);
    joint_state_msg.effort = std::vector<double>(joint_names.size(), 0.);
    if (!n.getParam("/mujoco/joint_state_publisher/publish_rate", joint_state_pub_rate))
    {
        joint_state_pub_rate = 60;
    }
    
    follow_joint_traj_feedback_msg = control_msgs::FollowJointTrajectoryFeedback();
    if (!n.getParam("/mujoco/follow_joint_trajectory_feedback_publisher/joints", joint_names))
    {
        joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    }
    for (std::string joint_name : joint_names)
    {
        follow_joint_traj_feedback_q_idx.push_back(mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str()));
    }
    follow_joint_traj_feedback_msg.joint_names = joint_names;
    follow_joint_traj_feedback_msg.desired.positions = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.desired.velocities = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.desired.accelerations = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.desired.effort = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.actual.positions = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.actual.velocities = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.actual.accelerations = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.actual.effort = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.error.positions = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.error.velocities = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.error.accelerations = std::vector<double>(joint_names.size(), 0.);
    follow_joint_traj_feedback_msg.error.effort = std::vector<double>(joint_names.size(), 0.);
    if (!n.getParam("/mujoco/follow_joint_trajectory_feedback_publisher/publish_rate", follow_joint_traj_feedback_pub_rate))
    {
        follow_joint_traj_feedback_pub_rate = 10;
    }
}

void MjRos::publish_joint_state()
{
    ros::Rate loop_rate(joint_state_pub_rate);
    while (ros::ok())
    {
        mtx.lock();
        mj_inverse(m, d);
        mtx.unlock();
        joint_state_msg.header.seq += 1;
        joint_state_msg.header.stamp = ros::Time().now();
        for (int i = 0; i < joint_state_pub_idx.size(); i++)
        {
            joint_state_msg.position[i] = d->qpos[joint_state_pub_idx[i]];
            joint_state_msg.velocity[i] = d->qvel[joint_state_pub_idx[i]];
            joint_state_msg.effort[i] = d->qfrc_inverse[joint_state_pub_idx[i]];
        }

        joint_state_pub.publish(joint_state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_follow_joint_traj_feedback()
{
    ros::Rate loop_rate(follow_joint_traj_feedback_pub_rate);
    while (ros::ok())
    {
        mtx.lock();
        mj_inverse(m, d);
        mtx.unlock();
        follow_joint_traj_feedback_msg.header.seq += 1;
        ros::Time now = ros::Time().now();
        ros::Duration duration = now - ros_start;
        follow_joint_traj_feedback_msg.header.stamp = now;
        for (int i = 0; i < follow_joint_traj_feedback_q_idx.size(); i++)
        {
            follow_joint_traj_feedback_msg.desired.positions[i] = MjSim::q_ref[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.desired.velocities[i] = MjSim::dq_ref[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.desired.accelerations[i] = MjSim::ddq_ref[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.desired.effort[i] = d->qfrc_applied[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.desired.time_from_start = duration;
            follow_joint_traj_feedback_msg.actual.positions[i] = d->qpos[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.actual.velocities[i] = d->qvel[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.actual.accelerations[i] = d->qacc[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.actual.effort[i] = d->qfrc_inverse[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.actual.time_from_start = duration;
            follow_joint_traj_feedback_msg.error.positions[i] = MjSim::q_ref[follow_joint_traj_feedback_q_idx[i]] - d->qpos[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.error.velocities[i] = MjSim::dq_ref[follow_joint_traj_feedback_q_idx[i]] - d->qvel[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.error.accelerations[i] = MjSim::q_ref[follow_joint_traj_feedback_q_idx[i]] - d->qacc[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.error.effort[i] = d->qfrc_applied[follow_joint_traj_feedback_q_idx[i]] - d->qfrc_inverse[follow_joint_traj_feedback_q_idx[i]];
            follow_joint_traj_feedback_msg.error.time_from_start = duration;
        }

        follow_joint_traj_feedback_pub.publish(follow_joint_traj_feedback_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::set_joint_trajectory_point_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{
    for (int i = 0; i < n_dof; i++)
    {
        MjSim::q_ref[i] = msg->positions[i];
        MjSim::dq_ref[i] = msg->velocities[i];
        MjSim::ddq_ref[i] = msg->accelerations[i];
    }
}