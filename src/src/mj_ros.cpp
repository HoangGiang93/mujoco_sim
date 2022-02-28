#include "mj_ros.h"

int MjRos::n_dof = 0;
ros::Time MjRos::ros_start;

MjRos::MjRos()
{
}

void MjRos::init(ros::NodeHandle &n)
{
    ros_start = ros::Time().now();
    if (!n.getParam("/mujoco/init_positions", MjSim::q_inits))
    {
        mju_warning("Couldn't find states in /mujoco/init_positions, set default to 0");
    }

    if (!n.getParam("/mujoco/joint_trajectory_controller/joints", MjSim::q_names))
    {
        mju_error("Couldn't find joints in /mujoco/joint_trajectory_controller/joints");
    }

    if (!n.getParam("/mujoco/joint_trajectory_controller/controller_gains/Kp", MjSim::Kp) || MjSim::Kp.size() != MjSim::q_names.size())
    {
        mju_warning_i("Couldn't find enough Kp in /mujoco/joint_trajectory_controller/controller_gains/Kp, set to default (2000 x %d)", MjSim::q_names.size());
        MjSim::Kp = std::vector<double>(MjSim::q_names.size(), 2000);
    }
    if (!n.getParam("/mujoco/joint_trajectory_controller/controller_gains/Kv", MjSim::Kv))
    {
        mju_warning_i("Couldn't find enough Kv in /mujoco/joint_trajectory_controller/controller_gains/Kv, set to default (50 x %d)", MjSim::q_names.size());
        MjSim::Kv = std::vector<double>(MjSim::q_names.size(), 50);
    }
    if (!n.getParam("/mujoco/joint_trajectory_controller/controller_gains/Ki", MjSim::Ki))
    {
        mju_warning_i("Couldn't find enough Ki in /mujoco/joint_trajectory_controller/controller_gains/Ki, set to default (100 x %d)", MjSim::q_names.size());
        MjSim::Ki = std::vector<double>(MjSim::q_names.size(), 100);
    }
    

    joint_trajectory_points_sub = n.subscribe("mujoco/joint_trajectory_points", 10, &MjRos::set_joint_trajectory_point_callback, this);
    joint_state_pub = n.advertise<sensor_msgs::JointState>("mujoco/joint_states", 10);
    follow_joint_traj_feedback_pub = n.advertise<control_msgs::FollowJointTrajectoryFeedback>("mujoco/follow_joint_trajectory/feedback", 10);

    joint_state_msg = sensor_msgs::JointState();
    if (!n.getParam("/mujoco/joint_state_publisher/joints", joint_state_msg.name))
    {
        mju_error("Couldn't find joints in /mujoco/joint_state_publisher/joints");
    }
    std::size_t joint_state_msg_size = joint_state_msg.name.size();
    joint_state_msg.position = std::vector<double>(joint_state_msg_size, 0.);
    joint_state_msg.velocity = std::vector<double>(joint_state_msg_size, 0.);
    joint_state_msg.effort = std::vector<double>(joint_state_msg_size, 0.);
    if (!n.getParam("/mujoco/joint_state_publisher/publish_rate", joint_state_pub_rate))
    {
        joint_state_pub_rate = 60;
    }
    
    follow_joint_traj_feedback_msg = control_msgs::FollowJointTrajectoryFeedback();
    if (!n.getParam("/mujoco/follow_joint_trajectory_feedback_publisher/joints", follow_joint_traj_feedback_msg.joint_names))
    {
        mju_error("Couldn't find joints in /mujoco/follow_joint_trajectory_feedback_publisher/joints");
    }
    std::size_t follow_joint_traj_feedback_msg_size = follow_joint_traj_feedback_msg.joint_names.size();
    follow_joint_traj_feedback_msg.desired.positions = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.desired.velocities = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.desired.accelerations = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.desired.effort = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.actual.positions = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.actual.velocities = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.actual.accelerations = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.actual.effort = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.error.positions = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.error.velocities = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.error.accelerations = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    follow_joint_traj_feedback_msg.error.effort = std::vector<double>(follow_joint_traj_feedback_msg_size, 0.);
    if (!n.getParam("/mujoco/follow_joint_trajectory_feedback_publisher/publish_rate", follow_joint_traj_feedback_pub_rate))
    {
        follow_joint_traj_feedback_pub_rate = 10;
    }
}

void MjRos::publish_joint_state()
{
    ros::Rate loop_rate(joint_state_pub_rate);
    int idx;
    while (ros::ok())
    {
        mtx.lock();
        mj_inverse(m, d);
        mtx.unlock();
        joint_state_msg.header.seq += 1;
        joint_state_msg.header.stamp = ros::Time().now();
        std::size_t i = 0;
        for (const std::string name : joint_state_msg.name)
        {
            idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, name.c_str());
            joint_state_msg.position[i] = d->qpos[idx];
            joint_state_msg.velocity[i] = d->qvel[idx];
            joint_state_msg.effort[i] = d->qfrc_inverse[idx];
            i++;
        }

        joint_state_pub.publish(joint_state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_follow_joint_traj_feedback()
{
    ros::Rate loop_rate(follow_joint_traj_feedback_pub_rate);
    int idx;
    while (ros::ok())
    {
        mtx.lock();
        mj_inverse(m, d);
        mtx.unlock();
        follow_joint_traj_feedback_msg.header.seq += 1;
        ros::Time now = ros::Time().now();
        ros::Duration duration = now - ros_start;
        follow_joint_traj_feedback_msg.header.stamp = now;
        std::size_t i = 0;
        for (const std::string name : follow_joint_traj_feedback_msg.joint_names)
        {
            idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, name.c_str());
            follow_joint_traj_feedback_msg.desired.positions[i] = MjSim::q_refs[name];
            follow_joint_traj_feedback_msg.desired.velocities[i] = MjSim::dq_refs[name];
            follow_joint_traj_feedback_msg.desired.accelerations[i] = MjSim::ddq_refs[name];
            follow_joint_traj_feedback_msg.desired.effort[i] = d->qfrc_applied[idx];
            follow_joint_traj_feedback_msg.desired.time_from_start = duration;
            follow_joint_traj_feedback_msg.actual.positions[i] = d->qpos[idx];
            follow_joint_traj_feedback_msg.actual.velocities[i] = d->qvel[idx];
            follow_joint_traj_feedback_msg.actual.accelerations[i] = d->qacc[idx];
            follow_joint_traj_feedback_msg.actual.effort[i] = d->qfrc_inverse[idx];
            follow_joint_traj_feedback_msg.actual.time_from_start = duration;
            follow_joint_traj_feedback_msg.error.positions[i] = MjSim::q_refs[name] - d->qpos[idx];
            follow_joint_traj_feedback_msg.error.velocities[i] = MjSim::dq_refs[name] - d->qvel[idx];
            follow_joint_traj_feedback_msg.error.accelerations[i] = MjSim::q_refs[name] - d->qacc[idx];
            follow_joint_traj_feedback_msg.error.effort[i] = d->qfrc_applied[idx] - d->qfrc_inverse[idx];
            follow_joint_traj_feedback_msg.error.time_from_start = duration;
            i++;
        }

        follow_joint_traj_feedback_pub.publish(follow_joint_traj_feedback_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::set_joint_trajectory_point_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{   
    std::size_t i = 0;
    for (const std::string name : MjSim::q_names)
    {
        MjSim::q_refs[name] = msg->positions[i];
        MjSim::dq_refs[name] = msg->velocities[i];
        MjSim::ddq_refs[name] = msg->accelerations[i];
        i++;
    }
}