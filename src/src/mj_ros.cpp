#include "mj_ros.h"
#include <algorithm>

void MjRos::init()
{
    n = ros::NodeHandle();
    if (!n.getParam("joint_names", MjSim::joint_names))
    {
        mju_error_s("Couldn't find joint_names in %s/joint_names", n.getNamespace().c_str());
    }
    if (!n.getParam("init_positions", MjSim::q_inits))
    {
        mju_warning_s("Couldn't find joints and positions in %s/init_positions, set default to 0", n.getNamespace().c_str());
    }

    int joint_idx;
    std::string link_name;
    for (const std::string joint_name : MjSim::joint_names)
    {
        joint_idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
        link_name = mj_id2name(m, mjtObj::mjOBJ_BODY, m->jnt_bodyid[joint_idx]);
        link_names.push_back(link_name);
    }
}

void MjRos::update()
{
    ros::Rate loop_rate(60);
    while (ros::ok())
    {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        std::string object_name;
        for (int idx = 1; idx < m->nbody; idx++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, idx);
            if(std::find(link_names.begin(), link_names.end(), object_name) == link_names.end()) 
            {
                transform.child_frame_id = object_name;
                transform.transform.translation.x = d->xpos[3*idx];
                transform.transform.translation.y = d->xpos[3*idx+1];
                transform.transform.translation.z = d->xpos[3*idx+2];
                transform.transform.rotation.x = d->xquat[4*idx+1];
                transform.transform.rotation.y = d->xquat[4*idx+2];
                transform.transform.rotation.z = d->xquat[4*idx+3];
                transform.transform.rotation.w = d->xquat[4*idx];
                br.sendTransform(transform);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}