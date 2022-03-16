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
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        std::string object_name;
        for (int idx = 1; idx < m->nbody; idx++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, idx);
            if(std::find(link_names.begin(), link_names.end(), object_name) == link_names.end()) 
            {
                int geom_idx = m->body_geomadr[idx];
                if (geom_idx != -1)
                {
                    marker.id++;
                    int size_idx[2] = {0, 0};
                    switch (m->geom_type[geom_idx])
                    {
                    case mjtGeom::mjGEOM_SPHERE:
                        marker.type = visualization_msgs::Marker::SPHERE;
                        break;

                    case mjtGeom::mjGEOM_BOX:
                        marker.type = visualization_msgs::Marker::CUBE;
                        size_idx[0] = 1;
                        size_idx[1] = 2;
                        break;
                    
                    default:
                        break;
                    }
                    marker.scale.x = m->geom_size[3*geom_idx]*2;
                    marker.scale.y = m->geom_size[3*geom_idx+size_idx[0]]*2;
                    marker.scale.z = m->geom_size[3*geom_idx+size_idx[1]]*2;
                    marker.pose.position.x = d->xpos[3*idx];
                    marker.pose.position.y = d->xpos[3*idx+1];
                    marker.pose.position.z = d->xpos[3*idx+2];
                    marker.pose.orientation.x = d->xquat[4*idx+1];
                    marker.pose.orientation.y = d->xquat[4*idx+2];
                    marker.pose.orientation.z = d->xquat[4*idx+3];
                    marker.pose.orientation.w = d->xquat[4*idx];
                    vis_pub.publish(marker);
                }

                geometry_msgs::TransformStamped transform;
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = "map";

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