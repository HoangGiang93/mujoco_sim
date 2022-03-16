#include "mj_ros.h"

void MjRos::init()
{
    n = ros::NodeHandle();

    if (!n.getParam("object_names", object_names))
    {
        mju_warning_s("Couldn't find objects in %s/object_names", n.getNamespace().c_str());
    }
}

void MjRos::tick()
{
    geometry_msgs::TransformStamped transform;
    for (const std::string object_name : object_names)
    {
        const int idx = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = object_name;
        transform.transform.translation.x = d->xpos[3*idx];
        transform.transform.translation.y = d->xpos[3*idx+1];
        transform.transform.translation.z = d->xpos[3*idx+2];
        transform.transform.rotation.x = d->xquat[4*idx];
        transform.transform.rotation.y = d->xquat[4*idx+1];
        transform.transform.rotation.z = d->xquat[4*idx+2];
        transform.transform.rotation.w = d->xquat[4*idx+3];
        br.sendTransform(transform);
    } 
}