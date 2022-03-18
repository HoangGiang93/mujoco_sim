#include "mj_ros.h"

#include <algorithm>
#include <ros/package.h>
#include <tinyxml2.h>

ros::Time MjRos::ros_start;

MjRos::~MjRos()
{
    object_gen_sub.shutdown();
}

void MjRos::init()
{
    ros_start = ros::Time::now();

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
        MjSim::link_names.push_back(link_name);
    }

    object_gen_sub = n.subscribe("create_object", 1, &MjRos::object_gen_callback, this);
}

void MjRos::object_gen_callback(const mujoco_msgs::ModelState &msg)
{
    // Create add.xml
    std::string path = ros::package::getPath("mujoco_sim");
    tinyxml2::XMLDocument object_xml_doc;
    tinyxml2::XMLNode *root = object_xml_doc.NewElement("mujoco");
    object_xml_doc.InsertFirstChild(root);

    tinyxml2::XMLElement *worldbody_element = object_xml_doc.NewElement("worldbody");
    tinyxml2::XMLElement *body_element = object_xml_doc.NewElement("body");
    tinyxml2::XMLElement *joint_element = object_xml_doc.NewElement("joint");
    tinyxml2::XMLElement *geom_element = object_xml_doc.NewElement("geom");

    body_element->InsertEndChild(joint_element);
    body_element->InsertEndChild(geom_element);
    worldbody_element->InsertEndChild(body_element);
    root->InsertEndChild(worldbody_element);

    joint_element->SetAttribute("type", "free");

    switch (msg.type)
    {
    case mujoco_msgs::ModelState::CUBE:
        geom_element->SetAttribute("type", "box");
        geom_element->SetAttribute("size",
                                   (std::to_string(msg.scale.x) + " " +
                                    std::to_string(msg.scale.y) + " " +
                                    std::to_string(msg.scale.z))
                                       .c_str());
        break;

    case mujoco_msgs::ModelState::SPHERE:
        geom_element->SetAttribute("type", "sphere");
        geom_element->SetAttribute("size", msg.scale.x);
        break;

    default:
        break;
    }

    geom_element->SetAttribute("rgba",
                               (std::to_string(msg.color.r) + " " +
                                std::to_string(msg.color.g) + " " +
                                std::to_string(msg.color.b) + " " +
                                std::to_string(msg.color.a))
                                   .c_str());

    body_element->SetAttribute("name", msg.name.c_str());

    body_element->SetAttribute("pos",
                               (std::to_string(msg.pose.position.x) + " " +
                                std::to_string(msg.pose.position.y) + " " +
                                std::to_string(msg.pose.position.z))
                                   .c_str());

    body_element->SetAttribute("quat",
                               (std::to_string(msg.pose.orientation.w) + " " +
                                std::to_string(msg.pose.orientation.x) + " " +
                                std::to_string(msg.pose.orientation.y) + " " +
                                std::to_string(msg.pose.orientation.z))
                                   .c_str());

    object_xml_doc.SaveFile((path + "/model/tmp/add.xml").c_str());

    MjSim::add_data();
}

void MjRos::update()
{
    ros::Rate loop_rate(60);
    vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::ADD;

    transform.header.frame_id = "map";
    while (ros::ok())
    {
        std::string object_name;
        for (int body_idx = 1; body_idx < m->nbody; body_idx++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_idx);
            if (std::find(MjSim::link_names.begin(), MjSim::link_names.end(), object_name) == MjSim::link_names.end())
            {
                publish_markers(body_idx, object_name);

                publish_tf(body_idx, object_name);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_markers(int body_idx, std::string object_name)
{
    int geom_idx = m->body_geomadr[body_idx];
    if (geom_idx != -1)
    {
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

        marker.header.stamp = ros::Time::now();
        marker.ns = object_name;
        marker.id = body_idx;
        marker.color.a = m->geom_rgba[4 * geom_idx + 3];
        marker.color.r = m->geom_rgba[4 * geom_idx];
        marker.color.g = m->geom_rgba[4 * geom_idx + 1];
        marker.color.b = m->geom_rgba[4 * geom_idx + 2];
        marker.scale.x = m->geom_size[3 * geom_idx] * 2;
        marker.scale.y = m->geom_size[3 * geom_idx + size_idx[0]] * 2;
        marker.scale.z = m->geom_size[3 * geom_idx + size_idx[1]] * 2;
        marker.pose.position.x = d->xpos[3 * body_idx];
        marker.pose.position.y = d->xpos[3 * body_idx + 1];
        marker.pose.position.z = d->xpos[3 * body_idx + 2];
        marker.pose.orientation.x = d->xquat[4 * body_idx + 1];
        marker.pose.orientation.y = d->xquat[4 * body_idx + 2];
        marker.pose.orientation.z = d->xquat[4 * body_idx + 3];
        marker.pose.orientation.w = d->xquat[4 * body_idx];
        vis_pub.publish(marker);
    }
}

void MjRos::publish_tf(int body_idx, std::string object_name)
{
    transform.header.stamp = ros::Time::now();

    transform.child_frame_id = object_name;
    transform.transform.translation.x = d->xpos[3 * body_idx];
    transform.transform.translation.y = d->xpos[3 * body_idx + 1];
    transform.transform.translation.z = d->xpos[3 * body_idx + 2];
    transform.transform.rotation.x = d->xquat[4 * body_idx + 1];
    transform.transform.rotation.y = d->xquat[4 * body_idx + 2];
    transform.transform.rotation.z = d->xquat[4 * body_idx + 3];
    transform.transform.rotation.w = d->xquat[4 * body_idx];
    br.sendTransform(transform);
}