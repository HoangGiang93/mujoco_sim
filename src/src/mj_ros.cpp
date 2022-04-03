// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mj_ros.h"

#include <algorithm>
#include <ros/package.h>
#include <tinyxml2.h>

ros::Time MjRos::ros_start;

std::string root_name;

MjRos::~MjRos()
{
}

void MjRos::init()
{
    ros_start = ros::Time::now();

    int joint_idx;
    std::string link_name;
    for (const std::string joint_name : MjSim::joint_names)
    {
        joint_idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
        link_name = mj_id2name(m, mjtObj::mjOBJ_BODY, m->jnt_bodyid[joint_idx]);
        MjSim::link_names.push_back(link_name);
    }

    n = ros::NodeHandle();
    object_gen_sub = n.subscribe("create_object", 1, &MjRos::object_gen_callback, this);

    if (use_odom_joints)
    {
        cmd_vel_sub = n.subscribe("cmd_vel", 10, &MjRos::cmd_vel_callback, this);
    }

    urdf::Model urdf_model;
    init_urdf(urdf_model, n); // this looks so retared...
    root_name = urdf_model.getRoot()->name;
    for (const std::pair<std::string, urdf::JointSharedPtr> &joint : urdf_model.joints_)
    {
        if (joint.second->mimic != nullptr)
        {
            MimicJoint mimic_joint;
            mimic_joint.from_joint = joint.second->mimic->joint_name;
            mimic_joint.multiplier = joint.second->mimic->multiplier;
            mimic_joint.offset = joint.second->mimic->offset;
            MjSim::mimic_joints[joint.first] = mimic_joint;
        }
    }
}

void MjRos::cmd_vel_callback(const geometry_msgs::Twist &msg)
{
    const std::string odom_z_joint_name = MjSim::odom_joints["odom_z_joint"].first;
    const int odom_z_joint_idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, odom_z_joint_name.c_str());
    const mjtNum odom_z_joint_pos = d->qpos[odom_z_joint_idx];

    MjSim::odom_joints["odom_x_joint"].second = msg.linear.x * mju_cos(odom_z_joint_pos) - msg.linear.y * mju_sin(odom_z_joint_pos);
    MjSim::odom_joints["odom_y_joint"].second = msg.linear.x * mju_sin(odom_z_joint_pos) + msg.linear.y * mju_cos(odom_z_joint_pos);
    MjSim::odom_joints["odom_z_joint"].second = msg.angular.z;
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

    case mujoco_msgs::ModelState::CYLINDER:
        geom_element->SetAttribute("type", "cylinder");
        geom_element->SetAttribute("size",
                                   (std::to_string(msg.scale.x) + " " +
                                    std::to_string(msg.scale.y))
                                       .c_str());
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

void MjRos::update(double frequency = 60)
{
    ros::Rate loop_rate(frequency); // Publish with 60 Hz
    vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.header.stamp = ros::Time();

    transform.header.frame_id = "map";
    while (ros::ok())
    {
        std::string object_name;
        for (int body_idx = 1; body_idx < m->nbody; body_idx++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_idx);
            if (std::find(MjSim::link_names.begin(), MjSim::link_names.end(), object_name) == MjSim::link_names.end())
            {
                if (use_odom_joints && object_name == model_path.stem().string())
                {
                    continue;
                }
                
                publish_markers(body_idx, object_name);

                publish_tf(body_idx, object_name);
            }
        }
        
        std::string base_name = "world";
        if (use_odom_joints)
        {
            base_name = model_path.stem().string();
        }
        publish_tf(mj_name2id(m, mjtObj::mjOBJ_BODY, base_name.c_str()), root_name);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_markers(int body_idx, std::string object_name)
{
    int geom_idx = m->body_geomadr[body_idx];
    if (geom_idx != -1)
    {
        switch (m->geom_type[geom_idx])
        {
        case mjtGeom::mjGEOM_BOX:
            marker.type = visualization_msgs::Marker::CUBE;
            marker.mesh_resource = "";
            marker.scale.x = m->geom_size[3 * geom_idx] * 2;
            marker.scale.y = m->geom_size[3 * geom_idx + 1] * 2;
            marker.scale.z = m->geom_size[3 * geom_idx + 2] * 2;
            break;

        case mjtGeom::mjGEOM_SPHERE:
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.mesh_resource = "";
            marker.scale.x = m->geom_size[3 * geom_idx] * 2;
            marker.scale.y = m->geom_size[3 * geom_idx] * 2;
            marker.scale.z = m->geom_size[3 * geom_idx] * 2;
            break;

        case mjtGeom::mjGEOM_CYLINDER:
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = m->geom_size[3 * geom_idx] * 2;
            marker.scale.y = m->geom_size[3 * geom_idx] * 2;
            marker.scale.z = m->geom_size[3 * geom_idx + 1] * 2;
            break;

        case mjtGeom::mjGEOM_MESH:
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://mujoco_sim/model/tmp/" + model_path.stem().string() + "/meshes/" + object_name + ".dae";
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;

        default:
            break;
        }

        marker.ns = object_name;
        marker.color.a = m->geom_rgba[4 * geom_idx + 3];
        marker.color.r = m->geom_rgba[4 * geom_idx];
        marker.color.g = m->geom_rgba[4 * geom_idx + 1];
        marker.color.b = m->geom_rgba[4 * geom_idx + 2];
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