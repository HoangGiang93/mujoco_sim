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
#include <condition_variable>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <ros/package.h>
#include <tinyxml2.h>

using namespace std::chrono_literals;

ros::Time MjRos::ros_start;

std::vector<std::string> root_names;

double pub_object_marker_array_rate;
double pub_object_tf_rate;
double pub_object_state_array_rate;
double pub_world_joint_state_rate;
double pub_base_pose_rate;
double pub_sensor_data_rate;
double spawn_and_destroy_objects_rate;
int spawn_object_count_per_cycle;

visualization_msgs::Marker marker;
visualization_msgs::MarkerArray marker_array;
std::vector<nav_msgs::Odometry> base_poses;
sensor_msgs::JointState world_joint_states;
mujoco_msgs::ObjectStateArray object_state_array;

std::condition_variable condition;

int spawn_nr = 0;
std::mutex spawn_mtx;
std::vector<mujoco_msgs::ObjectStatus> objects_to_spawn;
bool spawn_success;

int destroy_nr = 0;
std::mutex destroy_mtx;
std::vector<std::string> object_names_to_destroy;
bool destroy_success;

CmdVelCallback::CmdVelCallback(const size_t in_id, const std::string &in_robot) : id(in_id), robot(in_robot)
{
}

void CmdVelCallback::callback(const geometry_msgs::Twist &msg)
{
    MjSim::odom_vels[robot + "_lin_odom_x_joint"] = MjSim::add_odom_joints[robot]["lin_odom_x_joint"] ? msg.linear.x : 0.0;
    MjSim::odom_vels[robot + "_lin_odom_y_joint"] = MjSim::add_odom_joints[robot]["lin_odom_y_joint"] ? msg.linear.y : 0.0;
    MjSim::odom_vels[robot + "_lin_odom_z_joint"] = MjSim::add_odom_joints[robot]["lin_odom_z_joint"] ? msg.linear.z : 0.0;
    MjSim::odom_vels[robot + "_ang_odom_x_joint"] = MjSim::add_odom_joints[robot]["ang_odom_x_joint"] ? msg.angular.x : 0.0;
    MjSim::odom_vels[robot + "_ang_odom_y_joint"] = MjSim::add_odom_joints[robot]["ang_odom_y_joint"] ? msg.angular.y : 0.0;
    MjSim::odom_vels[robot + "_ang_odom_z_joint"] = MjSim::add_odom_joints[robot]["ang_odom_z_joint"] ? msg.angular.z : 0.0;

    if (pub_base_pose_rate > 1E-9)
    {
        base_poses[id].twist.twist = msg;
    }
}

MjRos::~MjRos()
{
}

void MjRos::init()
{
    n = ros::NodeHandle();

    if (!ros::param::get("~pub_object_marker_array_rate", pub_object_marker_array_rate))
    {
        pub_object_marker_array_rate = 60.0;
    }
    if (!ros::param::get("~pub_object_tf_rate", pub_object_tf_rate))
    {
        pub_object_tf_rate = 60.0;
    }
    if (!ros::param::get("~pub_object_state_array_rate", pub_object_state_array_rate))
    {
        pub_object_state_array_rate = 60.0;
    }
    if (!ros::param::get("~pub_world_joint_state_rate", pub_world_joint_state_rate))
    {
        pub_world_joint_state_rate = 0.0;
    }
    if (!ros::param::get("~pub_base_pose_rate", pub_base_pose_rate))
    {
        pub_base_pose_rate = 60.0;
    }
    if (!ros::param::get("~pub_sensor_data_rate", pub_sensor_data_rate))
    {
        pub_sensor_data_rate = 60.0;
    }
    if (!ros::param::get("~spawn_and_destroy_objects_rate", spawn_and_destroy_objects_rate))
    {
        spawn_and_destroy_objects_rate = 600.0;
    }
    if (!ros::param::get("~root_frame_id", root_frame_id))
    {
        root_frame_id = "map";
    }
    if (!ros::param::get("~spawn_object_count_per_cycle", spawn_object_count_per_cycle))
    {
        spawn_object_count_per_cycle = -1;
    }

    ros_start = ros::Time::now();

    int joint_id;
    std::string link_name;
    for (const std::string &robot : MjSim::robots)
    {
        for (const std::string joint_name : MjSim::joint_names[robot])
        {
            joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
            link_name = mj_id2name(m, mjtObj::mjOBJ_BODY, m->jnt_bodyid[joint_id]);
            MjSim::link_names.push_back(link_name);
        }
    }

    if (MjSim::robots.size() < 2)
    {
        urdf::Model urdf_model;
        if (init_urdf(urdf_model, n)) // this looks so retared...
        {
            root_names.push_back(urdf_model.getRoot()->name);
        }
        else
        {
            root_names.push_back("world");
        }
    }
    else
    {
        for (const std::string &robot : MjSim::robots)
        {
            urdf::Model urdf_model;
            if (init_urdf(urdf_model, n, (robot + "/robot_description").c_str())) // this looks so retared...
            {
                root_names.push_back(urdf_model.getRoot()->name);
            }
            else
            {
                root_names.push_back("world");
            }
        }
    }

    for (size_t i = 0; i < MjSim::robots.size(); i++)
    {
        if (MjSim::add_odom_joints[MjSim::robots[i]]["lin_odom_x_joint"] ||
            MjSim::add_odom_joints[MjSim::robots[i]]["lin_odom_y_joint"] ||
            MjSim::add_odom_joints[MjSim::robots[i]]["lin_odom_z_joint"] ||
            MjSim::add_odom_joints[MjSim::robots[i]]["ang_odom_x_joint"] ||
            MjSim::add_odom_joints[MjSim::robots[i]]["ang_odom_y_joint"] ||
            MjSim::add_odom_joints[MjSim::robots[i]]["ang_odom_z_joint"])
        {
            cmd_vel_callbacks.push_back(new CmdVelCallback(i, MjSim::robots[i]));

            cmd_vel_subs.push_back(n.subscribe("/" + MjSim::robots[i] + "/cmd_vel", 1, &CmdVelCallback::callback, cmd_vel_callbacks[i]));
        }
    }

    reset_robot_server = n.advertiseService("reset", &MjRos::reset_robot_service, this);
    ROS_INFO("Started [%s] service.", reset_robot_server.getService().c_str());

    spawn_objects_server = n.advertiseService("/mujoco/spawn_objects", &MjRos::spawn_objects_service, this);
    ROS_INFO("Started [%s] service.", spawn_objects_server.getService().c_str());

    destroy_objects_server = n.advertiseService("/mujoco/destroy_objects", &MjRos::destroy_objects_service, this);
    ROS_INFO("Started [%s] service.", destroy_objects_server.getService().c_str());

    if (!ros::param::get("~joint_inits", joint_inits))
    {
        ROS_WARN("joint_inits not found, will set to default value (0)");
    }
    if (ros::param::get("~joint_ignores", MjSim::joint_ignores))
    {
        for (const std::string &joint_ignore : MjSim::joint_ignores)
        {
            ROS_INFO("Ignore joint: %s", joint_ignore.c_str());
        }
    }

    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("/mujoco/visualization_marker_array", 0);
    for (size_t i = 0; i < MjSim::robots.size(); i++)
    {
        base_pose_pubs.push_back(n.advertise<nav_msgs::Odometry>("/" + MjSim::robots[i] + "/" + root_names[i], 0));
    }

    object_states_pub = n.advertise<mujoco_msgs::ObjectStateArray>("/mujoco/object_states", 0);
    world_joint_states_pub = n.advertise<sensor_msgs::JointState>("/mujoco/joint_states", 0);
    sensors_pub = n.advertise<geometry_msgs::Vector3Stamped>("/mujoco/sensors_3D", 0);

    reset_robot();
}

void MjRos::reset_robot()
{
    for (const std::string &robot : MjSim::robots)
    {
        for (const std::string &joint_name : MjSim::joint_names[robot])
        {
            const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
            if (joint_id != -1)
            {
                const int qpos_id = m->jnt_qposadr[joint_id];
                if (joint_inits.count(joint_name) != 0)
                {
                    d->qpos[qpos_id] = joint_inits[joint_name];
                }
                else
                {
                    d->qpos[qpos_id] = 0.f;
                }
                const int dof_id = m->jnt_dofadr[joint_id];
                d->qvel[dof_id] = 0.f;
                d->qacc[dof_id] = 0.f;
            }
        }
    }
    for (std::pair<const std::string, mjtNum> &odom_joint : MjSim::odom_vels)
    {
        odom_joint.second = 0.f;
        const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, odom_joint.first.c_str());
        if (joint_id != -1)
        {
            const int qpos_id = m->jnt_qposadr[joint_id];
            const int dof_id = m->jnt_dofadr[joint_id];
            d->qpos[qpos_id] = 0.f;
            d->qvel[dof_id] = 0.f;
            d->qacc[dof_id] = 0.f;
        }
    }
    mj_forward(m, d);
}

bool MjRos::reset_robot_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    ros::ServiceClient list_controllers_client = n.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
    controller_manager_msgs::ListControllers list_controllers_srv;
    ros::ServiceClient switch_controller_client = n.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    controller_manager_msgs::SwitchController switch_controller_srv;
    switch_controller_srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
    switch_controller_srv.request.start_asap = true;
    std::vector<std::string> controller_names;
    bool should_switch_controller = false;
    if (list_controllers_client.call(list_controllers_srv))
    {
        should_switch_controller = true;

        for (const controller_manager_msgs::ControllerState &controller_state : list_controllers_srv.response.controller)
        {
            if (controller_state.state.compare("running") == 0 && controller_state.type.find("effort_controllers") != std::string::npos)
            {
                controller_names.push_back(controller_state.name);
                ROS_INFO("Reset %s", controller_state.name.c_str());
            }
        }
    }
    if (should_switch_controller)
    {
        switch_controller_srv.request.start_controllers.clear();
        switch_controller_srv.request.stop_controllers = controller_names;
        if (!switch_controller_client.call(switch_controller_srv))
        {
            ROS_WARN("Failed to call switch_controller");
        }
    }

    mtx.lock();
    reset_robot();
    mtx.unlock();
    ros::Duration(100 * m->opt.timestep).sleep();
    float error_sum = 0.f;
    for (const std::string &robot : MjSim::robots)
    {
        for (const std::string &joint_name : MjSim::joint_names[robot])
        {
            const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
            if (joint_id != -1)
            {
                const int qpos_id = m->jnt_qposadr[joint_id];
                if (joint_inits.count(joint_name) != 0)
                {
                    error_sum += mju_abs(d->qpos[qpos_id] - joint_inits[joint_name]);
                }
                else
                {
                    error_sum += mju_abs(d->qpos[qpos_id]);
                }
            }
        }
    }
    if (error_sum < MjSim::joint_names.size() * 0.1)
    {
        res.success = true;
        res.message = "Reset successfully! (error_sum = " + std::to_string(error_sum) + ")";
    }
    else
    {
        res.success = false;
        res.message = "Failed to reset (error_sum = " + std::to_string(error_sum) + "). Did you stop the controlllers?";
    }

    if (should_switch_controller)
    {
        switch_controller_srv.request.start_controllers = controller_names;
        switch_controller_srv.request.stop_controllers.clear();
        if (!switch_controller_client.call(switch_controller_srv))
        {
            ROS_WARN("Failed to call switch_controller");
        }
    }
    return true;
}

bool MjRos::spawn_objects_service(mujoco_msgs::SpawnObjectRequest &req, mujoco_msgs::SpawnObjectResponse &res)
{
    std::vector<std::string> names;
    for (const mujoco_msgs::ObjectStatus &object : req.objects)
    {
        if (std::find(objects_to_spawn.begin(), objects_to_spawn.end(), object) == objects_to_spawn.end() &&
            mj_name2id(m, mjtObj::mjOBJ_BODY, object.info.name.c_str()) == -1)
        {
            objects_to_spawn.push_back(object);
            names.push_back(object.info.name);
        }
    }
    if (objects_to_spawn.empty())
    {
        res.names = std::vector<std::string>();
        return true;
    }

    std::unique_lock<std::mutex> lk(spawn_mtx);
    spawn_success = false;
    if (condition.wait_until(lk, std::chrono::system_clock::now() + 100ms, [&]
                             { return spawn_success; }))
    {
        res.names = names;
    }
    else
    {
        res.names = std::vector<std::string>();
    }
    return true;
}

void MjRos::spawn_objects(const std::vector<mujoco_msgs::ObjectStatus> objects)
{
    // Create add.xml
    tinyxml2::XMLDocument object_xml_doc;
    tinyxml2::XMLNode *root = object_xml_doc.NewElement("mujoco");
    object_xml_doc.LinkEndChild(root);

    tinyxml2::XMLElement *worldbody_element = object_xml_doc.NewElement("worldbody");
    root->LinkEndChild(worldbody_element);

    for (const mujoco_msgs::ObjectStatus &object : objects)
    {
        if (mj_name2id(m, mjtObj::mjOBJ_BODY, object.info.name.c_str()) != -1)
        {
            ROS_WARN("Object [%s] already exists, ignore...", object.info.name.c_str());
            continue;
        }

        tinyxml2::XMLElement *body_element = object_xml_doc.NewElement("body");
        if (object.info.movable)
        {
            tinyxml2::XMLElement *joint_element = object_xml_doc.NewElement("freejoint");
            body_element->LinkEndChild(joint_element);
        }
        else
        {
            body_element->SetAttribute("mocap", "true");
        }

        tinyxml2::XMLElement *geom_element = object_xml_doc.NewElement("geom");
        tinyxml2::XMLElement *inertial_element = object_xml_doc.NewElement("inertial");

        boost::filesystem::path object_mesh_path = object.info.mesh;
        switch (object.info.type)
        {
        case mujoco_msgs::ObjectInfo::CUBE:
            geom_element->SetAttribute("type", "box");
            geom_element->SetAttribute("size",
                                       (std::to_string(object.info.size.x) + " " +
                                        std::to_string(object.info.size.y) + " " +
                                        std::to_string(object.info.size.z))
                                           .c_str());
            break;

        case mujoco_msgs::ObjectInfo::SPHERE:
            geom_element->SetAttribute("type", "sphere");
            geom_element->SetAttribute("size", object.info.size.x);
            break;

        case mujoco_msgs::ObjectInfo::CYLINDER:
            geom_element->SetAttribute("type", "cylinder");
            geom_element->SetAttribute("size",
                                       (std::to_string(object.info.size.x) + " " +
                                        std::to_string(object.info.size.z))
                                           .c_str());
            break;

        case mujoco_msgs::ObjectInfo::MESH:
            if (boost::filesystem::exists(world_path.parent_path() / (object_mesh_path.string() + ".xml")))
            {
                object_mesh_path.replace_extension(".xml");
            }
            else if (boost::filesystem::exists(world_path.parent_path() / (world_path.stem().string() + "/meshes/" + object_mesh_path.string() + ".stl")))
            {
                object_mesh_path.replace_extension(".stl");
            }

            if (object_mesh_path.extension().compare(".xml") == 0)
            {
                object_mesh_path = world_path.parent_path() / object_mesh_path;
                tinyxml2::XMLDocument mesh_xml_doc;

                if (mesh_xml_doc.LoadFile(object_mesh_path.c_str()) != tinyxml2::XML_SUCCESS)
                {
                    mju_warning_s("Failed to load file \"%s\"\n", object_mesh_path.c_str());
                    continue;
                }
                for (tinyxml2::XMLNode *node = mesh_xml_doc.FirstChild()->FirstChild();
                     node != nullptr;
                     node = node->NextSibling())
                {
                    tinyxml2::XMLNode *copy = node->DeepClone(&object_xml_doc);
                    // Don't copy asset, it should be included in world
                    if (strcmp(copy->Value(), "asset") == 0)
                    {
                        continue;
                    }

                    if (strcmp(copy->Value(), "worldbody") == 0)
                    {
                        for (tinyxml2::XMLElement *element = copy->ToElement()->FirstChildElement();
                             element != nullptr;
                             element = element->NextSiblingElement())
                        {
                            if (strcmp(element->Value(), "body") == 0)
                            {
                                element->SetAttribute("name", object.info.name.c_str());
                                element->SetAttribute("pos", (std::to_string(object.pose.position.x) + " " +
                                                              std::to_string(object.pose.position.y) + " " +
                                                              std::to_string(object.pose.position.z))
                                                                 .c_str());
                                element->SetAttribute("quat",
                                                      (std::to_string(object.pose.orientation.w) + " " +
                                                       std::to_string(object.pose.orientation.x) + " " +
                                                       std::to_string(object.pose.orientation.y) + " " +
                                                       std::to_string(object.pose.orientation.z))
                                                          .c_str());
                                break;
                            }
                        }
                    }
                    root->InsertEndChild(copy);
                }
                continue;
            }
            else if (object_mesh_path.extension().compare(".stl") == 0)
            {
                geom_element->SetAttribute("type", "mesh");
                geom_element->SetAttribute("size",
                                           (std::to_string(object.info.size.x) + " " +
                                            std::to_string(object.info.size.y) + " " +
                                            std::to_string(object.info.size.z))
                                               .c_str());
                geom_element->SetAttribute("mesh", object_mesh_path.stem().c_str());
            }
            else
            {
                ROS_WARN("Object [%s] with extension [%s] not supported", object_mesh_path.c_str(), object_mesh_path.extension().c_str());
                continue;
            }

        default:
            break;
        }

        body_element->SetAttribute("name", object.info.name.c_str());

        body_element->SetAttribute("pos",
                                   (std::to_string(object.pose.position.x) + " " +
                                    std::to_string(object.pose.position.y) + " " +
                                    std::to_string(object.pose.position.z))
                                       .c_str());

        body_element->SetAttribute("quat",
                                   (std::to_string(object.pose.orientation.w) + " " +
                                    std::to_string(object.pose.orientation.x) + " " +
                                    std::to_string(object.pose.orientation.y) + " " +
                                    std::to_string(object.pose.orientation.z))
                                       .c_str());

        if (object.info.inertial.m != 0)
        {
            inertial_element->SetAttribute("pos", (std::to_string(object.info.inertial.com.x) + " " +
                                                   std::to_string(object.info.inertial.com.y) + " " +
                                                   std::to_string(object.info.inertial.com.z))
                                                      .c_str());

            inertial_element->SetAttribute("mass", object.info.inertial.m);

            if (object.info.inertial.ixx != 0 ||
                object.info.inertial.ixy != 0 ||
                object.info.inertial.ixz != 0 ||
                object.info.inertial.iyy != 0 ||
                object.info.inertial.iyz != 0 ||
                object.info.inertial.izz != 0)
            {
                inertial_element->SetAttribute("fullinertia", (std::to_string(object.info.inertial.ixx) + " " +
                                                               std::to_string(object.info.inertial.iyy) + " " +
                                                               std::to_string(object.info.inertial.izz) + " " +
                                                               std::to_string(object.info.inertial.ixy) + " " +
                                                               std::to_string(object.info.inertial.ixz) + " " +
                                                               std::to_string(object.info.inertial.iyz))
                                                                  .c_str());
            }

            body_element->LinkEndChild(inertial_element);
        }

        geom_element->SetAttribute("rgba",
                                   (std::to_string(object.info.rgba.r) + " " +
                                    std::to_string(object.info.rgba.g) + " " +
                                    std::to_string(object.info.rgba.b) + " " +
                                    std::to_string(object.info.rgba.a))
                                       .c_str());

        body_element->LinkEndChild(geom_element);
        worldbody_element->LinkEndChild(body_element);
    }

    if (object_xml_doc.SaveFile((tmp_model_path.parent_path() / "add.xml").c_str()) != tinyxml2::XML_SUCCESS)
    {
        mju_warning_s("Failed to save file \"%s\"\n", (tmp_model_path.parent_path() / "add.xml").c_str());
        spawn_success = false;
    }
    else
    {
        spawn_success = MjSim::add_data();
        mtx.lock();
        for (const mujoco_msgs::ObjectStatus &object : objects)
        {
            const char *name = object.info.name.c_str();
            ROS_INFO("[Spawn #%d] Try to spawn body %s", spawn_nr, name);
            int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, name);
            if (body_id != -1)
            {
                if (m->body_dofnum[body_id] != 6)
                {
                    continue;
                }

                int dof_adr = m->jnt_dofadr[m->body_jntadr[body_id]];
                d->qvel[dof_adr] = object.velocity.linear.x;
                d->qvel[dof_adr + 1] = object.velocity.linear.y;
                d->qvel[dof_adr + 2] = object.velocity.linear.z;
                d->qvel[dof_adr + 3] = object.velocity.angular.x;
                d->qvel[dof_adr + 4] = object.velocity.angular.y;
                d->qvel[dof_adr + 5] = object.velocity.angular.z;
            }
            else
            {
                ROS_WARN("Object %s not found to spawn", name);
                spawn_success = false;
            }
        }
        spawn_nr++;
        mj_forward(m, d);
        mtx.unlock();
    }
    objects_to_spawn.erase(std::remove_if(objects_to_spawn.begin(), objects_to_spawn.end(), [objects](const mujoco_msgs::ObjectStatus &object)
                                          { return std::find(objects.begin(), objects.end(), object) != objects.end(); }),
                           objects_to_spawn.end());
}

bool MjRos::destroy_objects_service(mujoco_msgs::DestroyObjectRequest &req, mujoco_msgs::DestroyObjectResponse &res)
{
    for (const std::string &object_name : req.names)
    {
        if (std::find(object_names_to_destroy.begin(), object_names_to_destroy.end(), object_name) == object_names_to_destroy.end() &&
            mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) != -1)
        {
            object_names_to_destroy.push_back(object_name);
        }
    }

    std::vector<mujoco_msgs::ObjectState> object_states;
    if (object_names_to_destroy.empty())
    {
        res.object_states = object_states;
        return true;
    }
    else
    {
        const std::size_t objects_num = object_names_to_destroy.size();
        object_states.reserve(objects_num);

        for (int i = 0; i < objects_num; i++)
        {
            const char *name = object_names_to_destroy[i].c_str();
            ROS_INFO("[Destroy #%d] Try to destroy body %s", destroy_nr, name);
            int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, name);
            if (body_id != -1)
            {
                mujoco_msgs::ObjectState object_state;
                object_state.name = name;
                if (m->body_dofnum[body_id] != 6)
                {
                    continue;
                }

                int dof_adr = m->jnt_dofadr[m->body_jntadr[body_id]];
                object_state.velocity.linear.x = d->qvel[dof_adr];
                object_state.velocity.linear.y = d->qvel[dof_adr + 1];
                object_state.velocity.linear.z = d->qvel[dof_adr + 2];
                object_state.velocity.angular.x = d->qvel[dof_adr + 3];
                object_state.velocity.angular.y = d->qvel[dof_adr + 4];
                object_state.velocity.angular.z = d->qvel[dof_adr + 5];

                object_states.push_back(object_state);
            }
            else
            {
                ROS_WARN("Object %s not found to destroy", name);
            }
        }
        destroy_nr++;
    }

    std::unique_lock<std::mutex> lk(destroy_mtx);

    destroy_success = false;
    if (condition.wait_until(lk, std::chrono::system_clock::now() + 100ms, [&]
                             { return destroy_success; }))
    {
        res.object_states = object_states;
    }
    else
    {
        res.object_states = std::vector<mujoco_msgs::ObjectState>();
    }
    return true;
}

void MjRos::destroy_objects(const std::vector<std::string> object_names)
{
    MjSim::remove_body(object_names);
    object_names_to_destroy.erase(std::remove_if(object_names_to_destroy.begin(), object_names_to_destroy.end(), [object_names](const std::string &object_name)
                                                 { return std::find(object_names.begin(), object_names.end(), object_name) != object_names.end(); }),
                                  object_names_to_destroy.end());
}

void MjRos::spawn_and_destroy_objects()
{
    if (spawn_and_destroy_objects_rate < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(spawn_and_destroy_objects_rate);
    while (ros::ok())
    {
        // Spawn objects
        if (objects_to_spawn.size() > 0)
        {
            if (pub_object_tf_rate > 1E-9)
            {
                std_msgs::Header header;
                header.frame_id = root_frame_id;
                header.stamp = ros::Time::now();

                geometry_msgs::TransformStamped transform;
                transform.header = header;

                // Publish tf of static objects
                std::string object_name;
                for (const mujoco_msgs::ObjectStatus &object : objects_to_spawn)
                {
                    const char *name = object.info.name.c_str();
                    const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, name);
                    if (m->body_mocapid[body_id] == -1)
                    {
                        continue;
                    }
                    set_transform(transform, body_id, name);
                    static_br.sendTransform(transform);
                }
            }

            std::unique_lock<std::mutex> lk(spawn_mtx);

            while (objects_to_spawn.size() > 0)
            {
                if (spawn_object_count_per_cycle == -1)
                {
                    spawn_objects(objects_to_spawn);
                }
                else
                {
                    std::vector<mujoco_msgs::ObjectStatus> objects;
                    size_t i = 0;
                    for (const mujoco_msgs::ObjectStatus &object : objects_to_spawn)
                    {
                        if (i++ > spawn_object_count_per_cycle)
                        {
                            break;
                        }
                        objects.push_back(object);
                    }

                    spawn_objects(objects);
                }
            }

            lk.unlock();
            condition.notify_all();
        }

        // Destroy objects
        if (object_names_to_destroy.size() > 0)
        {
            std::unique_lock<std::mutex> lk(destroy_mtx);

            while (object_names_to_destroy.size() > 0)
            {
                destroy_objects(object_names_to_destroy);
            }

            lk.unlock();
            condition.notify_all();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_tf()
{
    if (pub_object_tf_rate < 1E-9)
    {
        return;
    }

    geometry_msgs::TransformStamped transform;

    ros::Rate loop_rate(pub_object_tf_rate);

    std_msgs::Header header;
    header.frame_id = root_frame_id;
    header.stamp = ros::Time::now();

    transform.header = header;

    // Publish tf of static objects
    std::string object_name;
    for (int body_id = 1; body_id < m->nbody; body_id++)
    {
        if (m->body_mocapid[body_id] == -1)
        {
            continue;
        }
        object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
        set_transform(transform, body_id, object_name);
        static_br.sendTransform(transform);
    }

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        transform.header = header;

        // Publish tf of objects
        std::string object_name;
        for (int body_id = 1; body_id < m->nbody; body_id++)
        {
            if (m->body_mocapid[body_id] != -1)
            {
                continue;
            }
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
            if (std::find(MjSim::link_names.begin(), MjSim::link_names.end(), object_name) == MjSim::link_names.end())
            {
                if (object_name == model_path.stem().string() || (std::find(MjSim::robots.begin(), MjSim::robots.end(), object_name) != MjSim::robots.end()))
                {
                    continue;
                }

                set_transform(transform, body_id, object_name);
                br.sendTransform(transform);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_marker_array()
{
    if (pub_object_marker_array_rate < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_object_marker_array_rate);

    marker.action = visualization_msgs::Marker::MODIFY;

    std_msgs::Header header;
    header.frame_id = root_frame_id;

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        marker.header = header;
        marker_array.markers.clear();

        // Publish marker of objects
        std::string object_name;
        for (int body_id = 1; body_id < m->nbody; body_id++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
            if (std::find(MjSim::link_names.begin(), MjSim::link_names.end(), object_name) == MjSim::link_names.end())
            {
                if (object_name == model_path.stem().string() || (std::find(MjSim::robots.begin(), MjSim::robots.end(), object_name) != MjSim::robots.end()))
                {
                    continue;
                }

                add_marker(body_id);
            }
        }

        // Publish markers
        marker_array_pub.publish(marker_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_object_state_array()
{
    if (pub_object_state_array_rate < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_object_state_array_rate);

    std_msgs::Header header;
    header.frame_id = root_frame_id;

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        object_state_array.header = header;
        object_state_array.object_states.clear();

        // Publish state of objects
        std::string object_name;
        for (int body_id = 1; body_id < m->nbody; body_id++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
            if (std::find(MjSim::link_names.begin(), MjSim::link_names.end(), object_name) == MjSim::link_names.end())
            {
                if (object_name == model_path.stem().string() || (std::find(MjSim::robots.begin(), MjSim::robots.end(), object_name) != MjSim::robots.end()))
                {
                    continue;
                }

                if (m->body_mocapid[body_id] != -1)
                {
                    continue;
                }

                add_object_state(body_id);
            }
        }

        object_states_pub.publish(object_state_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_world_joint_states()
{
    if (pub_world_joint_state_rate < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_world_joint_state_rate);

    std_msgs::Header header;
    header.frame_id = root_frame_id;

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        world_joint_states.header = header;

        world_joint_states.name.clear();
        world_joint_states.position.clear();
        world_joint_states.velocity.clear();
        world_joint_states.effort.clear();

        // Publish tf and marker of objects
        std::string object_name;
        for (int body_id = 1; body_id < m->nbody; body_id++)
        {
            object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
            if (std::find(MjSim::link_names.begin(), MjSim::link_names.end(), object_name) == MjSim::link_names.end())
            {
                if (object_name == model_path.stem().string() || (std::find(MjSim::robots.begin(), MjSim::robots.end(), object_name) != MjSim::robots.end()))
                {
                    continue;
                }

                add_world_joint_states(body_id);
            }
        }

        world_joint_states_pub.publish(world_joint_states);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_base_pose()
{
    if (pub_base_pose_rate < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_base_pose_rate);

    std_msgs::Header header;
    header.frame_id = root_frame_id;

    geometry_msgs::TransformStamped transform;

    for (const std::string &root_name : root_names)
    {
        nav_msgs::Odometry base_pose;
        base_pose.child_frame_id = root_name;
        base_poses.push_back(base_pose);
    }

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        for (nav_msgs::Odometry &base_pose : base_poses)
        {
            base_pose.header = header;
        }

        transform.header = header;

        // Publish tf of root
        for (size_t i = 0; i < root_names.size(); i++)
        {
            const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, MjSim::robots[i].c_str());
            if (body_id != -1)
            {
                if (MjSim::robots.size() > 1)
                {
                    set_transform(transform, body_id, MjSim::robots[i] + "/" + root_names[i]);
                }
                else
                {
                    set_transform(transform, body_id, root_names[i]);
                }

                br.sendTransform(transform);

                if (MjSim::add_odom_joints[MjSim::robots[i]]["lin_odom_x_joint"] ||
                    MjSim::add_odom_joints[MjSim::robots[i]]["lin_odom_y_joint"] ||
                    MjSim::add_odom_joints[MjSim::robots[i]]["lin_odom_z_joint"] ||
                    MjSim::add_odom_joints[MjSim::robots[i]]["ang_odom_x_joint"] ||
                    MjSim::add_odom_joints[MjSim::robots[i]]["ang_odom_y_joint"] ||
                    MjSim::add_odom_joints[MjSim::robots[i]]["ang_odom_z_joint"])
                {
                    set_base_pose(body_id, i);
                    base_pose_pubs[i].publish(base_poses[i]);
                }
            }
            else
            {
                ROS_ERROR("Body id of %s not found", MjSim::robots[i].c_str());
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_sensor_data()
{
    if (pub_sensor_data_rate < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_sensor_data_rate);

    std_msgs::Header header;
    geometry_msgs::Vector3Stamped sensor_data;
    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();

        for (const std::pair<size_t, std::string> &sensor : MjSim::sensors)
        {
            header.seq += 1;
            header.frame_id = sensor.second;

            sensor_data.header = header;
            const int sensor_adr = m->sensor_adr[sensor.first];
            sensor_data.vector.x = d->sensordata[sensor_adr];
            sensor_data.vector.y = d->sensordata[sensor_adr + 1];
            sensor_data.vector.z = d->sensordata[sensor_adr + 2];

            sensors_pub.publish(sensor_data);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::add_marker(const int body_id)
{
    const int geom_id = m->body_geomadr[body_id];
    if (geom_id != -1)
    {
        boost::filesystem::path mesh_path;
        switch (m->geom_type[geom_id])
        {
        case mjtGeom::mjGEOM_BOX:
            marker.type = visualization_msgs::Marker::CUBE;
            marker.mesh_resource = "";
            marker.scale.x = m->geom_size[3 * geom_id] * 2;
            marker.scale.y = m->geom_size[3 * geom_id + 1] * 2;
            marker.scale.z = m->geom_size[3 * geom_id + 2] * 2;
            break;

        case mjtGeom::mjGEOM_SPHERE:
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.mesh_resource = "";
            marker.scale.x = m->geom_size[3 * geom_id] * 2;
            marker.scale.y = m->geom_size[3 * geom_id] * 2;
            marker.scale.z = m->geom_size[3 * geom_id] * 2;
            break;

        case mjtGeom::mjGEOM_CYLINDER:
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = m->geom_size[3 * geom_id] * 2;
            marker.scale.y = m->geom_size[3 * geom_id] * 2;
            marker.scale.z = m->geom_size[3 * geom_id + 1] * 2;
            break;

        case mjtGeom::mjGEOM_MESH:
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            mesh_path = tmp_world_path.parent_path() / (world_path.stem().string() + "/meshes/" + mj_id2name(m, mjtObj::mjOBJ_BODY, body_id) + ".dae");
            if (!boost::filesystem::exists(mesh_path))
            {
                ROS_WARN("Mesh %s not found in %s", mesh_path.filename().c_str(), mesh_path.parent_path().c_str());
                return;
            }
            marker.mesh_resource = "package://mujoco_sim/model/tmp/" + world_path.stem().string() + "/meshes/" + mj_id2name(m, mjtObj::mjOBJ_BODY, body_id) + ".dae";
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            break;

        default:
            break;
        }

        marker.ns = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
        marker.color.a = m->geom_rgba[4 * geom_id + 3];
        marker.color.r = m->geom_rgba[4 * geom_id];
        marker.color.g = m->geom_rgba[4 * geom_id + 1];
        marker.color.b = m->geom_rgba[4 * geom_id + 2];
        marker.pose.position.x = d->xpos[3 * body_id];
        marker.pose.position.y = d->xpos[3 * body_id + 1];
        marker.pose.position.z = d->xpos[3 * body_id + 2];
        marker.pose.orientation.x = d->xquat[4 * body_id + 1];
        marker.pose.orientation.y = d->xquat[4 * body_id + 2];
        marker.pose.orientation.z = d->xquat[4 * body_id + 3];
        marker.pose.orientation.w = d->xquat[4 * body_id];

        marker_array.markers.push_back(marker);
    }
}

void MjRos::add_object_state(const int body_id)
{
    mujoco_msgs::ObjectState object_state;
    object_state.name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
    object_state.pose.position.x = d->xpos[3 * body_id];
    object_state.pose.position.y = d->xpos[3 * body_id + 1];
    object_state.pose.position.z = d->xpos[3 * body_id + 2];
    object_state.pose.orientation.x = d->xquat[4 * body_id + 1];
    object_state.pose.orientation.y = d->xquat[4 * body_id + 2];
    object_state.pose.orientation.z = d->xquat[4 * body_id + 3];
    object_state.pose.orientation.w = d->xquat[4 * body_id];

    if (m->body_dofnum[body_id] == 6)
    {
        const int dof_adr = m->jnt_dofadr[m->body_jntadr[body_id]];
        object_state.velocity.linear.x = d->qvel[dof_adr];
        object_state.velocity.linear.y = d->qvel[dof_adr + 1];
        object_state.velocity.linear.z = d->qvel[dof_adr + 2];
        object_state.velocity.angular.x = d->qvel[dof_adr + 3];
        object_state.velocity.angular.y = d->qvel[dof_adr + 4];
        object_state.velocity.angular.z = d->qvel[dof_adr + 5];
    }

    object_state_array.object_states.push_back(object_state);
}

void MjRos::set_transform(geometry_msgs::TransformStamped &transform, const int body_id, const std::string &object_name)
{
    transform.child_frame_id = object_name;

    transform.transform.translation.x = d->xpos[3 * body_id];
    transform.transform.translation.y = d->xpos[3 * body_id + 1];
    transform.transform.translation.z = d->xpos[3 * body_id + 2];

    const double sqrt_sum_square = mju_sqrt(d->xquat[4 * body_id] * d->xquat[4 * body_id] +
                                            d->xquat[4 * body_id + 1] * d->xquat[4 * body_id + 1] +
                                            d->xquat[4 * body_id + 2] * d->xquat[4 * body_id + 2] +
                                            d->xquat[4 * body_id + 3] * d->xquat[4 * body_id + 3]);

    transform.transform.rotation.x = d->xquat[4 * body_id + 1] / sqrt_sum_square;
    transform.transform.rotation.y = d->xquat[4 * body_id + 2] / sqrt_sum_square;
    transform.transform.rotation.z = d->xquat[4 * body_id + 3] / sqrt_sum_square;
    transform.transform.rotation.w = d->xquat[4 * body_id] / sqrt_sum_square;
}

void MjRos::set_base_pose(const int body_id, const int robot_id)
{
    base_poses[robot_id].pose.pose.position.x = d->xpos[3 * body_id];
    base_poses[robot_id].pose.pose.position.y = d->xpos[3 * body_id + 1];
    base_poses[robot_id].pose.pose.position.z = d->xpos[3 * body_id + 2];
    base_poses[robot_id].pose.pose.orientation.x = d->xquat[4 * body_id + 1];
    base_poses[robot_id].pose.pose.orientation.y = d->xquat[4 * body_id + 2];
    base_poses[robot_id].pose.pose.orientation.z = d->xquat[4 * body_id + 3];
    base_poses[robot_id].pose.pose.orientation.w = d->xquat[4 * body_id];
    base_poses[robot_id].pose.covariance.assign(0.0);
    base_poses[robot_id].twist.covariance.assign(0.0);
}

void MjRos::add_world_joint_states(const int body_id)
{
    if (m->body_jntnum[body_id] == 1 && (m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_HINGE || m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_SLIDE))
    {
        const int joint_id = m->body_jntadr[body_id];
        const char *joint_name = mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id);
        const int qpos_id = m->jnt_qposadr[joint_id];
        const int dof_id = m->jnt_dofadr[joint_id];
        world_joint_states.name.push_back(joint_name);
        world_joint_states.position.push_back(d->qpos[qpos_id]);
        world_joint_states.velocity.push_back(d->qvel[dof_id]);
    }
}