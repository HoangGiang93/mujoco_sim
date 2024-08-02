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

#include "mj_util.h"

#include <condition_variable>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <numeric>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <urdf/model.h>

using namespace std::chrono_literals;

ros::Time MjRos::ros_start;

static std::map<std::string, std::string> root_names;

static std::map<EObjectType, double> pub_marker_array_rate;
static std::map<EObjectType, double> pub_tf_rate;
static std::map<EObjectType, double> pub_object_state_array_rate;
static std::map<EObjectType, double> pub_joint_states_rate;

static double pub_base_pose_rate;
static double pub_sensor_data_rate;
static double spawn_and_destroy_objects_rate;
static int spawn_object_count_per_cycle;

static std::map<EObjectType, visualization_msgs::Marker> marker;
static std::map<EObjectType, visualization_msgs::MarkerArray> marker_array;
static std::map<EObjectType, sensor_msgs::JointState> joint_states;
static std::map<EObjectType, mujoco_msgs::ObjectStateArray> object_state_array;
static std::map<EObjectType, bool> free_bodies_only;

static std::map<std::string, nav_msgs::Odometry> base_poses;

static std::condition_variable condition;

static int spawn_nr = 0;
static std::mutex spawn_mtx;
static std::vector<mujoco_msgs::ObjectStatus> objects_to_spawn;
static std::set<std::string> spawned_object_names;
static bool spawn_success;

static int destroy_nr = 0;
static std::mutex destroy_mtx;
static std::set<std::string> object_names_to_destroy;
static bool destroy_success;

static bool pub_tf_of_free_bodies_only;
static bool pub_object_marker_array_of_free_bodies_only;
static bool pub_object_state_array_of_free_bodies_only;

static std::map<mjtObj, std::map<std::string, std::string>> name_map;

static std::map<mjtObj, int> unique_index = {{mjtObj::mjOBJ_MESH, 0}, {mjtObj::mjOBJ_BODY, 0}, {mjtObj::mjOBJ_JOINT, 0}, {mjtObj::mjOBJ_GEOM, 0}};

static bool init_urdf(urdf::Model &urdf_model, const ros::NodeHandle &n, const char *robot_description = "robot_description")
{
    std::string robot_description_string;
    if (ros::param::get(robot_description, robot_description_string))
    {
        return urdf_model.initParamWithNodeHandle(robot_description, n);
    }
    else
    {
        ROS_WARN("%s not found", robot_description);
        return false;
    }
}

static void set_ros_msg(MjRos &mj_ros, const EObjectType object_type, bool free_body_only, std::function<void(MjRos &, const int, const EObjectType)> function)
{

    for (int body_id = 1; body_id < m->nbody; body_id++)
    {
        const std::string body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
        switch (object_type)
        {
        case EObjectType::Robot:
            if (MjSim::robot_link_names.find(body_name) != MjSim::robot_link_names.end())
            {
                function(mj_ros, body_id, object_type);
            }
            break;

        case EObjectType::World:
            if (MjSim::robot_names.find(body_name) == MjSim::robot_names.end() && MjSim::robot_link_names.find(body_name) == MjSim::robot_link_names.end() && MjSim::spawned_object_body_names.find(body_name) == MjSim::spawned_object_body_names.end())
            {
                if (!free_body_only ||
                    (m->body_jntnum[body_id] == 1 && m->jnt_type[m->body_jntadr[body_id]] == mjJNT_FREE))
                {
                    function(mj_ros, body_id, object_type);
                }
            }
            break;

        case EObjectType::SpawnedObject:
            if (MjSim::spawned_object_body_names.find(body_name) != MjSim::spawned_object_body_names.end())
            {
                if (!free_body_only ||
                    (m->body_jntnum[body_id] == 1 && m->jnt_type[m->body_jntadr[body_id]] == mjJNT_FREE))
                {
                    function(mj_ros, body_id, object_type);
                }
            }
            break;

        default:
            break;
        }
    }
}

static std::function<void(tinyxml2::XMLElement *, const mjtObj)> add_index = [](tinyxml2::XMLElement *element, const mjtObj type)
{
    if (element->Attribute("name") == nullptr)
    {
        return;
    }

    std::string name = element->Attribute("name");
    name += "_" + std::to_string(unique_index[type]);
    name_map[type][element->Attribute("name")] = name;
    element->SetAttribute("name", name.c_str());
};

static std::function<void(tinyxml2::XMLElement *, const mjtObj)> check_index = [](tinyxml2::XMLElement *element, const mjtObj type)
{
    if (element->Attribute("name") == nullptr)
    {
        return;
    }

    std::string name = element->Attribute("name");

    if (type != mjtObj::mjOBJ_MESH)
    {
        name_map[type].erase(name);
    }

    do
    {
        const size_t last_underscore_index = name.find_last_of("_");
        const std::string string_after_underscore = name.substr(last_underscore_index + 1);
        if (string_after_underscore.empty() || string_after_underscore.find_first_not_of("0123456789") != std::string::npos)
        {
            name += "_" + std::to_string(unique_index[type]);
        }
        else
        {
            const size_t last_index = name.find_last_not_of("0123456789");
            name.replace(last_index + 1, unique_index[type] / 10 + 1, std::to_string(unique_index[type]));
        }

        if (mj_name2id(m, type, name.c_str()) != -1)
        {
            unique_index[type] += 1;
        }
    } while (mj_name2id(m, type, name.c_str()) != -1);

    name_map[type][element->Attribute("name")] = name;

    element->SetAttribute("name", name.c_str());
};

CmdVelCallback::CmdVelCallback(const std::string &in_robot) : robot(in_robot)
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
        base_poses[robot].twist.twist = msg;
    }
}

MjRos::~MjRos()
{
}

void MjRos::set_params()
{
    std::string model_path_string;
    if (ros::param::get("~robot", model_path_string))
    {
        boost::filesystem::path model_path_path = model_path_string;
        if (model_path_path.extension() == ".urdf")
        {
            model_path = model_path.parent_path() / (model_path_path.stem().string() + ".xml");
            tmp_model_name = "current_" + std::to_string(ros::Time::now().toBoost().time_of_day().total_microseconds()) + "_" + model_path.filename().string();
        }
        else
        {
            model_path = model_path_string;
            tmp_model_name = "current_" + std::to_string(ros::Time::now().toBoost().time_of_day().total_microseconds()) + "_" + model_path.filename().string();
        }
    }

    if (!ros::param::get("~disable_gravity", MjSim::disable_gravity))
    {
        MjSim::disable_gravity = true;
    }
    ROS_INFO("Set disable_gravity to %s", MjSim::disable_gravity ? "true" : "false");

    if (ros::param::get("~max_time_step", MjSim::max_time_step))
    {
        ROS_INFO("Set max_time_step = %f", MjSim::max_time_step);
    }
    else
    {
        MjSim::max_time_step = 0.005;
    }

    std::string world_path_string;
    if (ros::param::get("~world", world_path_string))
    {
        ROS_INFO("Set world from %s", world_path_string.c_str());
        world_path = world_path_string;
    }

    std::vector<std::string> robots;
    if (!ros::param::get("~robots", robots))
    {
        ROS_WARN("Robot names not found in parameter server, searching for robot names from urdf...");
        urdf::Model urdf_model;
        if (init_urdf(urdf_model, ros::NodeHandle()))
        {
            robots.push_back(urdf_model.getName());
        }
        else
        {
            ROS_WARN("Robot names not found in urdf, searching for robot names from mjcf...");
            tinyxml2::XMLDocument cache_model_xml_doc;
            if (!load_XML(cache_model_xml_doc, model_path.c_str()))
            {
                ROS_WARN("Failed to load file \"%s\"\n", model_path.c_str());
            }
            for (tinyxml2::XMLElement *worldbody_element = cache_model_xml_doc.FirstChildElement()->FirstChildElement("worldbody");
                 worldbody_element != nullptr;
                 worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
            {
                for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement();
                     body_element != nullptr;
                     body_element = body_element->NextSiblingElement())
                {
                    if (body_element->Attribute("name") != nullptr)
                    {
                        robots.push_back(body_element->Attribute("name"));
                    }
                }
            }
        }
    }
    MjSim::robot_names = std::set<std::string>(robots.begin(), robots.end());
    if (MjSim::robot_names.size() == 0)
    {
        ROS_WARN("No robot found in [%s]", model_path.c_str());
        return;
    }
    std::string log = "Found " + std::to_string(MjSim::robot_names.size()) + " robots:";
    for (const std::string &robot : MjSim::robot_names)
    {
        log += " [" + robot + "]";
    }
    ROS_INFO("%s", log.c_str());

    std::vector<float> pose_init;
    if (ros::param::get("~pose_init", pose_init) && pose_init.size() == 6)
    {
        for (const std::string &robot : MjSim::robot_names)
        {
            MjSim::pose_inits[robot] = pose_init;
        }
    }
    else
    {
        for (const std::string &robot : MjSim::robot_names)
        {
            if (ros::param::get("~pose_init/" + robot, pose_init) && pose_init.size() == 6)
            {
                MjSim::pose_inits[robot] = pose_init;
            }
        }
    }

    bool add_odom_joints_bool;
    if (ros::param::get("~add_odom_joints", add_odom_joints_bool))
    {
        for (const std::string &robot : MjSim::robot_names)
        {
            MjSim::add_odom_joints[robot]["lin_odom_x_joint"] = add_odom_joints_bool;
            MjSim::add_odom_joints[robot]["lin_odom_y_joint"] = add_odom_joints_bool;
            MjSim::add_odom_joints[robot]["lin_odom_z_joint"] = false;
            MjSim::add_odom_joints[robot]["ang_odom_x_joint"] = false;
            MjSim::add_odom_joints[robot]["ang_odom_y_joint"] = false;
            MjSim::add_odom_joints[robot]["ang_odom_z_joint"] = add_odom_joints_bool;
        }
    }
    else
    {
        bool odom_joint_bool;
        for (const std::string &odom_joint_name : {"lin_odom_x_joint", "lin_odom_y_joint", "lin_odom_z_joint", "ang_odom_x_joint", "ang_odom_y_joint", "ang_odom_z_joint"})
        {
            if (ros::param::get("~add_odom_joints/" + odom_joint_name, odom_joint_bool))
            {
                for (const std::string &robot : MjSim::robot_names)
                {
                    MjSim::add_odom_joints[robot][odom_joint_name] = odom_joint_bool;
                }
            }
            else
            {
                for (const std::string &robot : MjSim::robot_names)
                {
                    MjSim::add_odom_joints[robot][odom_joint_name] = false;
                }
            }
        }

        for (const std::string &robot : MjSim::robot_names)
        {
            if (ros::param::get("~add_odom_joints/" + robot, odom_joint_bool))
            {
                MjSim::add_odom_joints[robot]["lin_odom_x_joint"] = odom_joint_bool;
                MjSim::add_odom_joints[robot]["lin_odom_y_joint"] = odom_joint_bool;
                MjSim::add_odom_joints[robot]["lin_odom_z_joint"] = false;
                MjSim::add_odom_joints[robot]["ang_odom_x_joint"] = false;
                MjSim::add_odom_joints[robot]["ang_odom_y_joint"] = false;
                MjSim::add_odom_joints[robot]["ang_odom_z_joint"] = odom_joint_bool;
            }
            else
            {
                for (const std::string &odom_joint_name : {"lin_odom_x_joint", "lin_odom_y_joint", "lin_odom_z_joint", "ang_odom_x_joint", "ang_odom_y_joint", "ang_odom_z_joint"})
                {
                    if (ros::param::get("~add_odom_joints/" + robot + "/" + odom_joint_name, odom_joint_bool))
                    {
                        MjSim::add_odom_joints[robot][odom_joint_name] = odom_joint_bool;
                    }
                }
            }
        }
    }
}

void MjRos::init()
{
    n = ros::NodeHandle();

    if (ros::param::has("~pub_object_marker_array"))
    {
        if (!ros::param::get("~pub_object_marker_array/free_bodies_only", pub_object_marker_array_of_free_bodies_only))
        {
            pub_object_marker_array_of_free_bodies_only = true;
        }
        if (!ros::param::get("~pub_object_marker_array/robot_bodies_rate", pub_marker_array_rate[EObjectType::Robot]))
        {
            pub_marker_array_rate[EObjectType::Robot] = 0.0;
        }
        if (!ros::param::get("~pub_object_marker_array/world_bodies_rate", pub_marker_array_rate[EObjectType::World]))
        {
            pub_marker_array_rate[EObjectType::World] = 0.0;
        }
        if (!ros::param::get("~pub_object_marker_array/spawned_object_bodies_rate", pub_marker_array_rate[EObjectType::SpawnedObject]))
        {
            pub_marker_array_rate[EObjectType::SpawnedObject] = 0.0;
        }
    }

    if (ros::param::has("~pub_tf"))
    {
        if (!ros::param::get("~pub_tf/free_bodies_only", pub_tf_of_free_bodies_only))
        {
            pub_tf_of_free_bodies_only = true;
        }
        if (!ros::param::get("~pub_tf/robot_bodies_rate", pub_tf_rate[EObjectType::Robot]))
        {
            pub_tf_rate[EObjectType::Robot] = 0.0;
        }
        if (!ros::param::get("~pub_tf/world_bodies_rate", pub_tf_rate[EObjectType::World]))
        {
            pub_tf_rate[EObjectType::World] = 0.0;
        }
        if (!ros::param::get("~pub_tf/spawned_object_bodies_rate", pub_tf_rate[EObjectType::SpawnedObject]))
        {
            pub_tf_rate[EObjectType::SpawnedObject] = 60.0;
        }
    }

    if (ros::param::has("~pub_object_state_array"))
    {
        if (!ros::param::get("~pub_object_state_array/free_bodies_only", pub_object_state_array_of_free_bodies_only))
        {
            pub_object_state_array_of_free_bodies_only = true;
        }
        if (!ros::param::get("~pub_object_state_array/robot_bodies_rate", pub_object_state_array_rate[EObjectType::Robot]))
        {
            pub_object_state_array_rate[EObjectType::Robot] = 0.0;
        }
        if (!ros::param::get("~pub_object_state_array/world_bodies_rate", pub_object_state_array_rate[EObjectType::World]))
        {
            pub_object_state_array_rate[EObjectType::World] = 0.0;
        }
        if (!ros::param::get("~pub_object_state_array/spawned_object_bodies_rate", pub_object_state_array_rate[EObjectType::SpawnedObject]))
        {
            pub_object_state_array_rate[EObjectType::SpawnedObject] = 60.0;
        }
    }

    if (ros::param::has("~pub_joint_states"))
    {
        if (!ros::param::get("~pub_joint_states/robot_bodies_rate", pub_joint_states_rate[EObjectType::Robot]))
        {
            pub_joint_states_rate[EObjectType::Robot] = 0.0;
        }
        if (!ros::param::get("~pub_joint_states/world_bodies_rate", pub_joint_states_rate[EObjectType::World]))
        {
            pub_joint_states_rate[EObjectType::World] = 60.0;
        }
        if (!ros::param::get("~pub_joint_states/spawned_object_bodies_rate", pub_joint_states_rate[EObjectType::SpawnedObject]))
        {
            pub_joint_states_rate[EObjectType::SpawnedObject] = 60.0;
        }
    }

    if (!ros::param::get("~custom_controller_type", custom_controller_type)) {
        custom_controller_type = "";
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
    for (const std::string &robot : MjSim::robot_names)
    {
        for (const std::string joint_name : MjSim::joint_names[robot])
        {
            joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
            link_name = mj_id2name(m, mjtObj::mjOBJ_BODY, m->jnt_bodyid[joint_id]);
            MjSim::robot_link_names.insert(link_name);
        }
    }

    if (MjSim::robot_names.size() < 2)
    {
        urdf::Model urdf_model;
        if (init_urdf(urdf_model, n))
        {
            root_names[*MjSim::robot_names.begin()] = urdf_model.getRoot()->name;
        }
        else
        {
            root_names[*MjSim::robot_names.begin()] = "world";
        }
    }
    else
    {
        for (const std::string &robot : MjSim::robot_names)
        {
            urdf::Model urdf_model;
            if (init_urdf(urdf_model, n, (robot + "/robot_description").c_str()))
            {
                root_names[robot] = urdf_model.getRoot()->name;
            }
            else
            {
                root_names[robot] = "world";
            }
        }
    }

    for (const std::string &robot : MjSim::robot_names)
    {
        if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] ||
            MjSim::add_odom_joints[robot]["lin_odom_y_joint"] ||
            MjSim::add_odom_joints[robot]["lin_odom_z_joint"] ||
            MjSim::add_odom_joints[robot]["ang_odom_x_joint"] ||
            MjSim::add_odom_joints[robot]["ang_odom_y_joint"] ||
            MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
        {
            cmd_vel_callbacks[robot] = new CmdVelCallback(robot);

            cmd_vel_subs[robot] = n.subscribe("/" + robot + "/cmd_vel", 1, &CmdVelCallback::callback, cmd_vel_callbacks[robot]);
        }
    }

    screenshot_server = n.advertiseService("/mujoco/screenshot", &MjRos::screenshot_service, this);
    ROS_INFO("Started [%s] service.", screenshot_server.getService().c_str());

    reset_robot_server = n.advertiseService("/mujoco/reset", &MjRos::reset_robot_service, this);
    ROS_INFO("Started [%s] service.", reset_robot_server.getService().c_str());

    spawn_objects_server = n.advertiseService("/mujoco/spawn_objects", &MjRos::spawn_objects_service, this);
    ROS_INFO("Started [%s] service.", spawn_objects_server.getService().c_str());

    destroy_objects_server = n.advertiseService("/mujoco/destroy_objects", &MjRos::destroy_objects_service, this);
    ROS_INFO("Started [%s] service.", destroy_objects_server.getService().c_str());

    if (!ros::param::get("~joint_inits", joint_inits))
    {
        ROS_WARN("joint_inits not found, will set to default value (0)");
    }

    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("/mujoco/visualization_marker_array", 0);
    for (const std::string &robot : MjSim::robot_names)
    {
        base_pose_pubs[robot] = n.advertise<nav_msgs::Odometry>("/" + robot + "/" + root_names[robot], 0);
    }

    object_state_array_pub = n.advertise<mujoco_msgs::ObjectStateArray>("/mujoco/object_states", 0);
    joint_states_pub[EObjectType::Robot] = n.advertise<sensor_msgs::JointState>("/mujoco/robot_joint_states", 0);
    joint_states_pub[EObjectType::World] = n.advertise<sensor_msgs::JointState>("/mujoco/world_joint_states", 0);
    joint_states_pub[EObjectType::SpawnedObject] = n.advertise<sensor_msgs::JointState>("/mujoco/object_joint_states", 0);
    sensors_pub = n.advertise<geometry_msgs::Vector3Stamped>("/mujoco/sensors_3D", 0);

    reset_robot();
}

void MjRos::reset_robot()
{
    d = mj_makeData(m);
    
    for (const std::string &robot : MjSim::robot_names)
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

void MjRos::setup_publishers()
{
    std::thread ros_thread1(&MjRos::publish_tf, this, EObjectType::None);
    std::thread ros_thread2(&MjRos::publish_marker_array, this, EObjectType::None);
    std::thread ros_thread3(&MjRos::publish_object_state_array, this, EObjectType::None);
    std::thread ros_thread4(&MjRos::publish_joint_states, this, EObjectType::None);
    std::thread ros_thread5(&MjRos::publish_base_pose, this);
    std::thread ros_thread6(&MjRos::publish_sensor_data, this);

    ros_thread1.join();
    ros_thread2.join();
    ros_thread3.join();
    ros_thread4.join();
    ros_thread5.join();
    ros_thread6.join();
}

void MjRos::setup_service_servers()
{
    std::thread ros_thread(&MjRos::spawn_and_destroy_objects, this);
    ros_thread.join();
}

void MjRos::get_controlled_joints()
{
    ros::Rate loop_rate(1);
    int i = 0;
    while (ros::ok())
    {
        if (i++ == 10)
        {
            break;
        }
        ros::ServiceClient list_controllers_client = n.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");
        controller_manager_msgs::ListControllers list_controllers_srv;
        if (list_controllers_client.call(list_controllers_srv))
        {
            for (const controller_manager_msgs::ControllerState &controller_state : list_controllers_srv.response.controller)
            {
                if (controller_state.state.compare("running") == 0 && 
                    (controller_state.type.find("position_controllers") != std::string::npos ||
                    controller_state.type.find("velocity_controllers") != std::string::npos ||
                    controller_state.type.find("effort_controllers") != std::string::npos ||
                    (custom_controller_type != "" && controller_state.type.find(custom_controller_type) != std::string::npos)))
                {
                    for (const controller_manager_msgs::HardwareInterfaceResources &claimed_resources : controller_state.claimed_resources)
                    {
                        for (const std::string &joint_name : claimed_resources.resources)
                        {
                            MjSim::controlled_joints.insert(joint_name);
                        }
                    }
                }
            }
        }
        loop_rate.sleep();
    }
}

bool MjRos::screenshot_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    std::string save_path_string;
    if (!ros::param::get("~save_path", save_path_string))
    {
        if (strcmp(save_path.extension().c_str(), ".xml") == 0)
        {
            save_path = save_path.parent_path();
        }
        ROS_WARN("Parameter save_path not found or invalid, set to [%s]", save_path.c_str());
        save_path_string = save_path.string();
    }
    if (boost::filesystem::path(save_path_string).is_relative())
    {
        save_path_string = ros::package::getPath("mujoco_sim") + "/" + save_path_string;
    }

    if (boost::filesystem::exists(save_path_string) && strcmp(save_path.extension().c_str(), ".xml") != 0)
    {
        save_path = save_path_string;
        save_path /= model_path.stem().string() + ".xml";
    }
    else
    {
        if (!boost::filesystem::exists(save_path_string))
        {
            if (!boost::filesystem::create_directory(save_path_string))
            {
                ROS_WARN("save_path [%s] is invalid, set to [%s]", save_path_string.c_str(), save_path.c_str());
            }
        }
        save_path = save_path_string;
    }

    // if (boost::filesystem::exists(save_path))
    // {

    //     boost::filesystem::remove_all(save_path.parent_path());
    //     const boost::posix_time::ptime time_now = ros::Time::now().toBoost();
    //     const unsigned short year = time_now.date().year();
    //     const unsigned short month = time_now.date().month();
    //     const unsigned short day = time_now.date().day();
    //     const unsigned short hour = time_now.time_of_day().hours();
    //     const unsigned short minute = time_now.time_of_day().minutes();
    //     const unsigned short sec = time_now.time_of_day().seconds();
    //     const unsigned short msec = time_now.time_of_day().total_milliseconds() % 1000;
    //     const std::string time_stamp = std::to_string(year) + "_" + std::to_string(month) + "_" + std::to_string(day) + "_" + std::to_string(hour) + "_" + std::to_string(minute) + "_" + std::to_string(sec) + "_" + std::to_string(msec);
    //     const std::string filename = model_path.stem().string() + "_" + time_stamp + ".xml";
    //     save_path = save_path.parent_path() / filename;
    // }

    boost::filesystem::path save_model_path = save_path.parent_path() / (save_path.stem().string() + ".txt");
    boost::filesystem::path save_data_path = save_path.parent_path() / (save_path.stem().string() + "_data.txt");

    ROS_INFO("Trying to save screenshot to [%s]", save_path.c_str());

    if (save_XML(m, save_path.c_str()))
    {
        std::function<void(tinyxml2::XMLElement *)> copy_meshes_cb = [&](tinyxml2::XMLElement *mesh_element)
        {
            if (mesh_element->Attribute("name") != nullptr && mesh_element->Attribute("file") != nullptr && boost::filesystem::exists(mesh_element->Attribute("file")))
            {
                boost::filesystem::path file = mesh_element->Attribute("file");
                boost::filesystem::path new_path = save_path.parent_path() / file.parent_path().parent_path().filename() / file.parent_path().filename();
                if (!boost::filesystem::exists(new_path))
                {
                    boost::filesystem::create_directories(new_path);
                }
                new_path /= file.filename();
                if (!boost::filesystem::exists(new_path))
                {
                    boost::filesystem::copy_file(file, new_path);
                }
                mesh_element->SetAttribute("file", boost::filesystem::relative(new_path, save_path.parent_path()).c_str());
            }
        };

        std::function<void(tinyxml2::XMLElement *)> change_meshdir_cb = [&](tinyxml2::XMLElement *compiler_element)
        {
            compiler_element->DeleteAttribute("meshdir");
            compiler_element->SetAttribute("boundmass", "0.000001");
            compiler_element->SetAttribute("boundinertia", "0.000001");
        };

        tinyxml2::XMLDocument doc;
        load_XML(doc, save_path.c_str());
        do_each_child_element(doc.FirstChildElement()->FirstChildElement("asset"), "mesh", copy_meshes_cb);
        do_each_child_element(doc.FirstChildElement(), "compiler", change_meshdir_cb);

        save_XML(doc, save_path.c_str());

        mj_printModel(m, save_model_path.c_str());
        mtx.lock();
        mj_printData(m, d, save_data_path.c_str());
        mtx.unlock();
        res.success = true;
        res.message = save_path.string();
        ROS_INFO("Saved screenshot to [%s] successfully", save_path.c_str());
    }
    else
    {
        res.success = false;
        res.message = save_path.parent_path().string() + " not found";
        ROS_INFO("Failed to save screenshot to [%s]", save_path.c_str());
    }

    return true;
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
    for (const std::string &robot : MjSim::robot_names)
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
    int i = 0;
    for (mujoco_msgs::ObjectStatus &object : req.objects)
    {
        if (object.info.name.empty())
        {
            object.info.name = "Object_" + std::to_string(spawn_nr);
            if (i++ > 0)
            {
                object.info.name += "_" + std::to_string(i);
            }

            ROS_WARN("[Spawn #%d] Empty name found, replace to %s", spawn_nr, object.info.name.c_str());
        }
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
        ROS_WARN("[Spawn #%d] Can't find any spawnable object, either the object exists already or there is no object to spawn", spawn_nr);
        return true;
    }

    std::unique_lock<std::mutex> lk(spawn_mtx);
    spawn_success = false;
    if (condition.wait_until(lk, std::chrono::system_clock::now() + 1000ms, [&]
                             { return spawn_success; }))
    {
        MjSim::reload_mesh = true;
        res.names = names;
        ROS_INFO("[Spawn #%d] Spawned successfully", spawn_nr++);
    }
    else
    {
        res.names = std::vector<std::string>();
        ROS_WARN("[Spawn #%d] Spawned unsuccessfully", spawn_nr++);
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
        body_element->SetAttribute("name", object.info.name.c_str());
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
            if (object_mesh_path.extension().compare(".xml") == 0)
            {
                if (object_mesh_path.is_relative())
                {
                    object_mesh_path = world_path.parent_path() / object_mesh_path;
                }
                else if (!object_mesh_path.is_absolute())
                {
                    ROS_WARN("Mesh path [%s] is not valid", object_mesh_path.c_str());
                }

                tinyxml2::XMLDocument mesh_xml_doc;
                if (!load_XML(mesh_xml_doc, object_mesh_path.c_str()))
                {
                    ROS_WARN("Failed to load file \"%s\"\n", object_mesh_path.c_str());
                    continue;
                }

                boost::filesystem::path mesh_dir = object_mesh_path.parent_path();
                for (tinyxml2::XMLNode *node = mesh_xml_doc.FirstChild()->FirstChild();
                     node != nullptr;
                     node = node->NextSibling())
                {
                    tinyxml2::XMLNode *copy = node->DeepClone(&object_xml_doc);

                    // Save path of asset
                    if (strcmp(copy->Value(), "compiler") == 0 && copy->ToElement()->Attribute("meshdir") != nullptr)
                    {
                        mesh_dir = mesh_dir / copy->ToElement()->Attribute("meshdir");
                        continue;
                    }

                    // Change path of asset
                    else if (strcmp(copy->Value(), "asset") == 0)
                    {
                        do_each_child_element(copy->ToElement(), "mesh", mjtObj::mjOBJ_MESH, add_index);

                        std::vector<tinyxml2::XMLElement *> elements_to_remove;
                        do_each_child_element(copy->ToElement(), "mesh", [&](tinyxml2::XMLElement *mesh_element)
                                              {
                                                if (mesh_element->Attribute("file") != nullptr && mesh_element->Attribute("name") != nullptr)
                                                {
                                                    if (mesh_element->Attribute("file")[0] != '/')
                                                    {
                                                        mesh_element->SetAttribute("file", (mesh_dir / mesh_element->Attribute("file")).c_str());
                                                    }
                                                    std::string mesh_name = mesh_element->Attribute("name");
                                                    if (mesh_paths.find(mesh_name) != mesh_paths.end() || mj_name2id(m, mjtObj::mjOBJ_MESH, mesh_name.c_str()) != -1)
                                                    {
                                                        if (mesh_element->Attribute("file", mesh_paths[mesh_name].first.c_str()))
                                                        {
                                                            remove:
                                                            elements_to_remove.push_back(mesh_element);
                                                        }
                                                        else
                                                        {
                                                            int i = 0;
                                                            do
                                                            {
                                                                const size_t last_underscore_index = mesh_name.find_last_of("_");
                                                                const std::string string_after_underscore = mesh_name.substr(last_underscore_index + 1);
                                                                if (string_after_underscore.empty() || string_after_underscore.find_first_not_of("0123456789") != std::string::npos)
                                                                {
                                                                    mesh_name += "_" + std::to_string(i);
                                                                }
                                                                else
                                                                {
                                                                    size_t last_index = mesh_name.find_last_not_of("0123456789");
                                                                    mesh_name.replace(last_index + 1, i / 10 + 1, std::to_string(i));
                                                                }
                                                                if (mj_name2id(m, mjtObj::mjOBJ_MESH, mesh_name.c_str()) != -1 
                                                                    && mesh_paths.find(mesh_name) != mesh_paths.end() 
                                                                    && mesh_element->Attribute("file", mesh_paths[mesh_name].first.c_str()))
                                                                {
                                                                    goto remove;
                                                                }
                                                                
                                                                i += 1;
                                                            } while (mj_name2id(m, mjtObj::mjOBJ_MESH, mesh_name.c_str()) != -1 || mesh_paths.find(mesh_name) != mesh_paths.end());

                                                            name_map[mjtObj::mjOBJ_MESH][mesh_element->Attribute("name")] = mesh_name;
                                                            mesh_element->SetAttribute("name", mesh_name.c_str());

                                                            goto next;
                                                        }
                                                    }                                                    
                                                    else
                                                    {
                                                        next:
                                                        std::string scale_str = "1 1 1";
                                                        if (mesh_element->Attribute("scale") != nullptr)
                                                        {
                                                            scale_str = mesh_element->Attribute("scale");
                                                        }
                                                        std::istringstream iss(scale_str);
                                                        std::vector<mjtNum> scale = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};

                                                        if (mju_abs(object.info.size.x) > mjMINVAL || mju_abs(object.info.size.y) > mjMINVAL || mju_abs(object.info.size.z) > mjMINVAL)
                                                        {
                                                            scale[0] *= object.info.size.x;
                                                            scale[1] *= object.info.size.y;
                                                            scale[2] *= object.info.size.z;
                                                            mesh_element->SetAttribute("scale",
                                                                                    (std::to_string(scale[0]) + " " +
                                                                                        std::to_string(scale[1]) + " " +
                                                                                        std::to_string(scale[2]))
                                                                                        .c_str());
                                                        }

                                                        mesh_paths[mesh_element->Attribute("name")] = {mesh_element->Attribute("file"), scale};
                                                    }
                                                } });

                        for (tinyxml2::XMLElement *mesh_element : elements_to_remove)
                        {
                            copy->DeleteChild(mesh_element);
                        }

                        root->InsertEndChild(copy);
                    }
                }

                mesh_dir = object_mesh_path.parent_path();
                for (tinyxml2::XMLNode *node = mesh_xml_doc.FirstChild()->FirstChild();
                     node != nullptr;
                     node = node->NextSibling())
                {
                    tinyxml2::XMLNode *copy = node->DeepClone(&object_xml_doc);

                    if (strcmp(copy->Value(), "asset") == 0)
                    {
                        continue;
                    }
                    else if (strcmp(copy->Value(), "worldbody") == 0)
                    {
                        tinyxml2::XMLElement *copy_body_element = copy->FirstChildElement("body");

                        if (copy_body_element != nullptr && copy_body_element->Attribute("name") != nullptr)
                        {
                            name_map[mjtObj::mjOBJ_BODY][copy_body_element->Attribute("name")] = object.info.name;
                        }
                        else
                        {
                            ROS_WARN("Body name not found");
                        }

                        do_each_child_element(copy_body_element, mjtObj::mjOBJ_BODY, add_index);

                        do_each_child_element(copy_body_element, "joint", mjtObj::mjOBJ_JOINT, add_index);

                        do_each_child_element(copy_body_element, "geom", mjtObj::mjOBJ_GEOM, add_index);

                        int i;
                        do
                        {
                            i = unique_index[mjtObj::mjOBJ_BODY];
                            do_each_child_element(copy_body_element, mjtObj::mjOBJ_BODY, check_index);
                        } while (i != unique_index[mjtObj::mjOBJ_BODY]);

                        do
                        {
                            i = unique_index[mjtObj::mjOBJ_JOINT];
                            do_each_child_element(copy_body_element, "joint", mjtObj::mjOBJ_JOINT, check_index);
                        } while (i != unique_index[mjtObj::mjOBJ_JOINT]);

                        do
                        {
                            i = unique_index[mjtObj::mjOBJ_GEOM];
                            do_each_child_element(copy_body_element, "geom", mjtObj::mjOBJ_GEOM, check_index);
                        } while (i != unique_index[mjtObj::mjOBJ_GEOM]);

                        std::function<void(tinyxml2::XMLElement *, const mujoco_msgs::ObjectInfo &)> adjust_body = [](tinyxml2::XMLElement *body_element, const mujoco_msgs::ObjectInfo &info)
                        {
                            if (mju_abs(info.rgba.r) > mjMINVAL || mju_abs(info.rgba.g) > mjMINVAL || mju_abs(info.rgba.b) > mjMINVAL || mju_abs(info.rgba.a) > mjMINVAL)
                            {
                                for (tinyxml2::XMLElement *geom_element = body_element->FirstChildElement("geom");
                                     geom_element != nullptr;
                                     geom_element = geom_element->NextSiblingElement("geom"))
                                {
                                    geom_element->SetAttribute("rgba", (std::to_string(info.rgba.r) + " " +
                                                                        std::to_string(info.rgba.g) + " " +
                                                                        std::to_string(info.rgba.b) + " " +
                                                                        std::to_string(info.rgba.a))
                                                                           .c_str());
                                }
                            }

                            if (mju_abs(info.size.x) > mjMINVAL || mju_abs(info.size.y) > mjMINVAL || mju_abs(info.size.z) > mjMINVAL)
                            {
                                for (tinyxml2::XMLElement *joint_element = body_element->FirstChildElement("joint");
                                     joint_element != nullptr;
                                     joint_element = joint_element->NextSiblingElement("joint"))
                                {
                                    std::vector<mjtNum> size = {info.size.x, info.size.y, info.size.z};

                                    std::vector<mjtNum> joint_pos = {0, 0, 0};
                                    if (joint_element->Attribute("pos") != nullptr)
                                    {
                                        const std::string pos_str = joint_element->Attribute("pos");
                                        std::istringstream iss(pos_str);
                                        joint_pos = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};
                                    }
                                    for (int i = 0; i < joint_pos.size(); i++)
                                    {
                                        joint_pos[i] *= size[i];
                                    }
                                    joint_element->SetAttribute("pos", (std::to_string(joint_pos[0]) + " " +
                                                                        std::to_string(joint_pos[1]) + " " +
                                                                        std::to_string(joint_pos[2]))
                                                                           .c_str());
                                }

                                for (tinyxml2::XMLElement *geom_element = body_element->FirstChildElement("geom");
                                     geom_element != nullptr;
                                     geom_element = geom_element->NextSiblingElement("geom"))
                                {
                                    std::vector<mjtNum> size = {info.size.x, info.size.y, info.size.z};

                                    std::vector<mjtNum> geom_pos = {0, 0, 0};
                                    std::vector<mjtNum> geom_size = {1, 1, 1};

                                    if (geom_element->Attribute("type") == nullptr)
                                    {
                                        geom_element->SetAttribute("type", "sphere");
                                    }

                                    if (geom_element->Attribute("type", "sphere"))
                                    {
                                        geom_size = {1};
                                    }
                                    else if (geom_element->Attribute("type", "cylinder"))
                                    {
                                        geom_size = {1, 1};
                                    }

                                    if (geom_element->Attribute("size") != nullptr)
                                    {
                                        std::string geom_size_str = geom_element->Attribute("size");
                                        std::istringstream iss(geom_size_str);
                                        geom_size = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};
                                    }

                                    std::string size_str = std::to_string(size[0] * geom_size[0]);

                                    for (int i = 1; i < geom_size.size(); i++)
                                    {
                                        size_str += " " + std::to_string(size[i] * geom_size[i]);
                                    }

                                    if (geom_element->Attribute("pos") != nullptr)
                                    {
                                        const std::string pos_str = geom_element->Attribute("pos");
                                        std::istringstream iss(pos_str);
                                        geom_pos = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};
                                    }
                                    if (geom_element->Attribute("type", "box") || geom_element->Attribute("type", "mesh"))
                                    {
                                        for (int i = 0; i < geom_pos.size(); i++)
                                        {
                                            geom_pos[i] *= size[i];
                                        }
                                    }
                                    else if (geom_element->Attribute("type", "sphere"))
                                    {
                                        geom_pos[0] *= size[0];
                                        geom_pos[1] *= size[0];
                                        geom_pos[2] *= size[0];
                                    }
                                    else if (geom_element->Attribute("type", "cylinder"))
                                    {
                                        geom_pos[0] *= size[0];
                                        geom_pos[1] *= size[0];
                                        geom_pos[2] *= size[1];
                                    }
                                    geom_element->SetAttribute("pos", (std::to_string(geom_pos[0]) + " " +
                                                                       std::to_string(geom_pos[1]) + " " +
                                                                       std::to_string(geom_pos[2]))
                                                                          .c_str());
                                    geom_element->SetAttribute("size", size_str.c_str());
                                }
                            }
                        };

                        do_each_child_element(copy->ToElement(), object.info, adjust_body);

                        std::function<void(tinyxml2::XMLElement *)> rename_mesh = [](tinyxml2::XMLElement *geom_element)
                        {
                            if (geom_element->Attribute("type", "mesh"))
                            {
                                geom_element->SetAttribute("mesh", name_map[mjtObj::mjOBJ_MESH][geom_element->Attribute("mesh")].c_str());
                            }
                        };

                        do_each_child_element(copy_body_element, "geom", rename_mesh);

                        copy_body_element->SetAttribute("name", object.info.name.c_str());

                        copy_body_element->SetAttribute("pos", (std::to_string(object.pose.position.x) + " " +
                                                                std::to_string(object.pose.position.y) + " " +
                                                                std::to_string(object.pose.position.z))
                                                                   .c_str());
                        copy_body_element->SetAttribute("quat",
                                                        (std::to_string(object.pose.orientation.w) + " " +
                                                         std::to_string(object.pose.orientation.x) + " " +
                                                         std::to_string(object.pose.orientation.y) + " " +
                                                         std::to_string(object.pose.orientation.z))
                                                            .c_str());

                        unique_index[mjtObj::mjOBJ_BODY] += 1;
                        unique_index[mjtObj::mjOBJ_JOINT] += 1;
                        unique_index[mjtObj::mjOBJ_GEOM] += 1;
                    }
                    else if (strcmp(copy->Value(), "contact") == 0)
                    {
                        for (tinyxml2::XMLElement *exclude_element = copy->FirstChildElement("exclude");
                             exclude_element != nullptr;
                             exclude_element = exclude_element->NextSiblingElement("exclude"))
                        {
                            const std::string body1 = exclude_element->Attribute("body1");
                            const std::string body2 = exclude_element->Attribute("body2");
                            exclude_element->SetAttribute("body1", name_map[mjtObj::mjOBJ_BODY][body1].c_str());
                            exclude_element->SetAttribute("body2", name_map[mjtObj::mjOBJ_BODY][body2].c_str());
                        }
                    }
                    else if (strcmp(copy->Value(), "equality") == 0)
                    {
                        for (tinyxml2::XMLElement *joint_element = copy->FirstChildElement("joint");
                             joint_element != nullptr;
                             joint_element = joint_element->NextSiblingElement("joint"))
                        {
                            const std::string joint1 = joint_element->Attribute("joint1");
                            const std::string joint2 = joint_element->Attribute("joint2");
                            joint_element->SetAttribute("joint1", name_map[mjtObj::mjOBJ_JOINT][joint1].c_str());
                            joint_element->SetAttribute("joint2", name_map[mjtObj::mjOBJ_JOINT][joint2].c_str());
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

    if (!save_XML(object_xml_doc, (tmp_model_path.parent_path() / "add.xml").c_str()))
    {
        ROS_WARN("Failed to save file \"%s\"\n", (tmp_model_path.parent_path() / "add.xml").c_str());
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
                MjSim::spawned_object_body_names.insert(name);
                spawned_object_names.insert(name);
                do_each_child_body_id(m, body_id, [&](int child_body_id)
                                      { MjSim::spawned_object_body_names.insert(mj_id2name(m, mjtObj::mjOBJ_BODY, child_body_id)); });

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
        if (mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str()) != -1)
        {
            object_names_to_destroy.insert(object_name);
        }
    }

    std::vector<mujoco_msgs::ObjectState> object_states;
    if (object_names_to_destroy.empty())
    {
        res.object_states = object_states;
        ROS_WARN("[Destroy #%d] Can't find any destroyable object, either the object doesn't exist or there is no object to destroy", destroy_nr);
        return true;
    }
    else
    {
        const std::size_t objects_num = object_names_to_destroy.size();
        object_states.reserve(objects_num);

        for (const std::string &object_name_to_destroy : object_names_to_destroy)
        {
            const char *name = object_name_to_destroy.c_str();
            ROS_INFO("[Destroy #%d] Try to detroy body %s", destroy_nr, name);
            int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, name);
            if (body_id != -1)
            {
                mujoco_msgs::ObjectState object_state;
                object_state.name = name;
                if (m->body_dofnum[body_id] != 6)
                {
                    continue;
                }

                object_state.pose.position.x = d->xpos[3 * body_id];
                object_state.pose.position.y = d->xpos[3 * body_id + 1];
                object_state.pose.position.z = d->xpos[3 * body_id + 2];

                object_state.pose.orientation.x = d->xquat[4 * body_id + 1];
                object_state.pose.orientation.x = d->xquat[4 * body_id + 2];
                object_state.pose.orientation.z = d->xquat[4 * body_id + 3];
                object_state.pose.orientation.w = d->xquat[4 * body_id];

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
    }

    std::unique_lock<std::mutex> lk(destroy_mtx);

    destroy_success = false;
    if (condition.wait_until(lk, std::chrono::system_clock::now() + 1000ms, [&]
                             { return destroy_success; }))
    {
        res.object_states = object_states;
        ROS_INFO("[Destroy #%d] Destroyed successfully", destroy_nr++);
    }
    else
    {
        res.object_states = std::vector<mujoco_msgs::ObjectState>();
        ROS_WARN("[Destroy #%d] Destroyed unsuccessfully", destroy_nr++);
    }
    return true;
}

void MjRos::destroy_objects(const std::set<std::string> object_names)
{
    destroy_success = MjSim::remove_body(object_names);
    for (const std::string &object_name : object_names)
    {
        object_names_to_destroy.erase(object_name);
        MjSim::spawned_object_body_names.erase(object_name);
        spawned_object_names.erase(object_name);
    }
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
            if (pub_tf_rate[EObjectType::SpawnedObject] > 1E-9)
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
        visualization_msgs::Marker destroy_marker;
        visualization_msgs::MarkerArray destroy_marker_array;

        destroy_marker.action = visualization_msgs::Marker::DELETE;

        std_msgs::Header header;
        header.frame_id = root_frame_id;

        // Set header
        header.stamp = ros::Time::now();

        destroy_marker.header = header;

        for (const std::string &object_name : object_names_to_destroy)
        {
            const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, object_name.c_str());
            destroy_marker.ns = object_name;
            for (int geom_id = m->body_geomadr[body_id]; geom_id < m->body_geomadr[body_id] + m->body_geomnum[body_id]; geom_id++)
            {
                if (geom_id == -1)
                {
                    continue;
                }
                destroy_marker.id = geom_id;
                destroy_marker_array.markers.push_back(destroy_marker);
            }
        }

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

        if (destroy_marker_array.markers.size() > 0)
        {
            // Publish destroy markers
            marker_array_pub.publish(destroy_marker_array);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_tf(const EObjectType object_type)
{
    if (object_type == EObjectType::None)
    {
        std::thread publish_tf_robot_thread(&MjRos::publish_tf, this, EObjectType::Robot);
        std::thread publish_tf_world_thread(&MjRos::publish_tf, this, EObjectType::World);
        std::thread publish_tf_spawned_object_thread(&MjRos::publish_tf, this, EObjectType::SpawnedObject);
        publish_tf_robot_thread.join();
        publish_tf_world_thread.join();
        publish_tf_spawned_object_thread.join();
        return;
    }

    if (pub_tf_rate[object_type] < 1E-9)
    {
        return;
    }

    geometry_msgs::TransformStamped transform;

    ros::Rate loop_rate(pub_tf_rate[object_type]);

    std_msgs::Header header;
    header.frame_id = root_frame_id;
    header.stamp = ros::Time::now();

    transform.header = header;

    // Publish tf of static objects
    if (object_type == EObjectType::World || EObjectType::SpawnedObject)
    {
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
    }

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        transform.header = header;

        set_ros_msg(*this, object_type, pub_tf_of_free_bodies_only, [&](MjRos &, const int body_id, const EObjectType object_type)
                    {
                                if (m->body_mocapid[body_id] != -1)
                                {
                                    return;
                                }

                                set_transform(transform, body_id, mj_id2name(m, mjtObj::mjOBJ_BODY, body_id));
                                br.sendTransform(transform); });

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_marker_array(const EObjectType object_type)
{
    if (object_type == EObjectType::None)
    {
        std::thread publish_marker_array_robot_thread(&MjRos::publish_marker_array, this, EObjectType::Robot);
        std::thread publish_marker_array_world_thread(&MjRos::publish_marker_array, this, EObjectType::World);
        std::thread publish_marker_array_spawned_object_thread(&MjRos::publish_marker_array, this, EObjectType::SpawnedObject);
        publish_marker_array_robot_thread.join();
        publish_marker_array_world_thread.join();
        publish_marker_array_spawned_object_thread.join();
        return;
    }

    if (pub_marker_array_rate[object_type] < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_marker_array_rate[object_type]);

    marker[object_type] = visualization_msgs::Marker();
    marker_array[object_type] = visualization_msgs::MarkerArray();
    marker[object_type].action = visualization_msgs::Marker::MODIFY;
    marker[object_type].frame_locked = true;
    marker[object_type].lifetime = ros::Duration(2.0 / pub_marker_array_rate[object_type]);

    std_msgs::Header header;
    header.frame_id = root_frame_id;

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        marker[object_type].header = header;
        marker_array[object_type].markers.clear();

        set_ros_msg(*this, object_type, pub_object_marker_array_of_free_bodies_only, &MjRos::add_marker);

        // Publish markers
        if (marker_array.size() > 0)
        {
            marker_array_pub.publish(marker_array[object_type]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_object_state_array(const EObjectType object_type)
{
    if (object_type == EObjectType::None)
    {
        std::thread publish_object_state_array_robot_thread(&MjRos::publish_object_state_array, this, EObjectType::Robot);
        std::thread publish_object_state_array_world_thread(&MjRos::publish_object_state_array, this, EObjectType::World);
        std::thread publish_object_state_array_spawned_object_thread(&MjRos::publish_object_state_array, this, EObjectType::SpawnedObject);
        publish_object_state_array_robot_thread.join();
        publish_object_state_array_world_thread.join();
        publish_object_state_array_spawned_object_thread.join();
        return;
    }

    if (pub_object_state_array_rate[object_type] < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_object_state_array_rate[object_type]);

    std_msgs::Header header;
    header.frame_id = root_frame_id;

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        object_state_array[object_type] = mujoco_msgs::ObjectStateArray();
        object_state_array[object_type].header = header;
        object_state_array[object_type].object_states.clear();

        set_ros_msg(*this, object_type, pub_object_state_array_of_free_bodies_only, &MjRos::add_object_state);

        object_state_array_pub.publish(object_state_array[object_type]);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MjRos::publish_joint_states(const EObjectType object_type)
{
    if (object_type == EObjectType::None)
    {
        std::thread publish_joint_states_robot_thread(&MjRos::publish_joint_states, this, EObjectType::Robot);
        std::thread publish_joint_states_world_thread(&MjRos::publish_joint_states, this, EObjectType::World);
        std::thread publish_joint_states_spawned_object_thread(&MjRos::publish_joint_states, this, EObjectType::SpawnedObject);
        publish_joint_states_robot_thread.join();
        publish_joint_states_world_thread.join();
        publish_joint_states_spawned_object_thread.join();
        return;
    }

    if (pub_joint_states_rate[object_type] < 1E-9)
    {
        return;
    }

    ros::Rate loop_rate(pub_joint_states_rate[object_type]);

    std_msgs::Header header;

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        switch (object_type)
        {
        case EObjectType::Robot:
            header.frame_id = root_frame_id;
            break;

        case EObjectType::World:
            header.frame_id = root_frame_id;
            break;

        default:
            header.frame_id = "";
            break;
        }

        joint_states[object_type] = sensor_msgs::JointState();
        joint_states[object_type].header = header;

        joint_states[object_type].name.clear();
        joint_states[object_type].position.clear();
        joint_states[object_type].velocity.clear();
        joint_states[object_type].effort.clear();

        set_ros_msg(*this, object_type, false, &MjRos::add_joint_states);

        if (!joint_states[object_type].name.empty())
        {
            joint_states_pub[object_type].publish(joint_states[object_type]);
        }

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

    for (const std::string &robot : MjSim::robot_names)
    {
        nav_msgs::Odometry base_pose;
        base_pose.child_frame_id = root_names[robot];
        base_poses[robot] = base_pose;
    }

    while (ros::ok())
    {
        // Set header
        header.stamp = ros::Time::now();
        header.seq += 1;

        for (const std::string &robot : MjSim::robot_names)
        {
            base_poses[robot].header = header;
        }

        transform.header = header;

        // Publish tf of root
        int i = 0;
        for (const std::string &robot : MjSim::robot_names)
        {
            const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, robot.c_str());
            if (body_id != -1)
            {
                if (MjSim::robot_names.size() > 1)
                {
                    set_transform(transform, body_id, robot + "/" + root_names[robot]);
                }
                else
                {
                    set_transform(transform, body_id, root_names[robot]);
                }

                br.sendTransform(transform);

                if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] ||
                    MjSim::add_odom_joints[robot]["lin_odom_y_joint"] ||
                    MjSim::add_odom_joints[robot]["lin_odom_z_joint"] ||
                    MjSim::add_odom_joints[robot]["ang_odom_x_joint"] ||
                    MjSim::add_odom_joints[robot]["ang_odom_y_joint"] ||
                    MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
                {
                    set_base_pose(body_id, robot);
                    base_pose_pubs[robot].publish(base_poses[robot]);
                }
            }
            i++;
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

void MjRos::add_marker(const int body_id, const EObjectType object_type)
{
    for (int geom_id = m->body_geomadr[body_id]; geom_id < m->body_geomadr[body_id] + m->body_geomnum[body_id]; geom_id++)
    {
        if (geom_id == -1)
        {
            continue;
        }
        boost::filesystem::path mesh_path;
        switch (m->geom_type[geom_id])
        {
        case mjtGeom::mjGEOM_BOX:
            marker[object_type].type = visualization_msgs::Marker::CUBE;
            marker[object_type].mesh_resource = "";
            marker[object_type].scale.x = m->geom_size[3 * geom_id] * 2;
            marker[object_type].scale.y = m->geom_size[3 * geom_id + 1] * 2;
            marker[object_type].scale.z = m->geom_size[3 * geom_id + 2] * 2;
            break;

        case mjtGeom::mjGEOM_SPHERE:
            marker[object_type].type = visualization_msgs::Marker::SPHERE;
            marker[object_type].mesh_resource = "";
            marker[object_type].scale.x = m->geom_size[3 * geom_id] * 2;
            marker[object_type].scale.y = m->geom_size[3 * geom_id] * 2;
            marker[object_type].scale.z = m->geom_size[3 * geom_id] * 2;
            break;

        case mjtGeom::mjGEOM_CYLINDER:
            marker[object_type].type = visualization_msgs::Marker::CYLINDER;
            marker[object_type].scale.x = m->geom_size[3 * geom_id] * 2;
            marker[object_type].scale.y = m->geom_size[3 * geom_id] * 2;
            marker[object_type].scale.z = m->geom_size[3 * geom_id + 1] * 2;
            break;

        case mjtGeom::mjGEOM_MESH:
            marker[object_type].type = visualization_msgs::Marker::MESH_RESOURCE;
            mesh_path = boost::filesystem::relative(mesh_paths[mj_id2name(m, mjtObj::mjOBJ_MESH, m->geom_dataid[geom_id])].first, tmp_world_path.parent_path());
            if (!boost::filesystem::exists(tmp_world_path.parent_path() / mesh_path) || !mesh_path.has_extension())
            {
                ROS_WARN("Body %s: Mesh %s - %s not found in [%s]", mj_id2name(m, mjtObj::mjOBJ_BODY, body_id), mj_id2name(m, mjtObj::mjOBJ_MESH, m->geom_dataid[geom_id]), mesh_paths[std::string(mj_id2name(m, mjtObj::mjOBJ_MESH, m->geom_dataid[geom_id]))].first.c_str(), mesh_path.parent_path().c_str());
                continue;
            }
            marker[object_type].mesh_resource = "package://mujoco_sim/model/tmp/" + mesh_path.string();
            marker[object_type].scale.x = 1;
            marker[object_type].scale.y = 1;
            marker[object_type].scale.z = 1;
            break;

        default:
            break;
        }

        marker[object_type].ns = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
        marker[object_type].id = geom_id;
        marker[object_type].color.a = m->geom_rgba[4 * geom_id + 3];
        marker[object_type].color.r = m->geom_rgba[4 * geom_id];
        marker[object_type].color.g = m->geom_rgba[4 * geom_id + 1];
        marker[object_type].color.b = m->geom_rgba[4 * geom_id + 2];

        if (m->geom_type[geom_id] != mjtGeom::mjGEOM_MESH)
        {
            marker[object_type].pose.position.x = d->geom_xpos[3 * geom_id];
            marker[object_type].pose.position.y = d->geom_xpos[3 * geom_id + 1];
            marker[object_type].pose.position.z = d->geom_xpos[3 * geom_id + 2];

            mjtNum quat[4];
            mjtNum mat[9];
            for (int i = 0; i < 9; i++)
            {
                mat[i] = d->geom_xmat[9 * geom_id + i];
            }
            mju_mat2Quat(quat, mat);
            marker[object_type].pose.orientation.x = quat[1];
            marker[object_type].pose.orientation.y = quat[2];
            marker[object_type].pose.orientation.z = quat[3];
            marker[object_type].pose.orientation.w = quat[0];
        }
        else
        {
            mjtNum body_pos[3];
            body_pos[0] = d->xpos[3 * body_id];
            body_pos[1] = d->xpos[3 * body_id + 1];
            body_pos[2] = d->xpos[3 * body_id + 2];

            mjtNum body_quat[4];
            body_quat[0] = d->xquat[4 * body_id];
            body_quat[1] = d->xquat[4 * body_id + 1];
            body_quat[2] = d->xquat[4 * body_id + 2];
            body_quat[3] = d->xquat[4 * body_id + 3];

            mjtNum geom_pos[3] = {0, 0, 0};
            if (MjSim::geom_pose.count(geom_id) > 0)
            {
                geom_pos[0] = MjSim::geom_pose[geom_id][0];
                geom_pos[1] = MjSim::geom_pose[geom_id][1];
                geom_pos[2] = MjSim::geom_pose[geom_id][2];
            }

            mjtNum pos[3];
            mju_rotVecQuat(pos, geom_pos, body_quat);
            mju_addTo3(pos, body_pos);

            marker[object_type].pose.position.x = pos[0];
            marker[object_type].pose.position.y = pos[1];
            marker[object_type].pose.position.z = pos[2];

            mjtNum geom_quat[4] = {1, 0, 0, 0};
            if (MjSim::geom_pose.count(geom_id) > 0)
            {
                geom_quat[0] = MjSim::geom_pose[geom_id][3];
                geom_quat[1] = MjSim::geom_pose[geom_id][4];
                geom_quat[2] = MjSim::geom_pose[geom_id][5];
                geom_quat[3] = MjSim::geom_pose[geom_id][6];
            }

            mjtNum quat[4];
            mju_mulQuat(quat, body_quat, geom_quat);

            marker[object_type].pose.orientation.x = quat[1];
            marker[object_type].pose.orientation.y = quat[2];
            marker[object_type].pose.orientation.z = quat[3];
            marker[object_type].pose.orientation.w = quat[0];
        }

        marker_array[object_type].markers.push_back(marker[object_type]);
    }
}

void MjRos::add_object_state(const int body_id, const EObjectType object_type)
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

    object_state_array[object_type].object_states.push_back(object_state);
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

    if (mju_abs(sqrt_sum_square) < mjMINVAL)
    {
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
    }
    else
    {
        transform.transform.rotation.x = d->xquat[4 * body_id + 1] / sqrt_sum_square;
        transform.transform.rotation.y = d->xquat[4 * body_id + 2] / sqrt_sum_square;
        transform.transform.rotation.z = d->xquat[4 * body_id + 3] / sqrt_sum_square;
        transform.transform.rotation.w = d->xquat[4 * body_id] / sqrt_sum_square;
    }
}

void MjRos::set_base_pose(const int body_id, const std::string &robot)
{
    base_poses[robot].pose.pose.position.x = d->xpos[3 * body_id];
    base_poses[robot].pose.pose.position.y = d->xpos[3 * body_id + 1];
    base_poses[robot].pose.pose.position.z = d->xpos[3 * body_id + 2];
    base_poses[robot].pose.pose.orientation.x = d->xquat[4 * body_id + 1];
    base_poses[robot].pose.pose.orientation.y = d->xquat[4 * body_id + 2];
    base_poses[robot].pose.pose.orientation.z = d->xquat[4 * body_id + 3];
    base_poses[robot].pose.pose.orientation.w = d->xquat[4 * body_id];
    base_poses[robot].pose.covariance.assign(0.0);
    base_poses[robot].twist.covariance.assign(0.0);
}

void MjRos::add_joint_states(const int body_id, const EObjectType object_type)
{
    if (m->body_jntnum[body_id] == 1 && (m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_HINGE || m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_SLIDE))
    {
        if (object_type == EObjectType::SpawnedObject)
        {
            int parent_body_id = body_id;
            std::string parent_body_name;
            do
            {
                parent_body_id = m->body_parentid[parent_body_id];
                if (parent_body_id == -1)
                {
                    break;
                }
                parent_body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, parent_body_id);
            } while (spawned_object_names.find(parent_body_name) == spawned_object_names.end());

            joint_states[object_type].header.frame_id = parent_body_name;
        }
        const int joint_id = m->body_jntadr[body_id];
        const char *joint_name = mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id);
        const int qpos_id = m->jnt_qposadr[joint_id];
        const int dof_id = m->jnt_dofadr[joint_id];

        joint_states[object_type].name.push_back(joint_name);
        joint_states[object_type].position.push_back(d->qpos[qpos_id]);
        joint_states[object_type].velocity.push_back(d->qvel[dof_id]);
        joint_states[object_type].effort.push_back(d->qfrc_inverse[dof_id]);
    }
}