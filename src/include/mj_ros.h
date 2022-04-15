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

#pragma once

#include "mj_sim.h"
#include "mujoco_msgs/DestroyObject.h"
#include "mujoco_msgs/ObjectInfo.h"
#include "mujoco_msgs/ObjectState.h"
#include "mujoco_msgs/ObjectStatus.h"
#include "mujoco_msgs/SpawnObject.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>

class MjRos
{
public:
    MjRos() = default;

    ~MjRos();

public:
    /**
     * @brief Initialize publishers, subscribers and MjRos members from rosparam
     *
     */
    void init();

    /**
     * @brief Update publishers
     *
     */
    void update(double frequency);

private:
    /**
     * @brief Create new object from ROS
     *
     * @param req New object parameters
     * @param res Success or not
     * @return true Success
     * @return false Fail
     */
    bool spawn_objects_service(mujoco_msgs::SpawnObjectRequest &req, mujoco_msgs::SpawnObjectResponse &res);

    /**
     * @brief Destroy object from ROS
     *
     * @param req Array of object names
     * @param res Success or not
     * @return true Success
     * @return false Fail
     */
    bool destroy_objects_service(mujoco_msgs::DestroyObjectRequest &req, mujoco_msgs::DestroyObjectResponse &res);

    /**
     * @brief Control base velocity from ROS
     *
     * @param msg cmd_vel message from ROS
     */
    void cmd_vel_callback(const geometry_msgs::Twist &msg);

    /**
     * @brief Publish markers for all objects
     *
     * @param body_idx Body index of the object
     * @param object_name Name of the object (object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_idx))
     */
    void publish_markers(int body_idx, std::string object_name);

    /**
     * @brief Set transform of an object
     *
     * @param body_idx Body index of the object
     * @param object_name Name of the object (object_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_idx))
     */
    void set_transform(int body_idx, std::string object_name);

public:
    /**
     * @brief Start time of ROS
     *
     */
    static ros::Time ros_start;

private:
    ros::NodeHandle n;

    ros::Subscriber cmd_vel_sub;

    ros::ServiceServer spawn_objects_server;

    ros::ServiceServer destroy_objects_server;

    ros::Publisher vis_pub;

    ros::Publisher base_pub;

    tf2_ros::TransformBroadcaster br;

    visualization_msgs::Marker marker;

    geometry_msgs::TransformStamped transform;
};

/**
 * @brief This function `urdf::Model::initParamWithNodeHandle` should be inside the implementation file, but c++ doesn't want to compile...
 * It makes no sense to put it here, I know...
 * If anyone know how to fix this, please kindly tell me, you will earn a lot of respect from me and my friends :) May your name be honoured!
 *
 * @param urdf_model The urdf model to be initialized
 * @param n The node handle
 * @return true Successfully initialized
 * @return false Fail to initialize
 */
bool init_urdf(urdf::Model &urdf_model, const ros::NodeHandle &n)
{
    if (n.hasParam("robot_description"))
    {
        return urdf_model.initParamWithNodeHandle("robot_description", n);
    }
    else
    {
        ROS_WARN("robot_description not found");
        return false;
    }
}