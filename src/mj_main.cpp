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

#include "mj_sim.h"
#ifdef VISUAL
#include "mj_visual.h"
#endif
#include "mj_hw_interface.h"
#include "mj_ros.h"
#include "mj_socket.h"

#include <controller_manager/controller_manager.h>
#include <thread>

static MjSim &mj_sim = MjSim::get_instance();
static MjSocket &mj_socket = MjSocket::get_instance();
#ifdef VISUAL
static MjVisual &mj_visual = MjVisual::get_instance();
#endif

static int i = 0;

#ifdef VISUAL
// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
    {
        MjSim::add_data();
    }
}
#endif

void controller(const mjModel *m, mjData *d)
{
    mj_sim.controller();
}

void simulate()
{
    std::vector<MjHWInterface *> mj_hw_interfaces;
    std::vector<controller_manager::ControllerManager *> controller_managers;
    for (const std::string &robot_name : MjSim::robot_names)
    {
        mj_hw_interfaces.push_back(new MjHWInterface(robot_name));
        if (MjSim::robot_names.size() < 2)
        {
            controller_managers.push_back(new controller_manager::ControllerManager(mj_hw_interfaces.back()));
        }
        else
        {
            controller_managers.push_back(new controller_manager::ControllerManager(mj_hw_interfaces.back(), ros::NodeHandle(robot_name)));
        }
    }

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Time last_sim_time = MjRos::ros_start;
    double time_step = m->opt.timestep;

    while (ros::ok())
    {
        if (MjSocket::enable)
        {
            mj_socket.communicate();          
        }
        {
            ros::Time sim_time = (ros::Time)(MjRos::ros_start.toSec() + d->time);
            ros::Duration sim_period = sim_time - last_sim_time;

            mtx.lock();
            mj_step1(m, d);
            // check if we should update the controllers
            if (sim_period.toSec() >= 1 / 10000.) // Controller with 10kHz
            {
                // store simulation time
                last_sim_time = sim_time;

                // update the robot simulation with the state of the mujoco model
                for (MjHWInterface *mj_hw_interface : mj_hw_interfaces)
                {
                    mj_hw_interface->read();
                }

                // compute the controller commands
                for (controller_manager::ControllerManager *controller_manager : controller_managers)
                {
                    controller_manager->update(sim_time, sim_period);
                }
            }
            // update the mujoco model with the result of the controller
            for (MjHWInterface *mj_hw_interface : mj_hw_interfaces)
            {
                mj_hw_interface->write();
            }

            mj_step2(m, d);

            mj_sim.set_odom_vels();

            mtx.unlock();
        }

        // Calculate real time factor
        int num_step = mju_ceil(1 / m->opt.timestep);
        static std::deque<double> last_sim_time;
        static std::deque<double> last_ros_time;
        double error_time;
        double ros_time;
        double sim_time = d->time - MjSim::sim_start;
        if (i == 0)
        {
            last_sim_time.clear();
            last_ros_time.clear();
        }
        do
        {
            ros_time = (ros::Time::now() - MjRos::ros_start).toSec();
            error_time = ros_time - sim_time;
        } while (error_time < -1E-6 && i != 0);

        sim_time = d->time - MjSim::sim_start;
        last_ros_time.push_front(ros_time);
        last_sim_time.push_front(sim_time);
        if (i == num_step)
        {
            double ros_time_diff = ros_time - last_ros_time.back();
            double sim_time_diff = sim_time - last_sim_time.back();
            rtf = sim_time_diff / ros_time_diff;
            last_ros_time.pop_back();
            last_sim_time.pop_back();
        }
        else
        {
            i++;
        }

        // Change timestep when out of sync
        if (error_time > 1E-3)
        {
            if (m->opt.timestep < MjSim::max_time_step)
            {
                m->opt.timestep *= 2;
            }
        }
        else
        {
            if (m->opt.timestep > time_step)
            {
                m->opt.timestep /= 2;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mujoco_sim");
    int port = 7500;
    if (argc > 1)
    {
        port = std::stoi(argv[1]);
    }
    ros::NodeHandle n;

    ROS_INFO("Get ROS parameters from the server...");
    MjRos::set_params();

    ROS_INFO("Initializing the MuJoCo simulator...");
    mj_sim.init();
    ROS_INFO("Initialized the MuJoCo simulator successfully.");

    ROS_INFO("Initializing the ROS interface...");
    MjRos &mj_ros = MjRos::get_instance();
    mj_ros.init();
    ROS_INFO("Initialized the ROS interface successfully.");

#ifdef VISUAL
    ROS_INFO("Initializing OpenGL...");
    mj_visual.init();
    glfwSetKeyCallback(mj_visual.window, keyboard);
    ROS_INFO("Initialized OpenGL successfully.");
#endif

    MjSocket &mj_socket = MjSocket::get_instance();
    mj_socket.init(port);
    if ((MjSocket::send_objects.size() > 0 || MjSocket::receive_objects.size() > 0))
    {
        mj_socket.send_meta_data();
    }

    mjcb_control = controller;

    std::thread ros_thread1(&MjRos::setup_publishers, &mj_ros);
    std::thread ros_thread2(&MjRos::setup_service_servers, &mj_ros);
    std::thread ros_thread3(&MjRos::get_controlled_joints, &mj_ros);

    // start simulation thread
    std::thread sim_thread(simulate);

    mjtNum sim_step_start = d->time;

#ifdef VISUAL
    while (ros::ok())
    {
        if (mj_visual.is_window_closed())
        {
            break;
        }

        if (d->time - sim_step_start > 1.0 / 60.0)
        {
            mj_visual.render(d->time - MjSim::sim_start, (ros::Time::now() - MjRos::ros_start).toSec());
            sim_step_start = d->time;
        }
    }
    ros::shutdown();
#else
    ros::waitForShutdown();
#endif

    ros_thread1.join();
    ros_thread2.join();
    ros_thread3.join();
    sim_thread.join();

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}