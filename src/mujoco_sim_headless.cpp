#include "mj_sim.h"
#ifdef VISUAL
#include "mj_visual.h"
#endif
#include "mj_hw_interface.h"
#include <controller_manager/controller_manager.h>
#include <thread>
#include <ros/package.h>

static MjSim mj_sim;
#ifdef VISUAL
static MjVisual mj_visual;
#endif

#define REDUCE 0.5

void load_model(int argc, char **argv)
{
  // check command-line arguments
  if (argc != 2)
  {
    mju_error("\n Usage:  model.xml\n");
  }

  char error[1000] = "Could not load binary model";
  // load and compile model
  m = mj_loadXML(argv[1], 0, error, 1000);
  if (!m)
  {
    mju_error_s("Could not load model file '%s'", argv[1]);
  }

  // make data
  d = mj_makeData(m);
}

#ifdef VISUAL
// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
  if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
  {
    std::string path = ros::package::getPath("mujoco_sim");
    std::string file_name = "ball.xml";
    std::string data_xml_path = path + "/model/tmp/" + file_name;
    mj_sim.add_data(data_xml_path);
  }
}
#endif

void controller(const mjModel *m, mjData *d)
{
  mj_sim.controller();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_sim");
  ros::NodeHandle n;

  load_model(argc, argv);

  mj_sim.init();

#ifdef VISUAL
  mj_visual.init();
  glfwSetKeyCallback(mj_visual.window, keyboard);
#endif

  mjcb_control = controller;

  MjHWInterface mj_hw_interface;
  controller_manager::ControllerManager controller_manager(&mj_hw_interface);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  const ros::Time ros_start = ros::Time::now();
  ros::Time last_sim_time = ros_start;

  while (ros::ok())
  {
#ifdef VISUAL
    if (mj_visual.is_window_closed())
    {
      break;
    }
#endif

    mjtNum sim_step_start = d->time;
    while (d->time - sim_step_start < 1.0 / 60.0)
    {
      ros::Time sim_time = (ros::Time)(ros_start.toSec() + d->time);
      ros::Duration sim_period = sim_time - last_sim_time;

      mj_step1(m, d);
      // check if we should update the controllers
      if (sim_period.toSec() >= 1 / 10000.) // Controller with 10kHz, start from 0.1s to avoid unstable
      {
        // store simulation time
        last_sim_time = sim_time;

        // update the robot simulation with the state of the mujoco model
        mj_hw_interface.read();

        // compute the controller commands
        controller_manager.update(sim_time, sim_period);
      }
      // update the mujoco model with the result of the controller
      mj_hw_interface.write();

      mj_step2(m, d);
    }

    // Change timestep when out of sync
    double error = (ros::Time::now() - ros_start).toSec() - (d->time - MjSim::sim_start);
    if (mju_abs(error) > 0.1)
    {
      m->opt.timestep *= 1 + mju_pow(mju_abs(error), REDUCE) * mju_sign(error);
      if (m->opt.timestep > 0.01)
      {
        m->opt.timestep = 0.01;
      }
    }

#ifdef VISUAL
    mj_visual.render();
#endif
  }

#ifdef VISUAL
  mj_visual.terminate();
#endif

  // free MuJoCo model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);

  return 0;
}