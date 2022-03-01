#include "mj_sim.h"
#ifdef VISUAL
#include "mj_visual.h"
#endif
#include "mj_hw_interface.h"
#include "controller_manager/controller_manager.h"
#include "thread"

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

void ros_run(void)
{
  MjHWInterface mj_hw_interface;
  controller_manager::ControllerManager controller_manager(&mj_hw_interface);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  const ros::Time time_start = ros::Time::now();
  ros::Time time = ros::Time::now();
  ros::Duration period;
  while (ros::ok())
  {
    period = ros::Time::now() - time;
    time = ros::Time::now();

    mj_hw_interface.read(time, period);
    controller_manager.update(time, period);
    mj_hw_interface.write(time, period);
  }
}

void controller(const mjModel *m, mjData *d)
{
  mj_sim.computed_torque_controller();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_sim", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  load_model(argc, argv);
  mj_sim.init();
#ifdef VISUAL
  mj_visual.init();
#endif

  std::thread ros_thread(ros_run);

  mjcb_control = controller;
  const ros::Time time_start = ros::Time::now();
  while (ros::ok())
  {
#ifdef VISUAL
    if (mj_visual.is_window_closed())
    {
      break;
    }
#endif

    mjtNum simstart = d->time;

    while (d->time - simstart < 1.0 / 60.0)
    {
      mtx.lock();
      mj_step(m, d);
      mtx.unlock();
    }
    double error = ((ros::Time::now() - time_start).toSec()) - (d->time - MjSim::sim_start);
    m->opt.timestep *= 1 + mju_pow(mju_abs(error), REDUCE) * mju_sign(error);

#ifdef VISUAL
    mj_visual.render();
#endif
  }

#ifdef VISUAL
  mj_visual.terminate();
#endif
  ros_thread.join();

  // free MuJoCo model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);

  return 0;
}