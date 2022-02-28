#include "mj_model.h"
#include "mj_sim.h"
#ifdef VISUAL
#include "mj_visual.h"
#endif
#include "mj_ros.h"
#include "thread"

static MjSim mj_sim;
static MjRos mj_ros;
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

void controller(const mjModel *m, mjData *d)
{
  mj_sim.computed_torque_controller();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mujoco_sim", ros::init_options::AnonymousName);

  load_model(argc, argv);

  ros::NodeHandle n;
  mj_ros.init(n);
  mj_sim.init();

#ifdef VISUAL
  mj_visual.init();
#endif

  mjcb_control = controller;

  std::thread publish_joint_state_thread(&MjRos::publish_joint_state, &mj_ros);
  std::thread publish_follow_joint_traj_feedback_thread(&MjRos::publish_follow_joint_traj_feedback, &mj_ros);
  
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
    double error = (ros::Time().now().toSec() - MjRos::ros_start.toSec()) - (d->time - MjSim::sim_start);
    m->opt.timestep *= 1 + mju_pow(mju_abs(error), REDUCE) * mju_sign(error);

#ifdef VISUAL
    mj_visual.render();
#endif
  }

#ifdef VISUAL
  mj_visual.terminate();
#endif

  publish_joint_state_thread.join();
  publish_follow_joint_traj_feedback_thread.join();

  // free MuJoCo model and data, deactivate
  mj_deleteData(d);
  mj_deleteModel(m);

  return 0;
}