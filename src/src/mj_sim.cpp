#include "mj_sim.h"
#include <iostream>
#include <ros/package.h>
#include <string.h>
#include <tinyxml2.h>
#include <algorithm>

std::vector<std::string> MjSim::joint_names;

std::vector<std::string> MjSim::link_names;

std::map<std::string, mjtNum> MjSim::q_inits;

mjtNum *MjSim::u = NULL;

mjtNum *MjSim::tau = NULL;

mjtNum MjSim::sim_start;

void MjSim::init_malloc()
{
  u = (mjtNum *)malloc(m->nv * sizeof(mjtNum *));
  mju_zero(u, m->nv);

  tau = (mjtNum *)malloc(m->nv * sizeof(mjtNum *));
  mju_zero(tau, m->nv);
}

void MjSim::init()
{
  init_malloc();
  sim_start = d->time;

  for (const std::string joint_name : joint_names)
  {
    int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
    d->qpos[idx] = q_inits[joint_name];
  }
  mj_forward(m, d);
}

void MjSim::add_data()
{
  std::string path = ros::package::getPath("mujoco_sim");

  std::string current_xml = "current.xml";
  std::string current_xml_path = path + "/model/tmp/" + current_xml;

  char error[1000] = "Could not load binary model";
  // Load current.xml
  mjModel *m_new = mj_loadXML(current_xml_path.c_str(), 0, error, 1000);
  if (!m_new)
  {
    mju_warning_s("Load model error: %s", error);
    return;
  }

  mtx.lock();
  // make data
  mjData *d_new = mj_makeData(m_new);
  d_new->time = d->time;

  mju_copy(d_new->qpos, d->qpos, m->nq);

  mju_copy(d_new->qvel, d->qvel, m->nv);
  mju_copy(d_new->qacc_warmstart, d->qacc_warmstart, m->nv);
  mju_copy(d_new->qfrc_applied, d->qfrc_applied, m->nv);
  mju_copy(d_new->qacc, d->qacc, m->nv);

  mju_copy(d_new->act, d->act, m->na);
  mju_copy(d_new->act_dot, d->act_dot, m->na);
  mju_copy(d_new->qfrc_applied, d->qfrc_applied, m->na);
  mju_copy(d_new->qacc, d->qacc, m->na);

  mju_copy(d_new->xfrc_applied, d->xfrc_applied, m->nbody * 6);

  mju_copy(d_new->mocap_pos, d->mocap_pos, 3 * m->nmocap);
  mju_copy(d_new->mocap_quat, d->mocap_quat, 4 * m->nmocap);

  mju_copy(d_new->userdata, d->userdata, m->nuserdata);

  mju_copy(d_new->sensordata, d->sensordata, m->nsensordata);

  d = d_new;
  m = m_new;

  init_malloc();
  mtx.unlock();
}

void MjSim::controller()
{
  mj_mulM(m, d, tau, u);
  for (const std::string joint_name : joint_names)
  {
    int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
    tau[idx] += d->qfrc_bias[idx];
  }

  mju_copy(d->qfrc_applied, tau, m->nv);
}