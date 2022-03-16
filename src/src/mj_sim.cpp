#include "mj_sim.h"
#include <iostream>
#include <ros/package.h>
#include <string.h>

std::vector<std::string> MjSim::q_names;

std::map<std::string, mjtNum> MjSim::q_inits;

mjtNum *MjSim::u = NULL;

mjtNum MjSim::sim_start;

void MjSim::init_malloc()
{
    u = (mjtNum *)malloc(m->nv * sizeof(mjtNum *)); mju_zero(u, m->nv);
    
    tau = (mjtNum *)malloc(m->nv * sizeof(mjtNum *)); mju_zero(tau, m->nv);
}

void MjSim::init()
{
    init_malloc();
    sim_start = d->time;

    for (const std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        d->qpos[idx] = q_inits[q_name];
    }
    mj_forward(m, d);
}

void MjSim::add_data(const std::string data_xml_path)
{
  std::string path = ros::package::getPath("mujoco_sim");

  std::string current_xml = "current.xml";
  std::string current_xml_path = path + "/model/tmp/" + current_xml;
  std::string new_xml = "new.xml";
  std::string new_xml_path = path + "/model/tmp/" + new_xml;

  char error[1000] = "Could not load binary model";
  mj_saveLastXML(current_xml_path.c_str(), m, error, 1000);

  FILE *fptr = NULL;
  char buff[255] = {0};
  char content[100000] = {0};

  fptr = fopen(data_xml_path.c_str(), "r");

  if (fgets(buff, 255, fptr))
  {
    strcat(content, buff);
  }
  strcat(content, "\t<include file=\"");
  strcat(content, current_xml.c_str());
  strcat(content, "\"/>\n");
  while (fgets(buff, 255, fptr))
  {
    strcat(content, buff);
  }
  fclose(fptr);
  
  fptr = fopen(new_xml_path.c_str(), "w");
  fprintf(fptr, "%s", content);
  fclose(fptr);

  mjModel *m_new = mj_loadXML(new_xml_path.c_str(), 0, error, 1000);
  
  if (!m_new)
  {
    mju_error_s("Load model error: %s", error);
  }

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
  m = mj_copyModel(NULL, m_new);

  init_malloc();
}

void MjSim::controller()
{
    mj_mulM(m, d, tau, u);
    for (const std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        tau[idx] += d->qfrc_bias[idx];
    }
    
    mju_copy(d->qfrc_applied, tau, m->nv);
}