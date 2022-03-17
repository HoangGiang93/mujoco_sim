#include "mj_sim.h"
#include <iostream>
#include <ros/package.h>
#include <string.h>
#include <tinyxml2.h>

std::vector<std::string> MjSim::joint_names;

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

void MjSim::add_data(const std::string data_xml_path)
{
  std::string path = ros::package::getPath("mujoco_sim");

  tinyxml2::XMLDocument data_xml_doc;
  if (data_xml_doc.LoadFile(data_xml_path.c_str()) != tinyxml2::XML_SUCCESS)
  {
    mju_warning_s("Failed to load file \"%s\"\n", data_xml_path.c_str());
    return;
  }
  tinyxml2::XMLElement *data_element = data_xml_doc.FirstChildElement();
  if (strcmp(data_element->Value(), "mujoco") != 0)
  {
    mju_warning_s("Failed to read file \"%s\", first tag should be <mujoco>\n", data_xml_path.c_str());
    return;
  }
  data_element = data_element->FirstChildElement();
  static int idx = 0;
  while (data_element != nullptr)
  {
    if (strcmp(data_element->Value(), "worldbody") == 0)
    {
      tinyxml2::XMLElement *worldbody_element = data_element->FirstChildElement();
      while (worldbody_element != nullptr)
      {
        if (strcmp(worldbody_element->Value(), "body") == 0)
        {
          if (worldbody_element->Attribute("name") == nullptr)
          {
            std::string name_with_idx = "object_" + std::to_string(idx++);
            worldbody_element->SetAttribute("name", name_with_idx.c_str());
          }
        }
        worldbody_element = worldbody_element->NextSiblingElement();
      }
    }
    data_element = data_element->NextSiblingElement();
  }
  std::string add_xml = "add.xml";
  data_xml_doc.SaveFile((path + "/model/tmp/" + add_xml).c_str());

  std::string current_xml = "current.xml";
  std::string current_xml_path = path + "/model/tmp/" + current_xml;

  char error[1000] = "Could not load binary model";
  mj_saveLastXML(current_xml_path.c_str(), m, error, 1000);

  tinyxml2::XMLDocument current_xml_doc;
  if (current_xml_doc.LoadFile(current_xml_path.c_str()) != tinyxml2::XML_SUCCESS)
  {
    mju_warning_s("Failed to load file \"%s\"\n", current_xml_path.c_str());
    return;
  }

  tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
  tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");

  include_element->SetAttribute("file", add_xml.c_str());
  current_element->LinkEndChild(include_element);
  current_xml_doc.SaveFile(current_xml_path.c_str());
  
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
  m = mj_copyModel(NULL, m_new);

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