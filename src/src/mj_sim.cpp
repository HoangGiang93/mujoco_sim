#include "mj_sim.h"

#include <algorithm>
#include <ros/package.h>
#include <tinyxml2.h>

std::vector<std::string> MjSim::joint_names;

std::vector<std::string> MjSim::link_names;

std::map<std::string, mjtNum> MjSim::q_inits;

mjtNum *MjSim::tau = NULL;

mjtNum MjSim::sim_start;

MjSim::~MjSim()
{
  free(tau);
  std::experimental::filesystem::remove_all(tmp_model_path.parent_path());
}

void MjSim::init_malloc()
{
  tau = (mjtNum *)malloc(m->nv * sizeof(mjtNum *));
  mju_zero(tau, m->nv);
}

void MjSim::init_tmp()
{
  tmp_model_path = ros::package::getPath("mujoco_sim") + "/model/tmp/" + model_path.stem().string() + "/meshes";
  std::experimental::filesystem::create_directories(tmp_model_path);
  copy(model_path.parent_path() / model_path.stem().c_str() / "meshes", tmp_model_path);
}

void MjSim::init()
{
  init_tmp();
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

  // Save current.xml
  std::string current_xml = "current.xml";
  std::string current_xml_path = path + "/model/tmp/" + current_xml;

  char error[1000] = "Could not load binary model";
  mj_saveLastXML(current_xml_path.c_str(), m, error, 1000);

  // Modify current.xml
  tinyxml2::XMLDocument current_xml_doc;
  if (current_xml_doc.LoadFile(current_xml_path.c_str()) != tinyxml2::XML_SUCCESS)
  {
    mju_warning_s("Failed to load file \"%s\"\n", current_xml_path.c_str());
    return;
  }
  tinyxml2::XMLElement *worldbody_element = current_xml_doc.FirstChildElement()->FirstChildElement();
  while (worldbody_element != nullptr)
  {
    if (strcmp(worldbody_element->Value(), "worldbody") == 0)
    {
      tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement();
      while (body_element != nullptr)
      {
        if (strcmp(body_element->Value(), "body") == 0)
        {
          const char *body_name = body_element->Attribute("name");
          if (body_name != nullptr && std::find(MjSim::link_names.begin(), MjSim::link_names.end(), body_name) == MjSim::link_names.end())
          {
            int body_idx = mj_name2id(m, mjtObj::mjOBJ_BODY, body_name);
            body_element->SetAttribute("pos",
                                       (std::to_string(d->xpos[3 * body_idx]) + " " +
                                        std::to_string(d->xpos[3 * body_idx + 1]) + " " +
                                        std::to_string(d->xpos[3 * body_idx + 2]))
                                           .c_str());
            body_element->SetAttribute("quat",
                                       (std::to_string(d->xquat[4 * body_idx]) + " " +
                                        std::to_string(d->xquat[4 * body_idx + 1]) + " " +
                                        std::to_string(d->xquat[4 * body_idx + 2]) + " " +
                                        std::to_string(d->xquat[4 * body_idx + 3]))
                                           .c_str());
          }
        }
        body_element = body_element->NextSiblingElement();
      }
    }
    worldbody_element = worldbody_element->NextSiblingElement();
  }

  // Add add.xml to current.xml
  tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
  tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
  include_element->SetAttribute("file", "add.xml");
  current_element->LinkEndChild(include_element);
  current_xml_doc.SaveFile(current_xml_path.c_str());

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
  mju_copy(d->qfrc_applied, tau, m->nv);
}