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

#include <algorithm>
#include <ros/package.h>
#include <tinyxml2.h>

std::vector<std::string> MjSim::joint_names;

std::map<std::string, MimicJoint> MjSim::mimic_joints;

std::vector<std::string> MjSim::link_names;

mjtNum *MjSim::tau = NULL;

mjtNum MjSim::sim_start;

MjSim::~MjSim()
{
	mju_free(tau);
	boost::filesystem::remove_all(tmp_mesh_path.parent_path().parent_path());
	boost::filesystem::remove(tmp_model_path);
}

static void get_joint_names(tinyxml2::XMLElement *parent_body_element)
{
	for (tinyxml2::XMLElement *joint_element = parent_body_element->FirstChildElement();
		 joint_element != nullptr;
		 joint_element = joint_element->NextSiblingElement())
	{
		if (strcmp(joint_element->Value(), "joint") == 0)
		{
			MjSim::joint_names.push_back(joint_element->Attribute("name"));
		}
	}
}

static void get_body_element(tinyxml2::XMLElement *parent_body_element)
{
	for (tinyxml2::XMLElement *body_element = parent_body_element->FirstChildElement();
		 body_element != nullptr;
		 body_element = body_element->NextSiblingElement())
	{
		if (strcmp(body_element->Value(), "body") == 0)
		{
			get_joint_names(body_element);
			get_body_element(body_element);
		}
	}
}

static void set_joint_names()
{
	tinyxml2::XMLDocument xml_doc;
	if (xml_doc.LoadFile(model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", model_path.c_str());
		return;
	}
	for (tinyxml2::XMLElement *worldbody_element = xml_doc.FirstChildElement()->FirstChildElement();
		 worldbody_element != nullptr;
		 worldbody_element = worldbody_element->NextSiblingElement())
	{
		if (strcmp(worldbody_element->Value(), "worldbody") == 0)
		{
			get_body_element(worldbody_element);
		}
	}
}

static void load_model()
{
	// load and compile model
	char error[1000] = "Could not load binary model";
	m = mj_loadXML(tmp_model_path.c_str(), 0, error, 1000);
	if (!m)
	{
			mju_error_s("Could not load model file '%s'", tmp_model_path.c_str());
	}

	// make data
	d = mj_makeData(m);
}

static void init_tmp()
{
	std::string model_path_tail = model_path.stem().string() + "/meshes/";
	tmp_model_path = ros::package::getPath("mujoco_sim") + "/model/tmp/";
	tmp_mesh_path = tmp_model_path / model_path_tail;
	
	if (boost::filesystem::exists(tmp_mesh_path))
	{
		boost::filesystem::remove_all(tmp_mesh_path.parent_path());
	}
	boost::filesystem::create_directories(tmp_mesh_path);
	
	for (const boost::filesystem::directory_entry& file : boost::filesystem::directory_iterator(model_path.parent_path() / model_path_tail))
	{
		boost::filesystem::copy_file(file.path(), tmp_mesh_path / file.path().filename());
	}
	tmp_model_path /= "current.xml";

	boost::filesystem::copy_file(model_path, tmp_model_path);

	// Modify current.xml
	tinyxml2::XMLDocument current_xml_doc;
	if (current_xml_doc.LoadFile(config_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", config_path.c_str());
		return;
	}
	// Add default.xml to current.xml
	tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	include_element->SetAttribute("file", boost::filesystem::relative(model_path, tmp_model_path.parent_path()).c_str());
	current_element->LinkEndChild(include_element);
	current_xml_doc.SaveFile(tmp_model_path.c_str());
}

static void init_malloc()
{
	MjSim::tau = (mjtNum *)mju_malloc(m->nv * sizeof(mjtNum *));
	mju_zero(MjSim::tau, m->nv);
}

void MjSim::init()
{
	init_tmp();
	load_model();
	init_malloc();
	set_joint_names();
	sim_start = d->time;

	for (const std::string joint_name : joint_names)
	{
		int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
	}
}

void MjSim::add_data()
{
	// Save current.xml
	char error[1000] = "Could not save binary model";
	mj_saveLastXML(tmp_model_path.c_str(), m, error, 1000);

	// Modify current.xml
	tinyxml2::XMLDocument current_xml_doc;
	if (current_xml_doc.LoadFile(tmp_model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", tmp_model_path.c_str());
		return;
	}
	tinyxml2::XMLElement *worldbody_element = current_xml_doc.FirstChildElement()->FirstChildElement();
	for (tinyxml2::XMLElement *worldbody_element = current_xml_doc.FirstChildElement()->FirstChildElement();
		 worldbody_element != nullptr;
		 worldbody_element = worldbody_element->NextSiblingElement())
	{
		if (strcmp(worldbody_element->Value(), "worldbody") == 0)
		{

			for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement();
				 body_element != nullptr;
				 body_element = body_element->NextSiblingElement())
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
			}
		}
	}

	// Add add.xml to current.xml
	tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	include_element->SetAttribute("file", "add.xml");
	current_element->LinkEndChild(include_element);
	current_xml_doc.SaveFile(tmp_model_path.c_str());

	// Load current.xml
	mjModel *m_new = mj_loadXML(tmp_model_path.c_str(), 0, error, 1000);
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

void MjSim::set_mimic_joint()
{
	for (const std::pair<std::string, MimicJoint> &mimic_joint : mimic_joints)
	{
		const std::string joint_name = mimic_joint.first;
		const int joint_idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
		const std::string from_joint_name = mimic_joint.second.from_joint;
		const int from_joint_idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, from_joint_name.c_str());

		d->qpos[joint_idx] = mimic_joint.second.multiplier * d->qpos[from_joint_idx] + mimic_joint.second.offset;
	}
}