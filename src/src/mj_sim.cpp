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
#include <ros/param.h>
#include <tinyxml2.h>

std::vector<std::string> MjSim::joint_names;

std::map<std::string, std::pair<std::string, mjtNum>> MjSim::odom_joints = {{"odom_x_joint", {"odom_x_joint", 0.0}}, {"odom_y_joint", {"odom_y_joint", 0.0}}, {"odom_z_joint", {"odom_z_joint", 0.0}}};

std::map<std::string, MimicJoint> MjSim::mimic_joints;

std::vector<std::string> MjSim::link_names;

mjtNum *MjSim::tau = NULL;

mjtNum MjSim::sim_start;

static boost::filesystem::path tmp_mesh_path;

MjSim::~MjSim()
{
	mju_free(tau);
	boost::filesystem::remove_all(tmp_mesh_path.parent_path().parent_path());
	boost::filesystem::remove(tmp_model_path);
	boost::filesystem::remove(cache_model_path);
}

static void set_param()
{
	std::string model_path_string;
	if (ros::param::get("~robot", model_path_string))
	{
		model_path = model_path_string;
		tmp_model_name = "current_" + model_path.filename().string();
	}

	std::string config_path_string;
	if (ros::param::get("~config", config_path_string))
	{
		config_path = config_path_string;
	}

	if (ros::param::get("~use_odom_joints", use_odom_joints))
	{
		std::map<std::string, std::string> odom_joints;
		if (ros::param::get("~odom_joints", odom_joints))
		{
			if (odom_joints.find("odom_x_joint") != odom_joints.end() && odom_joints.find("odom_y_joint") != odom_joints.end() && odom_joints.find("odom_z_joint") != odom_joints.end())
			{
				MjSim::odom_joints = {{"odom_x_joint", {odom_joints["odom_x_joint"], 0.0}}, {"odom_y_joint", {odom_joints["odom_y_joint"], 0.0}}, {"odom_z_joint", {odom_joints["odom_z_joint"], 0.0}}};
			}
		}
	}
}

static void get_joint_names(tinyxml2::XMLElement *parent_body_element)
{
	for (tinyxml2::XMLElement *joint_element = parent_body_element->FirstChildElement();
			 joint_element != nullptr;
			 joint_element = joint_element->NextSiblingElement())
	{
		if (strcmp(joint_element->Value(), "joint") == 0)
		{
			std::string joint_name = joint_element->Attribute("name");
			if (MjSim::odom_joints.count(joint_name))
			{
				continue;
			}
			MjSim::joint_names.push_back(joint_name);
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

void load_model()
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
	// Create directory tmp_mesh_path if not exist
	std::string model_path_tail = model_path.stem().string() + "/meshes/";
	tmp_mesh_path = tmp_model_path / model_path_tail;

	if (!boost::filesystem::exists(tmp_mesh_path))
	{
		boost::filesystem::create_directories(tmp_mesh_path);
	}

	// Copy model meshes to tmp_mesh_path
	for (const boost::filesystem::directory_entry &file : boost::filesystem::directory_iterator(model_path.parent_path() / model_path_tail))
	{
		boost::filesystem::path des_file_path = tmp_mesh_path / file.path().filename();
		if (!boost::filesystem::exists(des_file_path))
		{
			boost::filesystem::copy_file(file.path(), des_file_path);
		}
	}

	// Copy model file to cache_model_path
	cache_model_path = tmp_model_path / model_path.filename();
	if (!boost::filesystem::exists(cache_model_path))
	{
		boost::filesystem::copy_file(model_path, cache_model_path);
	}

	// Add default.xml to tmp_model_path
	tmp_model_path /= tmp_model_name;
	tinyxml2::XMLDocument current_xml_doc;
	if (current_xml_doc.LoadFile(config_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", config_path.c_str());
		return;
	}
	tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	include_element->SetAttribute("file", boost::filesystem::relative(cache_model_path, tmp_model_path.parent_path()).c_str());
	current_element->LinkEndChild(include_element);
	current_xml_doc.SaveFile(tmp_model_path.c_str());

	// Add odom joints to cache_model_path if required
	if (use_odom_joints)
	{
		tinyxml2::XMLDocument cache_model_xml_doc;
		if (cache_model_xml_doc.LoadFile(cache_model_path.c_str()) != tinyxml2::XML_SUCCESS)
		{
			mju_warning_s("Failed to load file \"%s\"\n", cache_model_path.c_str());
			return;
		}
		for (tinyxml2::XMLElement *worldbody_element = cache_model_xml_doc.FirstChildElement()->FirstChildElement();
				 worldbody_element != nullptr;
				 worldbody_element = worldbody_element->NextSiblingElement())
		{
			if (strcmp(worldbody_element->Value(), "worldbody") == 0)
			{
				tinyxml2::XMLElement *robot_element = cache_model_xml_doc.NewElement("body");

				tinyxml2::XMLElement *odom_x_joint_element = cache_model_xml_doc.NewElement("joint");
				tinyxml2::XMLElement *odom_y_joint_element = cache_model_xml_doc.NewElement("joint");
				tinyxml2::XMLElement *odom_z_joint_element = cache_model_xml_doc.NewElement("joint");

				robot_element->LinkEndChild(odom_x_joint_element);
				robot_element->LinkEndChild(odom_y_joint_element);
				robot_element->LinkEndChild(odom_z_joint_element);

				robot_element->SetAttribute("name", model_path.stem().c_str());

				odom_x_joint_element->SetAttribute("name", MjSim::odom_joints["odom_x_joint"].first.c_str());
				odom_x_joint_element->SetAttribute("type", "slide");
				odom_x_joint_element->SetAttribute("axis", "1 0 0");

				odom_y_joint_element->SetAttribute("name", MjSim::odom_joints["odom_y_joint"].first.c_str());
				odom_y_joint_element->SetAttribute("type", "slide");
				odom_y_joint_element->SetAttribute("axis", "0 1 0");

				odom_z_joint_element->SetAttribute("name", MjSim::odom_joints["odom_z_joint"].first.c_str());
				odom_z_joint_element->SetAttribute("type", "hinge");
				odom_z_joint_element->SetAttribute("axis", "0 0 1");

				while (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement())
				{
					robot_element->LinkEndChild(body_element);
				}
				worldbody_element->LinkEndChild(robot_element);

				break;
			}
		}

		cache_model_xml_doc.SaveFile(cache_model_path.c_str());
	}
}

static void add_new_state(mjModel *m_new, mjData *d_new)
{
	d_new->time = d->time;

	ROS_INFO("%d", m->nq);
	int body_id = 0;
	while (true)
	{
		body_id++;
		const char *name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
		if (name == nullptr)
		{
			break;
		}		
		ROS_INFO("%s", name);

		int body_id_new = mj_name2id(m, mjtObj::mjOBJ_BODY, name);
		if (body_id_new == -1)
		{
			ROS_WARN("New body [%s] not found", name);
			continue;
		}

		// Copy body states
		for (int body_nr = 0; body_nr < 6; body_nr++)
		{
			d_new->xfrc_applied[body_id_new + body_nr] = d->xfrc_applied[body_id + body_nr];
		}

		// Copy joint states
		int jnt_num = m->body_jntnum[body_id];
		int jnt_num_new = m_new->body_jntnum[body_id_new];
		if (jnt_num != jnt_num_new)
		{
			ROS_WARN("Old [%s] has %d joints != new [%s] has %d joints", name, jnt_num, name, jnt_num_new);
			continue;
		}
		int jnt_id = m->body_jntadr[body_id];
		int jnt_id_new = m_new->body_jntadr[body_id_new];
		for (int jnt_nr = 0; jnt_nr < jnt_num; jnt_nr++)
		{
			d_new->qpos[jnt_id_new + jnt_nr] = d->qpos[jnt_id + jnt_nr];
		}

		// Copy dof states
		int dof_num = m->body_dofnum[body_id];
		int dof_num_new = m_new->body_dofnum[body_id_new];
		if (dof_num != dof_num_new)
		{
			ROS_WARN("Old [%s] has %d dofs != new [%s] has %d dofs", name, dof_num, name, dof_num_new);
			continue;
		}
		int dof_id = m->body_dofadr[body_id];
		int dof_id_new = m_new->body_dofadr[body_id_new];
		for (int dof_nr = 0; dof_nr < dof_num; dof_nr++)
		{
			d_new->qvel[dof_id_new + dof_nr] = d->qvel[dof_id + dof_nr];
			d_new->qacc_warmstart[dof_id_new + dof_nr] = d->qacc_warmstart[dof_id + dof_nr];
			d_new->qfrc_applied[dof_id_new + dof_nr] = d->qfrc_applied[dof_id + dof_nr];
			d_new->qacc[dof_id_new + dof_nr] = d->qacc[dof_id + dof_nr];
		}
	}

	// Copy activation states
	if (m->na != 0 || m_new->na != 0)
	{
		ROS_WARN("Old model has %d activation states, new model has %d activation states, not supported, will be ignored...", m->na, m_new->na);
	}

	// Copy mocap bodies
	if (m->nmocap != 0 || m_new->nmocap != 0)
	{
		ROS_WARN("Old model has %d mocap bodies, new model has %d mocap bodies, not supported, will be ignored...", m->nmocap, m_new->nmocap);
	}

	// Copy sensor data
	if (m->nsensordata != m_new->nsensordata)
	{
		ROS_WARN("Old model has %d sensors, new model has %d sensors, not supported, will be ignored...", m->nmocap, m_new->nmocap);
	}
	else
	{
		mju_copy(d_new->sensordata, d->sensordata, m->nsensordata);
	}

	d = d_new;
	m = m_new;
}

void init_malloc()
{
	MjSim::tau = (mjtNum *)mju_malloc(m->nv * sizeof(mjtNum *));
	mju_zero(MjSim::tau, m->nv);
}

void MjSim::init()
{
	set_param();
	init_tmp();
	load_model();
	init_malloc();
	set_joint_names();
	sim_start = d->time;
}

bool MjSim::add_data()
{
	// Save current.xml
	char error[1000] = "Could not save binary model";
	mj_saveLastXML(tmp_model_path.c_str(), m, error, 1000);

	// Modify current.xml
	tinyxml2::XMLDocument current_xml_doc;
	if (current_xml_doc.LoadFile(tmp_model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", tmp_model_path.c_str());
		return false;
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
	if (current_xml_doc.SaveFile(tmp_model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		return false;
	}

	// Load current.xml
	mjModel *m_new = mj_loadXML(tmp_model_path.c_str(), 0, error, 1000);
	if (!m_new)
	{
		mju_warning_s("Load model error: %s", error);
		return false;
	}

	mtx.lock();
	// make data
	mjData *d_new = mj_makeData(m_new);

	add_new_state(m_new, d_new);

	init_malloc();
	mtx.unlock();

	return true;
}

void MjSim::controller()
{
	mju_copy(d->qfrc_applied, tau, m->nv);
}

void MjSim::set_mimic_joints()
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

void MjSim::set_odom_joints()
{
	for (const std::pair<std::string, std::pair<std::string, mjtNum>> &odom_joint : odom_joints)
	{
		const std::string joint_name = odom_joint.second.first;
		const int joint_idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());

		d->qvel[joint_idx] = odom_joint.second.second;
	}
}