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

double MjSim::max_time_step;

std::map<std::string, std::vector<std::string>> MjSim::joint_names;

std::vector<std::string> MjSim::joint_ignores;

std::map<std::string, mjtNum> MjSim::odom_vels;

std::vector<std::string> MjSim::link_names;

mjtNum *MjSim::u = NULL;

mjtNum *MjSim::tau = NULL;

mjtNum MjSim::sim_start;

std::map<std::string, std::map<std::string, bool>> MjSim::add_odom_joints;

std::vector<std::string> MjSim::robots;

std::map<size_t, std::string> MjSim::sensors;

MjSim::~MjSim()
{
	mju_free(tau);
	boost::filesystem::remove_all(tmp_model_path.parent_path());
}

/**
 * @brief Set tmp_model_name, world_path and odom_joints
 */
static void set_param()
{
	std::string model_path_string;
	if (ros::param::get("~robot", model_path_string))
	{
		boost::filesystem::path model_path_path = model_path_string;
		if (model_path_path.extension() == ".urdf")
		{
			model_path = model_path.parent_path() / (model_path_path.stem().string() + ".xml");
			tmp_model_name = "current_" + model_path.filename().string();
		}
		else
		{
			model_path = model_path_string;
			tmp_model_name = "current_" + model_path.filename().string();
		}
	}

	if (ros::param::get("~max_time_step", MjSim::max_time_step))
	{
		ROS_INFO("Set max time step = %f", MjSim::max_time_step);
	}
	else
	{
		MjSim::max_time_step = 0.005;
	}

	std::string world_path_string;
	if (ros::param::get("~world", world_path_string))
	{
		ROS_INFO("Set world from %s", world_path_string.c_str());
		world_path = world_path_string;
	}

	if (!ros::param::get("~robots", MjSim::robots))
	{
		MjSim::robots.push_back(model_path.stem().string());
	}

	bool add_odom_joints_bool;
	if (ros::param::get("~add_odom_joints", add_odom_joints_bool))
	{
		for (const std::string &robot : MjSim::robots)
		{
			MjSim::add_odom_joints[robot]["lin_odom_x_joint"] = add_odom_joints_bool;
			MjSim::add_odom_joints[robot]["lin_odom_y_joint"] = add_odom_joints_bool;
			MjSim::add_odom_joints[robot]["lin_odom_z_joint"] = false;
			MjSim::add_odom_joints[robot]["ang_odom_x_joint"] = false;
			MjSim::add_odom_joints[robot]["ang_odom_y_joint"] = false;
			MjSim::add_odom_joints[robot]["ang_odom_z_joint"] = add_odom_joints_bool;
		}
	}
	else
	{
		bool odom_joint_bool;
		for (const std::string &odom_joint_name : {"lin_odom_x_joint", "lin_odom_y_joint", "lin_odom_z_joint", "ang_odom_x_joint", "ang_odom_y_joint", "ang_odom_z_joint"})
		{
			if (ros::param::get("~add_odom_joints/" + odom_joint_name, odom_joint_bool))
			{
				for (const std::string &robot : MjSim::robots)
				{
					MjSim::add_odom_joints[robot][odom_joint_name] = odom_joint_bool;
				}
			}
			else
			{
				for (const std::string &robot : MjSim::robots)
				{
					MjSim::add_odom_joints[robot][odom_joint_name] = false;
				}
			}
		}

		for (const std::string &robot : MjSim::robots)
		{
			if (ros::param::get("~add_odom_joints/" + robot, odom_joint_bool))
			{
				MjSim::add_odom_joints[robot]["lin_odom_x_joint"] = odom_joint_bool;
				MjSim::add_odom_joints[robot]["lin_odom_y_joint"] = odom_joint_bool;
				MjSim::add_odom_joints[robot]["lin_odom_z_joint"] = false;
				MjSim::add_odom_joints[robot]["ang_odom_x_joint"] = false;
				MjSim::add_odom_joints[robot]["ang_odom_y_joint"] = false;
				MjSim::add_odom_joints[robot]["ang_odom_z_joint"] = odom_joint_bool;
			}
			else
			{
				for (const std::string &odom_joint_name : {"lin_odom_x_joint", "lin_odom_y_joint", "lin_odom_z_joint", "ang_odom_x_joint", "ang_odom_y_joint", "ang_odom_z_joint"})
				{
					if (ros::param::get("~add_odom_joints/" + robot + "/" + odom_joint_name, odom_joint_bool))
					{
						MjSim::add_odom_joints[robot][odom_joint_name] = odom_joint_bool;
					}
				}
			}
		}
	}
}

/**
 * @brief Get all joint names of this body element and save it in MjSim::joint_names
 *
 * @param body_element
 */
static void get_joint_names(tinyxml2::XMLElement *body_element)
{
	static std::string robot_name;
	if (std::find(MjSim::robots.begin(), MjSim::robots.end(), body_element->Attribute("name")) != MjSim::robots.end())
	{
		robot_name = body_element->Attribute("name");
	}

	if (robot_name.empty())
	{
		return;
	}

	for (tinyxml2::XMLElement *joint_element = body_element->FirstChildElement();
			 joint_element != nullptr;
			 joint_element = joint_element->NextSiblingElement())
	{
		if (strcmp(joint_element->Value(), "joint") == 0)
		{
			std::string joint_name = joint_element->Attribute("name");
			if (MjSim::odom_vels.count(joint_name))
			{
				continue;
			}
			MjSim::joint_names[robot_name].push_back(joint_name);
		}
	}
}

/**
 * @brief Iterate all child body element of this body element
 *
 * @param parent_body_element
 */
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

/**
 * @brief Set all joint names to MjSim::joint_names and MjSim::odom_vels
 *
 */
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

/**
 * @brief Create tmp_model_mesh_path and copy model meshes there,
 * add world to tmp_model_path,
 * copy model.xml to cache_model_path and add odom joints if required,
 * include cache_model_path into tmp_model_path
 */
static void init_tmp()
{
	// Remove directory tmp_model_path if exist and tmp_model_path doesn't contain model_path
	if (boost::filesystem::exists(tmp_model_path) && tmp_model_path.parent_path().compare(model_path.parent_path()) != 0)
	{
		boost::filesystem::remove_all(tmp_model_path);
	}

	// Create directory tmp_model_path if not exist
	if (!boost::filesystem::exists(tmp_model_path))
	{
		boost::filesystem::create_directory(tmp_model_path);
	}

	// Create directory tmp_world_mesh_path if not exist
	std::string world_path_tail = world_path.stem().string() + "/meshes/";
	boost::filesystem::path tmp_world_mesh_path = tmp_world_path / world_path_tail;
	if (!boost::filesystem::exists(tmp_world_mesh_path))
	{
		boost::filesystem::create_directories(tmp_world_mesh_path);
	}

	// Copy model meshes to tmp_world_mesh_path (to save .dae files)
	if (boost::filesystem::exists(world_path.parent_path() / world_path_tail))
	{
		for (const boost::filesystem::directory_entry &file : boost::filesystem::directory_iterator(world_path.parent_path() / world_path_tail))
		{
			boost::filesystem::path des_file_path = tmp_world_mesh_path / file.path().filename();
			if (!boost::filesystem::exists(des_file_path))
			{
				boost::filesystem::copy_file(file.path(), des_file_path);
			}
		}
	}

	// Copy model file to cache_model_path
	cache_model_path = tmp_model_path / model_path.filename();
	if (!boost::filesystem::exists(cache_model_path))
	{
		boost::filesystem::copy_file(model_path, cache_model_path);
	}

	// Add world to tmp_model_path
	tmp_model_path /= tmp_model_name;
	tinyxml2::XMLDocument current_xml_doc;
	if (current_xml_doc.LoadFile(world_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", world_path.c_str());
		return;
	}
	boost::filesystem::path meshdir_abs_path = world_path.parent_path();
	for (tinyxml2::XMLElement *element = current_xml_doc.FirstChildElement()->FirstChildElement();
			 element != nullptr;
			 element = element->NextSiblingElement())
	{
		if (strcmp(element->Value(), "compiler") == 0)
		{
			if (element->Attribute("meshdir") != nullptr)
			{
				boost::filesystem::path meshdir_path = element->Attribute("meshdir");
				if (meshdir_path.is_relative())
				{
					meshdir_abs_path /= meshdir_path;
				}
				else
				{
					meshdir_abs_path = meshdir_path;
				}

				element->DeleteAttribute("meshdir");
				break;
			}
		}
	}

	for (tinyxml2::XMLElement *element = current_xml_doc.FirstChildElement()->FirstChildElement();
			 element != nullptr;
			 element = element->NextSiblingElement())
	{
		if (strcmp(element->Value(), "asset") == 0)
		{
			for (tinyxml2::XMLElement *asset_element = element->FirstChildElement();
					 asset_element != nullptr;
					 asset_element = asset_element->NextSiblingElement())
			{
				if (asset_element->Attribute("file") != nullptr)
				{
					boost::filesystem::path file_path = asset_element->Attribute("file");
					if (file_path.is_relative())
					{
						asset_element->SetAttribute("file", (meshdir_abs_path / file_path).c_str());
					}
				}
			}
		}
	}

	tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	include_element->SetAttribute("file", boost::filesystem::relative(cache_model_path, tmp_model_path.parent_path()).c_str());
	current_element->LinkEndChild(include_element);
	current_xml_doc.SaveFile(tmp_model_path.c_str());

	tinyxml2::XMLDocument cache_model_xml_doc;
	if (cache_model_xml_doc.LoadFile(cache_model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", cache_model_path.c_str());
		return;
	}

	meshdir_abs_path = model_path.parent_path();
	for (tinyxml2::XMLElement *element = cache_model_xml_doc.FirstChildElement()->FirstChildElement();
			 element != nullptr;
			 element = element->NextSiblingElement())
	{
		if (strcmp(element->Value(), "compiler") == 0)
		{
			if (element->Attribute("meshdir") != nullptr)
			{
				boost::filesystem::path meshdir_path = element->Attribute("meshdir");
				if (meshdir_path.is_relative())
				{
					meshdir_abs_path /= meshdir_path;
				}
				else
				{
					meshdir_abs_path = meshdir_path;
				}
				element->DeleteAttribute("meshdir");
			}
		}

		// Add odom joints to cache_model_path if required
		if (strcmp(element->Value(), "worldbody") == 0)
		{
			for (const std::string &robot : MjSim::robots)
			{
				if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] ||
						MjSim::add_odom_joints[robot]["lin_odom_y_joint"] ||
						MjSim::add_odom_joints[robot]["lin_odom_z_joint"] ||
						MjSim::add_odom_joints[robot]["ang_odom_x_joint"] ||
						MjSim::add_odom_joints[robot]["ang_odom_y_joint"] ||
						MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
				{
					ROS_INFO("Add odom joints for %s", robot.c_str());
					for (tinyxml2::XMLElement *robot_body = element->FirstChildElement();
							 robot_body != nullptr;
							 robot_body = robot_body->NextSiblingElement())
					{
						if (strcmp(robot_body->Attribute("name"), robot.c_str()) == 0)
						{
							if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] || (MjSim::add_odom_joints[robot]["lin_odom_y_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]) || (MjSim::add_odom_joints[robot]["lin_odom_y_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]))
							{
								tinyxml2::XMLElement *odom_lin_x_joint_element = cache_model_xml_doc.NewElement("joint");
								robot_body->InsertEndChild(odom_lin_x_joint_element);
								std::string lin_odom_x_joint_name = robot + "_lin_odom_x_joint";
								odom_lin_x_joint_element->SetAttribute("name", lin_odom_x_joint_name.c_str());
								odom_lin_x_joint_element->SetAttribute("type", "slide");
								odom_lin_x_joint_element->SetAttribute("axis", "1 0 0");
							}
							if (MjSim::add_odom_joints[robot]["lin_odom_y_joint"] || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]) || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]))
							{
								tinyxml2::XMLElement *odom_lin_y_joint_element = cache_model_xml_doc.NewElement("joint");
								robot_body->InsertEndChild(odom_lin_y_joint_element);
								std::string lin_odom_y_joint_name = robot + "_lin_odom_y_joint";
								odom_lin_y_joint_element->SetAttribute("name", lin_odom_y_joint_name.c_str());
								odom_lin_y_joint_element->SetAttribute("type", "slide");
								odom_lin_y_joint_element->SetAttribute("axis", "0 1 0");
							}
							if (MjSim::add_odom_joints[robot]["lin_odom_z_joint"] || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_y_joint"]) || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_y_joint"]))
							{
								tinyxml2::XMLElement *odom_lin_z_joint_element = cache_model_xml_doc.NewElement("joint");
								robot_body->InsertEndChild(odom_lin_z_joint_element);
								std::string lin_odom_z_joint_name = robot + "_lin_odom_z_joint";
								odom_lin_z_joint_element->SetAttribute("name", lin_odom_z_joint_name.c_str());
								odom_lin_z_joint_element->SetAttribute("type", "slide");
								odom_lin_z_joint_element->SetAttribute("axis", "0 0 1");
							}
							if (MjSim::add_odom_joints[robot]["ang_odom_x_joint"])
							{
								tinyxml2::XMLElement *odom_ang_x_joint_element = cache_model_xml_doc.NewElement("joint");
								robot_body->InsertEndChild(odom_ang_x_joint_element);
								std::string ang_odom_x_joint_name = robot + "_ang_odom_x_joint";
								odom_ang_x_joint_element->SetAttribute("name", ang_odom_x_joint_name.c_str());
								odom_ang_x_joint_element->SetAttribute("type", "hinge");
								odom_ang_x_joint_element->SetAttribute("axis", "1 0 0");
							}
							if (MjSim::add_odom_joints[robot]["ang_odom_y_joint"])
							{
								tinyxml2::XMLElement *odom_ang_y_joint_element = cache_model_xml_doc.NewElement("joint");
								robot_body->InsertEndChild(odom_ang_y_joint_element);
								std::string ang_odom_y_joint_name = robot + "_ang_odom_y_joint";
								odom_ang_y_joint_element->SetAttribute("name", ang_odom_y_joint_name.c_str());
								odom_ang_y_joint_element->SetAttribute("type", "hinge");
								odom_ang_y_joint_element->SetAttribute("axis", "0 1 0");
							}
							if (MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
							{
								tinyxml2::XMLElement *odom_ang_z_joint_element = cache_model_xml_doc.NewElement("joint");
								robot_body->InsertEndChild(odom_ang_z_joint_element);
								std::string ang_odom_z_joint_name = robot + "_ang_odom_z_joint";
								odom_ang_z_joint_element->SetAttribute("name", ang_odom_z_joint_name.c_str());
								odom_ang_z_joint_element->SetAttribute("type", "hinge");
								odom_ang_z_joint_element->SetAttribute("axis", "0 0 1");
							}
							break;
						}
					}
				}
			}
		}
	}

	for (tinyxml2::XMLElement *element = cache_model_xml_doc.FirstChildElement()->FirstChildElement();
			 element != nullptr;
			 element = element->NextSiblingElement())
	{
		if (strcmp(element->Value(), "asset") == 0)
		{
			for (tinyxml2::XMLElement *asset_element = element->FirstChildElement();
					 asset_element != nullptr;
					 asset_element = asset_element->NextSiblingElement())
			{
				if (asset_element->Attribute("file") != nullptr)
				{
					boost::filesystem::path file_path = asset_element->Attribute("file");
					if (file_path.is_relative())
					{
						asset_element->SetAttribute("file", (meshdir_abs_path / file_path).c_str());
					}
				}
			}
		}
	}

	cache_model_xml_doc.SaveFile(cache_model_path.c_str());
}

/**
 * @brief Add the old state to the new state
 *
 * @param m_new New mjModel*
 * @param d_new New mjData*
 */
static void add_old_state(mjModel *m_new, mjData *d_new)
{
	d_new->time = d->time;

	int body_id = 0;
	while (true)
	{
		body_id++;
		const char *name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
		if (name == nullptr)
		{
			break;
		}

		int body_id_new = mj_name2id(m_new, mjtObj::mjOBJ_BODY, name);
		if (body_id_new == -1)
		{
			continue;
		}

		// Copy body position
		d_new->xpos[body_id_new] = d->xpos[body_id];
		d_new->xpos[body_id_new + 1] = d->xpos[body_id + 1];
		d_new->xpos[body_id_new + 2] = d->xpos[body_id + 2];

		// Copy body rotation
		d_new->xquat[body_id_new] = d->xquat[body_id];
		d_new->xquat[body_id_new + 1] = d->xquat[body_id + 1];
		d_new->xquat[body_id_new + 2] = d->xquat[body_id + 2];
		d_new->xquat[body_id_new + 3] = d->xquat[body_id + 3];

		// Copy body states
		for (int body_nr = 0; body_nr < 6; body_nr++)
		{
			d_new->xfrc_applied[body_id_new + body_nr] = d->xfrc_applied[body_id + body_nr];
		}

		// Copy joint states
		int jnt_num = m->body_jntnum[body_id];
		int jnt_num_new = m_new->body_jntnum[body_id_new];
		if (jnt_num == jnt_num_new)
		{
			int qpos_adr = m->jnt_qposadr[m->body_jntadr[body_id]];
			int qpos_adr_new = m_new->jnt_qposadr[m_new->body_jntadr[body_id_new]];
			for (int jnt_nr = 0; jnt_nr < jnt_num; jnt_nr++)
			{
				d_new->qpos[qpos_adr_new + jnt_nr] = d->qpos[qpos_adr + jnt_nr];
			}
		}
		else
		{
			ROS_WARN("Old [%s] has %d joints != new [%s] has %d joints", name, jnt_num, name, jnt_num_new);
		}

		// Copy dof states
		int dof_num = m->body_dofnum[body_id];
		int dof_num_new = m_new->body_dofnum[body_id_new];
		if (dof_num == dof_num_new)
		{
			int dof_adr = m->jnt_dofadr[m->body_jntadr[body_id]];
			int dof_adr_new = m_new->jnt_dofadr[m_new->body_jntadr[body_id_new]];
			for (int dof_nr = 0; dof_nr < dof_num; dof_nr++)
			{
				d_new->qvel[dof_adr_new + dof_nr] = d->qvel[dof_adr + dof_nr];
				d_new->qacc_warmstart[dof_adr_new + dof_nr] = d->qacc_warmstart[dof_adr + dof_nr];
				d_new->qfrc_applied[dof_adr_new + dof_nr] = d->qfrc_applied[dof_adr + dof_nr];
				d_new->qacc[dof_adr_new + dof_nr] = d->qacc[dof_adr + dof_nr];
			}
		}
		else
		{
			ROS_WARN("Old [%s] has %d dofs != new [%s] has %d dofs", name, dof_num, name, dof_num_new);
		}
	}

	// Copy activation states
	if (m->na != 0 || m_new->na != 0)
	{
		ROS_WARN("Old model has %d activation states, new model has %d activation states, not supported, will be ignored...", m->na, m_new->na);
	}

	// Copy sensor data
	if (m->nsensordata != m_new->nsensordata)
	{
		ROS_WARN("Old model has %d sensors, new model has %d sensors, not supported, will be ignored...", m->nsensordata, m_new->nsensordata);
	}
	else
	{
		mju_copy(d_new->sensordata, d->sensordata, m->nsensordata);
	}

	d = d_new;
	m = m_new;
}

/**
 * @brief Reset malloc for MjSim::tau
 */
static void init_malloc()
{
	MjSim::tau = (mjtNum *)mju_malloc(m->nv * sizeof(mjtNum *));
	mju_zero(MjSim::tau, m->nv);
	MjSim::u = (mjtNum *)mju_malloc(m->nv * sizeof(mjtNum *));
	mju_zero(MjSim::u, m->nv);
}

static void modify_xml(const char *xml_path, const std::vector<std::string> &remove_body_names = {""})
{
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(xml_path) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", xml_path);
		return;
	}

	mtx.lock();
	tinyxml2::XMLElement *worldbody_element = doc.FirstChildElement()->FirstChildElement();
	std::vector<tinyxml2::XMLNode *> bodies_to_delete;
	for (tinyxml2::XMLElement *worldbody_element = doc.FirstChildElement()->FirstChildElement();
			 worldbody_element != nullptr;
			 worldbody_element = worldbody_element->NextSiblingElement())
	{
		if (strcmp(worldbody_element->Value(), "worldbody") == 0)
		{
			for (tinyxml2::XMLNode *body_node = worldbody_element->FirstChild();
					 body_node != nullptr;
					 body_node = body_node->NextSibling())
			{
				if (strcmp(body_node->Value(), "body") == 0)
				{
					const char *body_name = body_node->ToElement()->Attribute("name");
					if (body_name != nullptr && std::find(remove_body_names.begin(), remove_body_names.end(), body_name) != remove_body_names.end())
					{
						bodies_to_delete.push_back(body_node);
					}
					else if (body_name != nullptr &&
									 strcmp(body_name, model_path.stem().c_str()) != 0 &&
									 std::find(MjSim::link_names.begin(), MjSim::link_names.end(), body_name) == MjSim::link_names.end() &&
									 std::find(MjSim::robots.begin(), MjSim::robots.end(), body_name) == MjSim::robots.end())
					{
						const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, body_name);
						body_node->ToElement()->SetAttribute("pos",
																								 (std::to_string(d->xpos[3 * body_id]) + " " +
																									std::to_string(d->xpos[3 * body_id + 1]) + " " +
																									std::to_string(d->xpos[3 * body_id + 2]))
																										 .c_str());
						body_node->ToElement()->SetAttribute("quat",
																								 (std::to_string(d->xquat[4 * body_id]) + " " +
																									std::to_string(d->xquat[4 * body_id + 1]) + " " +
																									std::to_string(d->xquat[4 * body_id + 2]) + " " +
																									std::to_string(d->xquat[4 * body_id + 3]))
																										 .c_str());
					}
				}
			}
		}
	}
	for (tinyxml2::XMLNode *&body_to_delete : bodies_to_delete)
	{
		worldbody_element->DeleteChild(body_to_delete);
	}

	mtx.unlock();

	doc.SaveFile(xml_path);
}

/**
 * @brief Load the tmp_model_path
 *
 * @param reset Reset the simulation or not
 * @return true if succeed
 */
bool load_tmp_model(bool reset)
{
	char error[1000] = "Could not load binary model";
	if (reset)
	{
		// load and compile model
		m = mj_loadXML(tmp_model_path.c_str(), 0, error, 1000);
		if (!m)
		{
			mju_warning_s("Could not load model file '%s'", tmp_model_path.c_str());
			return false;
		}

		// make data
		d = mj_makeData(m);
		init_malloc();
		return true;
	}
	else
	{
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
		add_old_state(m_new, d_new);
		init_malloc();
		mtx.unlock();

		return true;
	}
}

void MjSim::init()
{
	set_param();
	init_tmp();
	load_tmp_model(true);
	set_joint_names();
	init_sensors();
	sim_start = d->time;
}

void MjSim::init_sensors()
{
	for (size_t sensor_id = 0; sensor_id < m->nsensor; sensor_id++)
	{
		std::string sensor_name;

		if (m->sensor_type[sensor_id] == mjtSensor::mjSENS_FORCE)
		{
			if (mj_id2name(m, mjtObj::mjOBJ_SENSOR, sensor_id) == nullptr)
			{
				mju_warning_i("Sensor with id %d doesn't have a name, create one...", sensor_id);
				sensor_name = "force_sensor_";
				sensor_name += mj_id2name(m, mjtObj::mjOBJ_SITE, m->sensor_objid[sensor_id]);
				mju_warning_s("Created sensor %s", sensor_name.c_str());
			}
			else
			{
				sensor_name = mj_id2name(m, mjtObj::mjOBJ_SENSOR, sensor_id);
			}
		}
		else if (m->sensor_type[sensor_id] == mjtSensor::mjSENS_TORQUE)
		{
			if (mj_id2name(m, mjtObj::mjOBJ_SENSOR, sensor_id) == nullptr)
			{
				mju_warning_i("Sensor with id %d doesn't have a name, create one...", sensor_id);
				sensor_name = "torque_sensor_";
				sensor_name += mj_id2name(m, mjtObj::mjOBJ_SITE, m->sensor_objid[sensor_id]);
				mju_warning_s("Created sensor %s", sensor_name.c_str());
			}
			else
			{
				sensor_name = mj_id2name(m, mjtObj::mjOBJ_SENSOR, sensor_id);
			}
		}
		else
		{
			mju_warning_i("Sensor with type_id %d not implemented, ignore...", m->sensor_type[sensor_id]);
			continue;
		}

		sensors[sensor_id] = sensor_name;
	}
}

bool MjSim::add_data()
{
	// Save current.xml
	char error[1000] = "Could not save binary model";
	mj_saveLastXML(tmp_model_path.c_str(), m, error, 1000);

	modify_xml(tmp_model_path.c_str());

	// Add add.xml to current.xml
	tinyxml2::XMLDocument current_xml_doc;
	if (current_xml_doc.LoadFile(tmp_model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to load file \"%s\"\n", tmp_model_path.c_str());
		return false;
	}

	tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	include_element->SetAttribute("file", "add.xml");
	current_element->LinkEndChild(include_element);
	if (current_xml_doc.SaveFile(tmp_model_path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		mju_warning_s("Failed to save file \"%s\"\n", tmp_model_path.c_str());
		return false;
	}

	return load_tmp_model(false);
}

bool MjSim::remove_body(const std::vector<std::string> &body_names)
{
	// Save current.xml
	char error[1000] = "Could not save binary model";
	mj_saveLastXML(tmp_model_path.c_str(), m, error, 1000);

	// Modify current.xml
	modify_xml(tmp_model_path.c_str(), body_names);

	return load_tmp_model(false);
}

void MjSim::controller()
{
	mj_mulM(m, d, tau, u);
	for (const std::string &robot : MjSim::robots)
	{
		for (const std::string &joint_name : MjSim::joint_names[robot])
		{
			if (std::find(MjSim::joint_ignores.begin(), MjSim::joint_ignores.end(), joint_name) == MjSim::joint_ignores.end())
			{
				const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
				const int dof_id = m->jnt_dofadr[joint_id];
				tau[dof_id] += d->qfrc_bias[dof_id];
			}
		}
	}
	mju_copy(d->qfrc_applied, tau, m->nv);
	mju_zero(u, m->nv);
}

void MjSim::set_odom_vels()
{
	for (const std::string &robot : MjSim::robots)
	{
		const int odom_x_joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, (robot + "_ang_odom_x_joint").c_str());
		const int odom_y_joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, (robot + "_ang_odom_y_joint").c_str());
		const int odom_z_joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, (robot + "_ang_odom_z_joint").c_str());
		const int odom_x_qpos_id = m->jnt_qposadr[odom_x_joint_id];
		const int odom_y_qpos_id = m->jnt_qposadr[odom_y_joint_id];
		const int odom_z_qpos_id = m->jnt_qposadr[odom_z_joint_id];
		mjtNum odom_x_joint_pos = odom_x_joint_id != -1 ? d->qpos[odom_x_qpos_id] : 0.f;
		mjtNum odom_y_joint_pos = odom_y_joint_id != -1 ? d->qpos[odom_y_qpos_id] : 0.f;
		mjtNum odom_z_joint_pos = odom_z_joint_id != -1 ? d->qpos[odom_z_qpos_id] : 0.f;

		if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"])
		{
			const std::string joint_name = robot + "_lin_odom_x_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id == -1)
			{
				ROS_ERROR("Joint %s not found", joint_name.c_str());
			}
			else
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_cos(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) - mju_cos(odom_x_joint_pos) * mju_sin(odom_z_joint_pos)) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + mju_sin(odom_x_joint_pos) * mju_sin(odom_z_joint_pos));
			}
		}
		if (MjSim::add_odom_joints[robot]["lin_odom_y_joint"])
		{
			const std::string joint_name = robot + "_lin_odom_y_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id == -1)
			{
				ROS_ERROR("Joint %s not found", joint_name.c_str());
			}
			else
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_cos(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + mju_cos(odom_x_joint_pos) * mju_cos(odom_z_joint_pos)) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) - mju_sin(odom_x_joint_pos) * mju_cos(odom_z_joint_pos));
			}
		}
		if (MjSim::add_odom_joints[robot]["lin_odom_z_joint"])
		{
			const std::string joint_name = robot + "_lin_odom_z_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id == -1)
			{
				ROS_ERROR("Joint %s not found", joint_name.c_str());
			}
			else
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = -MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_sin(odom_y_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * mju_sin(odom_x_joint_pos) * mju_cos(odom_y_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * mju_cos(odom_x_joint_pos) * mju_cos(odom_y_joint_pos);
			}
		}
		if (MjSim::add_odom_joints[robot]["ang_odom_x_joint"])
		{
			const std::string joint_name = robot + "_ang_odom_x_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id == -1)
			{
				ROS_ERROR("Joint %s not found", joint_name.c_str());
			}
			else
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_ang_odom_x_joint"];
			}
		}
		if (MjSim::add_odom_joints[robot]["ang_odom_y_joint"])
		{
			const std::string joint_name = robot + "_ang_odom_y_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id == -1)
			{
				ROS_ERROR("Joint %s not found", joint_name.c_str());
			}
			else
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_ang_odom_y_joint"];
			}
		}
		if (MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
		{
			const std::string joint_name = robot + "_ang_odom_z_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id == -1)
			{
				ROS_ERROR("Joint %s not found", joint_name.c_str());
			}
			else
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_ang_odom_z_joint"];
			}
		}
	}
}