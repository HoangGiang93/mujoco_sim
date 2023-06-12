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

#include "mj_util.h"

#include <ros/package.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

double MjSim::max_time_step;

std::map<std::string, std::vector<std::string>> MjSim::joint_names;

std::map<std::string, mjtNum> MjSim::odom_vels;

std::set<std::string> MjSim::robot_link_names;

mjtNum *MjSim::dq = NULL;

mjtNum *MjSim::ddq = NULL;

mjtNum *MjSim::tau = NULL;

mjtNum MjSim::sim_start;

std::map<std::string, std::map<std::string, bool>> MjSim::add_odom_joints;

std::set<std::string> MjSim::controlled_joints;

std::set<std::string> MjSim::robot_names;

std::map<size_t, std::string> MjSim::sensors;

std::map<std::string, std::vector<float>> MjSim::pose_inits;

bool MjSim::reload_mesh = true;

std::set<std::string> MjSim::spawned_object_body_names;

// Fix bug from m->geom_pos and m->geom_quat
std::map<int, std::vector<mjtNum>> MjSim::geom_pose;

bool MjSim::disable_gravity = true;

MjSim::~MjSim()
{
	mju_free(tau);
	boost::filesystem::remove_all(tmp_model_path.parent_path());
}

static void set_joint_names()
{
	tinyxml2::XMLDocument cache_model_xml_doc;
	if (!load_XML(cache_model_xml_doc, model_path.c_str()))
	{
		ROS_WARN("Failed to load file \"%s\"\n", model_path.c_str());
		return;
	}
	if (cache_model_xml_doc.FirstChildElement()->FirstChildElement("worldbody") == nullptr)
	{
		ROS_WARN("%s doesn't have <worldbody>", model_path.c_str());
		return;
	}

	for (tinyxml2::XMLElement *body_element = cache_model_xml_doc.FirstChildElement()->FirstChildElement("worldbody")->FirstChildElement("body");
		 body_element != nullptr;
		 body_element = body_element->NextSiblingElement("body"))
	{
		if (body_element->Attribute("name") != nullptr)
		{
			const std::string robot_name = body_element->Attribute("name");
			if (MjSim::robot_names.find(robot_name) == MjSim::robot_names.end())
			{
				continue;
			}

			ROS_INFO("Getting joints of model %s...", robot_name.c_str());
			std::function<void(tinyxml2::XMLElement *, const std::string &)> func = [](tinyxml2::XMLElement *body_element, const std::string &robot_name)
			{
				const std::set<std::string> odom_joint_names = {
					robot_name + "_lin_odom_x_joint",
					robot_name + "_lin_odom_y_joint",
					robot_name + "_lin_odom_z_joint",
					robot_name + "_ang_odom_x_joint",
					robot_name + "_ang_odom_y_joint",
					robot_name + "_ang_odom_z_joint",
				};
				for (tinyxml2::XMLElement *joint_element = body_element->FirstChildElement("joint");
					 joint_element != nullptr;
					 joint_element = joint_element->NextSiblingElement("joint"))
				{
					if (joint_element->Attribute("name") != nullptr)
					{
						const std::string joint_name = joint_element->Attribute("name");
						if (odom_joint_names.find(joint_name) == odom_joint_names.end())
						{
							MjSim::joint_names[robot_name].push_back(joint_name);
						}
					}
				}
			};

			do_each_child_element(body_element, robot_name, func);
		}
	}
	if (MjSim::joint_names.size() == 0)
	{
		ROS_WARN("Model %s has 0 joints", model_path.c_str());
	}
	for (const std::string &robot : MjSim::robot_names)
	{
		ROS_INFO("Initialize model %s with %ld joints successfully", robot.c_str(), MjSim::joint_names[robot].size());
	}
}

static void save_mesh_paths(tinyxml2::XMLDocument &doc, const boost::filesystem::path &meshdir_abs_path)
{
	for (tinyxml2::XMLElement *asset_element = doc.FirstChildElement()->FirstChildElement("asset");
		 asset_element != nullptr;
		 asset_element = asset_element->NextSiblingElement("asset"))
	{
		for (tinyxml2::XMLElement *texture_element = asset_element->FirstChildElement("texture");
			 texture_element != nullptr;
			 texture_element = texture_element->NextSiblingElement("texture"))
		{
			if (texture_element->Attribute("file") != nullptr)
			{
				const boost::filesystem::path file_path = texture_element->Attribute("file");
				if (file_path.is_relative())
				{
					texture_element->SetAttribute("file", (meshdir_abs_path / file_path).c_str());
				}
			}
		}
		for (tinyxml2::XMLElement *mesh_element = asset_element->FirstChildElement("mesh");
			 mesh_element != nullptr;
			 mesh_element = mesh_element->NextSiblingElement("mesh"))
		{
			if (mesh_element->Attribute("file") != nullptr)
			{
				boost::filesystem::path file_path = mesh_element->Attribute("file");
				if (file_path.is_relative())
				{
					mesh_element->SetAttribute("file", (meshdir_abs_path / file_path).c_str());
				}
				std::string scale_str = "1 1 1";
				if (mesh_element->Attribute("scale") != nullptr)
				{
					scale_str = mesh_element->Attribute("scale");
				}
				std::istringstream iss(scale_str);
				std::vector<mjtNum> scale = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};

				mesh_paths[mesh_element->Attribute("name")] = {mesh_element->Attribute("file"), scale};
			}
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
	ROS_INFO("Copying model in %s...", tmp_model_path.parent_path().c_str());
	// Remove directory tmp_model_path if exist and tmp_model_path doesn't contain model_path
	// if (boost::filesystem::exists(tmp_model_path) && tmp_model_path.parent_path().compare(model_path.parent_path()) != 0)
	// {
	// 	boost::filesystem::remove_all(tmp_model_path);
	// }

	// Create directory tmp_model_path if not exist
	if (!boost::filesystem::exists(tmp_model_path))
	{
		boost::filesystem::create_directory(tmp_model_path);
	}

	// Create directory tmp_world_mesh_path if not exist
	std::string world_path_tail = world_path.stem().string() + "/stl/";
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
	if (!load_XML(current_xml_doc, world_path.c_str()))
	{
		ROS_WARN("Failed to load file \"%s\"\n", world_path.c_str());
		return;
	}
	boost::filesystem::path meshdir_abs_path = world_path.parent_path();
	for (tinyxml2::XMLElement *compiler_element = current_xml_doc.FirstChildElement()->FirstChildElement("compiler");
		 compiler_element != nullptr;
		 compiler_element = compiler_element->NextSiblingElement("compiler"))
	{
		if (compiler_element->Attribute("meshdir") != nullptr)
		{
			boost::filesystem::path meshdir_path = compiler_element->Attribute("meshdir");
			if (meshdir_path.is_relative())
			{
				meshdir_abs_path /= meshdir_path;
			}
			else
			{
				meshdir_abs_path = meshdir_path;
			}

			compiler_element->DeleteAttribute("meshdir");
			break;
		}
	}

	save_mesh_paths(current_xml_doc, meshdir_abs_path);

	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	current_xml_doc.FirstChildElement()->LinkEndChild(include_element);

	include_element->SetAttribute("file", boost::filesystem::relative(cache_model_path, tmp_model_path.parent_path()).c_str());

	save_XML(current_xml_doc, tmp_model_path.c_str());

	tinyxml2::XMLDocument cache_model_xml_doc;
	if (!load_XML(cache_model_xml_doc, cache_model_path.c_str()))
	{
		ROS_WARN("Failed to load file \"%s\"\n", cache_model_path.c_str());
		return;
	}

	meshdir_abs_path = model_path.parent_path();
	for (tinyxml2::XMLElement *compiler_element = cache_model_xml_doc.FirstChildElement()->FirstChildElement("compiler");
		 compiler_element != nullptr;
		 compiler_element = compiler_element->NextSiblingElement("compiler"))
	{
		if (compiler_element->Attribute("meshdir") != nullptr)
		{
			boost::filesystem::path meshdir_path = compiler_element->Attribute("meshdir");
			if (meshdir_path.is_relative())
			{
				meshdir_abs_path /= meshdir_path;
			}
			else
			{
				meshdir_abs_path = meshdir_path;
			}

			compiler_element->DeleteAttribute("meshdir");
		}
	}

	save_mesh_paths(cache_model_xml_doc, meshdir_abs_path);

	for (tinyxml2::XMLElement *worldbody_element = cache_model_xml_doc.FirstChildElement()->FirstChildElement("worldbody");
		 worldbody_element != nullptr;
		 worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
	{
		do_each_child_element(worldbody_element, [](tinyxml2::XMLElement *body_element)
							  { 
														if (MjSim::disable_gravity)
														{
															body_element->SetAttribute("gravcomp", "1");
														}
														else
														{
															body_element->SetAttribute("gravcomp", "0");
														} });

		for (tinyxml2::XMLElement *robot_body = worldbody_element->FirstChildElement("body");
			 robot_body != nullptr;
			 robot_body = robot_body->NextSiblingElement("body"))
		{
			const char *robot = robot_body->Attribute("name");
			if (robot != nullptr && MjSim::robot_names.find(robot) != MjSim::robot_names.end() && MjSim::pose_inits.find(robot) != MjSim::pose_inits.end() && MjSim::pose_inits[robot].size() == 6)
			{
				robot_body->SetAttribute("pos", (std::to_string(MjSim::pose_inits[robot][0]) + " " +
												 std::to_string(MjSim::pose_inits[robot][1]) + " " +
												 std::to_string(MjSim::pose_inits[robot][2]))
													.c_str());
				tf2::Quaternion quat;
				quat.setRPY(MjSim::pose_inits[robot][3], MjSim::pose_inits[robot][4], MjSim::pose_inits[robot][5]);
				robot_body->SetAttribute("quat", (std::to_string(quat.getW()) + " " +
												  std::to_string(quat.getX()) + " " +
												  std::to_string(quat.getY()) + " " +
												  std::to_string(quat.getZ()))
													 .c_str());
				break;
			}
		}

		// Add odom joints to cache_model_path if required
		for (const std::string &robot : MjSim::robot_names)
		{
			if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] ||
				MjSim::add_odom_joints[robot]["lin_odom_y_joint"] ||
				MjSim::add_odom_joints[robot]["lin_odom_z_joint"] ||
				MjSim::add_odom_joints[robot]["ang_odom_x_joint"] ||
				MjSim::add_odom_joints[robot]["ang_odom_y_joint"] ||
				MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
			{
				ROS_INFO("Adding odom joints for model %s...", robot.c_str());
				int odom_joint_count = 0;
				for (tinyxml2::XMLElement *robot_body = worldbody_element->FirstChildElement("body");
					 robot_body != nullptr;
					 robot_body = robot_body->NextSiblingElement("body"))
				{
					if (robot_body->Attribute("name", robot.c_str()))
					{
						if (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] || (MjSim::add_odom_joints[robot]["lin_odom_y_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]) || (MjSim::add_odom_joints[robot]["lin_odom_y_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]))
						{
							tinyxml2::XMLElement *odom_lin_x_joint_element = cache_model_xml_doc.NewElement("joint");
							robot_body->InsertEndChild(odom_lin_x_joint_element);
							std::string lin_odom_x_joint_name = robot + "_lin_odom_x_joint";
							odom_lin_x_joint_element->SetAttribute("name", lin_odom_x_joint_name.c_str());
							odom_lin_x_joint_element->SetAttribute("type", "slide");
							odom_lin_x_joint_element->SetAttribute("axis", "1 0 0");
							odom_joint_count++;
						}
						if (MjSim::add_odom_joints[robot]["lin_odom_y_joint"] || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]) || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_z_joint"]))
						{
							tinyxml2::XMLElement *odom_lin_y_joint_element = cache_model_xml_doc.NewElement("joint");
							robot_body->InsertEndChild(odom_lin_y_joint_element);
							std::string lin_odom_y_joint_name = robot + "_lin_odom_y_joint";
							odom_lin_y_joint_element->SetAttribute("name", lin_odom_y_joint_name.c_str());
							odom_lin_y_joint_element->SetAttribute("type", "slide");
							odom_lin_y_joint_element->SetAttribute("axis", "0 1 0");
							odom_joint_count++;
						}
						if (MjSim::add_odom_joints[robot]["lin_odom_z_joint"] || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_y_joint"]) || (MjSim::add_odom_joints[robot]["lin_odom_x_joint"] && MjSim::add_odom_joints[robot]["ang_odom_y_joint"]))
						{
							tinyxml2::XMLElement *odom_lin_z_joint_element = cache_model_xml_doc.NewElement("joint");
							robot_body->InsertEndChild(odom_lin_z_joint_element);
							std::string lin_odom_z_joint_name = robot + "_lin_odom_z_joint";
							odom_lin_z_joint_element->SetAttribute("name", lin_odom_z_joint_name.c_str());
							odom_lin_z_joint_element->SetAttribute("type", "slide");
							odom_lin_z_joint_element->SetAttribute("axis", "0 0 1");
							odom_joint_count++;
						}
						if (MjSim::add_odom_joints[robot]["ang_odom_x_joint"])
						{
							tinyxml2::XMLElement *odom_ang_x_joint_element = cache_model_xml_doc.NewElement("joint");
							robot_body->InsertEndChild(odom_ang_x_joint_element);
							std::string ang_odom_x_joint_name = robot + "_ang_odom_x_joint";
							odom_ang_x_joint_element->SetAttribute("name", ang_odom_x_joint_name.c_str());
							odom_ang_x_joint_element->SetAttribute("type", "hinge");
							odom_ang_x_joint_element->SetAttribute("axis", "1 0 0");
							odom_joint_count++;
						}
						if (MjSim::add_odom_joints[robot]["ang_odom_y_joint"])
						{
							tinyxml2::XMLElement *odom_ang_y_joint_element = cache_model_xml_doc.NewElement("joint");
							robot_body->InsertEndChild(odom_ang_y_joint_element);
							std::string ang_odom_y_joint_name = robot + "_ang_odom_y_joint";
							odom_ang_y_joint_element->SetAttribute("name", ang_odom_y_joint_name.c_str());
							odom_ang_y_joint_element->SetAttribute("type", "hinge");
							odom_ang_y_joint_element->SetAttribute("axis", "0 1 0");
							odom_joint_count++;
						}
						if (MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
						{
							tinyxml2::XMLElement *odom_ang_z_joint_element = cache_model_xml_doc.NewElement("joint");
							robot_body->InsertEndChild(odom_ang_z_joint_element);
							std::string ang_odom_z_joint_name = robot + "_ang_odom_z_joint";
							odom_ang_z_joint_element->SetAttribute("name", ang_odom_z_joint_name.c_str());
							odom_ang_z_joint_element->SetAttribute("type", "hinge");
							odom_ang_z_joint_element->SetAttribute("axis", "0 0 1");
							odom_joint_count++;
						}
						break;
					}
				}
				ROS_INFO("Add %d odom joints for model %s successfully", odom_joint_count, robot.c_str());
			}
		}
	}

	for (tinyxml2::XMLElement *asset_element = cache_model_xml_doc.FirstChildElement()->FirstChildElement("asset");
		 asset_element != nullptr;
		 asset_element = asset_element->NextSiblingElement("asset"))
	{
		for (tinyxml2::XMLElement *texture_element = asset_element->FirstChildElement("texture");
			 texture_element != nullptr;
			 texture_element = texture_element->NextSiblingElement("texture"))
		{
			if (texture_element->Attribute("file") != nullptr)
			{
				const boost::filesystem::path file_path = texture_element->Attribute("file");
				if (file_path.is_relative())
				{
					texture_element->SetAttribute("file", (meshdir_abs_path / file_path).c_str());
				}
			}
		}
		for (tinyxml2::XMLElement *mesh_element = asset_element->FirstChildElement("mesh");
			 mesh_element != nullptr;
			 mesh_element = mesh_element->NextSiblingElement("mesh"))
		{
			if (mesh_element->Attribute("file") != nullptr)
			{
				const boost::filesystem::path file_path = mesh_element->Attribute("file");
				if (file_path.is_relative())
				{
					mesh_element->SetAttribute("file", (meshdir_abs_path / file_path).c_str());
				}
			}
		}
	}

	save_XML(cache_model_xml_doc, cache_model_path.c_str());
	ROS_INFO("Save models in %s successfully", tmp_model_path.parent_path().c_str());
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
	MjSim::ddq = (mjtNum *)mju_malloc(m->nv * sizeof(mjtNum *));
	mju_zero(MjSim::ddq, m->nv);
	MjSim::dq = (mjtNum *)mju_malloc(m->nv * sizeof(mjtNum *));
	mju_zero(MjSim::dq, m->nv);
}

static void modify_xml(const char *xml_path, const std::set<std::string> &remove_body_names = {""})
{
	tinyxml2::XMLDocument doc;
	if (!load_XML(doc, xml_path))
	{
		ROS_WARN("Failed to load file \"%s\"\n", xml_path);
		return;
	}

	mtx.lock();

	std::function<void(tinyxml2::XMLElement *)> add_bound_cb = [&](tinyxml2::XMLElement *compiler_element)
	{
		compiler_element->SetAttribute("boundmass", "0.000001");
		compiler_element->SetAttribute("boundinertia", "0.000001");
	};

	do_each_child_element(doc.FirstChildElement(), "compiler", add_bound_cb);

	tinyxml2::XMLElement *worldbody_element = doc.FirstChildElement()->FirstChildElement();
	std::set<tinyxml2::XMLElement *> body_elements_to_delete;
	for (tinyxml2::XMLElement *worldbody_element = doc.FirstChildElement()->FirstChildElement("worldbody");
		 worldbody_element != nullptr;
		 worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
	{
		for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement("body");
			 body_element != nullptr;
			 body_element = body_element->NextSiblingElement("body"))
		{
			const char *body_name = body_element->Attribute("name");
			if (body_name != nullptr && remove_body_names.find(body_name) != remove_body_names.end())
			{
				body_elements_to_delete.insert(body_element);
			}
			else if (body_name != nullptr &&
					 strcmp(body_name, model_path.stem().c_str()) != 0 &&
					 MjSim::robot_link_names.find(body_name) == MjSim::robot_link_names.end() &&
					 MjSim::robot_names.find(body_name) == MjSim::robot_names.end())
			{
				const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, body_name);
				body_element->SetAttribute("pos",
										   (std::to_string(d->xpos[3 * body_id]) + " " +
											std::to_string(d->xpos[3 * body_id + 1]) + " " +
											std::to_string(d->xpos[3 * body_id + 2]))
											   .c_str());
				body_element->SetAttribute("quat",
										   (std::to_string(d->xquat[4 * body_id]) + " " +
											std::to_string(d->xquat[4 * body_id + 1]) + " " +
											std::to_string(d->xquat[4 * body_id + 2]) + " " +
											std::to_string(d->xquat[4 * body_id + 3]))
											   .c_str());
			}
		}
	}

	if (body_elements_to_delete.size() > 0)
	{
		std::set<std::string> body_names_to_delete;
		std::set<std::string> joint_names_to_delete;
		for (tinyxml2::XMLElement *body_element_to_delete : body_elements_to_delete)
		{
			std::string body_name = body_element_to_delete->Attribute("name");
			body_names_to_delete.insert(body_name);

			do_each_child_element(body_element_to_delete, [&](tinyxml2::XMLElement *body_element)
								  {
															if (body_element->Attribute("name") != nullptr)
															{
																std::string body_name = body_element->Attribute("name");
																body_names_to_delete.insert(body_name);
															}
															for (tinyxml2::XMLElement *joint_element = body_element->FirstChildElement("joint");
																joint_element != nullptr;
																joint_element = joint_element->NextSiblingElement("joint"))
															{
																if (joint_element->Attribute("name") != nullptr)
																{
																	std::string joint_name = joint_element->Attribute("name");
																	joint_names_to_delete.insert(joint_name);
																}
															} });
		}

		for (tinyxml2::XMLElement *worldbody_element = doc.FirstChildElement()->FirstChildElement("worldbody");
			 worldbody_element != nullptr;
			 worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
		{
			for (tinyxml2::XMLElement *body_element_to_delete : body_elements_to_delete)
			{
				worldbody_element->DeleteChild(body_element_to_delete);
			}
		}

		std::set<tinyxml2::XMLElement *> exclude_elements_to_delete;
		for (tinyxml2::XMLElement *contact_element = doc.FirstChildElement()->FirstChildElement("contact");
			 contact_element != nullptr;
			 contact_element = contact_element->NextSiblingElement("contact"))
		{
			for (tinyxml2::XMLElement *exclude_element = contact_element->FirstChildElement("exclude");
				 exclude_element != nullptr;
				 exclude_element = exclude_element->NextSiblingElement("exclude"))
			{
				if (body_names_to_delete.find(exclude_element->Attribute("body1")) != body_names_to_delete.end() || body_names_to_delete.find(exclude_element->Attribute("body2")) != body_names_to_delete.end())
				{
					exclude_elements_to_delete.insert(exclude_element);
				}
			}
			for (tinyxml2::XMLElement *exclude_element : exclude_elements_to_delete)
			{
				contact_element->DeleteChild(exclude_element);
			}
		}

		std::set<tinyxml2::XMLElement *> joint_elements_to_delete;
		for (tinyxml2::XMLElement *equality_element = doc.FirstChildElement()->FirstChildElement("equality");
			 equality_element != nullptr;
			 equality_element = equality_element->NextSiblingElement("equality"))
		{
			for (tinyxml2::XMLElement *joint_element = equality_element->FirstChildElement("joint");
				 joint_element != nullptr;
				 joint_element = joint_element->NextSiblingElement("joint"))
			{
				if (joint_names_to_delete.find(joint_element->Attribute("joint1")) != joint_names_to_delete.end() || joint_names_to_delete.find(joint_element->Attribute("joint2")) != joint_names_to_delete.end())
				{
					joint_elements_to_delete.insert(joint_element);
				}
			}
			for (tinyxml2::XMLElement *joint_element : joint_elements_to_delete)
			{
				equality_element->DeleteChild(joint_element);
			}
		}
	}

	save_XML(doc, xml_path);

	mtx.unlock();
}

/***********************************/
/* Fix bug for m->geom_quat begins */
/***********************************/
bool save_geom_quat(const char *path)
{
	mtx.lock();
	tinyxml2::XMLDocument xml_doc;
	if (!load_XML(xml_doc, path))
	{
		ROS_WARN("Failed to load file \"%s\"\n", path);
		return false;
	}

	for (tinyxml2::XMLElement *worldbody_element = xml_doc.FirstChildElement()->FirstChildElement("worldbody");
		 worldbody_element != nullptr;
		 worldbody_element = worldbody_element->NextSiblingElement("worldbody"))
	{
		std::function<void(tinyxml2::XMLElement *, const char *)> func = [](tinyxml2::XMLElement *parent_element, const char *element_type)
		{
			do_each_child_element(parent_element, element_type, [](tinyxml2::XMLElement *element)
								  {
															if (element->Attribute("type") != nullptr && element->Attribute("type", "mesh"))
															{
																std::vector<mjtNum> geom_pos = {0, 0, 0};
																if (element->Attribute("pos") != nullptr)
																{
																	std::string pos_str = element->Attribute("pos");
																	std::istringstream iss(pos_str);
																	geom_pos = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};
																}
																for (size_t i = 0; i < geom_pos.size(); i++)
																{
																	geom_pos[i] *= mesh_paths[element->Attribute("mesh")].second[i];
																}

																std::vector<mjtNum> euler = {0, 0, 0};
																if (element->Attribute("quat") != nullptr)
																{
																	std::string quat_str = element->Attribute("quat");
																	std::istringstream iss(quat_str);
																	std::vector<mjtNum> geom_quat = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};
																	tf::Quaternion quat(geom_quat[1], geom_quat[2], geom_quat[3], geom_quat[0]);
																	tf::Matrix3x3 rot_mat(quat);
																	rot_mat.getRPY(euler[0], euler[1], euler[2]);
																}
																else if (element->Attribute("euler") != nullptr)
																{
																	std::string euler_str = element->Attribute("euler");
																	std::istringstream iss(euler_str);
																	euler = std::vector<mjtNum>{std::istream_iterator<mjtNum>(iss), std::istream_iterator<mjtNum>()};
																}
																for (size_t i = 0; i < euler.size(); i++)
																{
																	if (mesh_paths[element->Attribute("mesh")].second[i] < 0)
																	{
																		euler[i] += M_PI;
																	}
																}

																tf2::Quaternion quat;
																quat.setRPY(euler[0], euler[1], euler[2]);
																std::vector<mjtNum> geom_quat = {quat.getW(), quat.getX(), quat.getY(), quat.getZ()};

																const int mesh_id = mj_name2id(m, mjtObj::mjOBJ_MESH, element->Attribute("mesh"));
																for (int geom_id = 0; geom_id < m->ngeom; geom_id++)
																{
																	if (m->geom_dataid[geom_id] == mesh_id && MjSim::geom_pose.find(geom_id) == MjSim::geom_pose.end())
																	{
																		MjSim::geom_pose[geom_id].reserve(7);
																		MjSim::geom_pose[geom_id].insert(MjSim::geom_pose[geom_id].end(), geom_pos.begin(), geom_pos.end());
																		MjSim::geom_pose[geom_id].insert(MjSim::geom_pose[geom_id].end(), geom_quat.begin(), geom_quat.end());
																		break;
																	}
																}
															} });
		};

		do_each_child_element(worldbody_element, "geom", func);
	}
	mtx.unlock();
	return true;
}
/*********************************/
/* Fix bug for m->geom_quat ends */
/*********************************/

/**
 * @brief Load the tmp_model_path
 *
 * @param reset Reset the simulation or not
 * @return true if succeed
 */
bool load_tmp_model(bool reset)
{
	if (reset)
	{
		// load and compile model
		if (!load_XML(m, tmp_model_path.c_str()))
		{
			ROS_WARN("Could not load model file '%s'", tmp_model_path.c_str());
			return false;
		}

		// make data
		d = mj_makeData(m);
		init_malloc();

		MjSim::geom_pose.clear();
		return save_geom_quat(cache_model_path.c_str()) && save_geom_quat(tmp_model_path.c_str());
	}
	else
	{
		mtx.lock();
		// Load current.xml
		mjModel *m_new;
		if (!load_XML(m_new, tmp_model_path.c_str()))
		{
			ROS_WARN("Could not load model file '%s'", tmp_model_path.c_str());
			mtx.unlock();
			return false;
		}

		// make data
		mjData *d_new = mj_makeData(m_new);
		add_old_state(m_new, d_new);
		init_malloc();

		mtx.unlock();

		MjSim::geom_pose.clear();

		return save_geom_quat((tmp_model_path.parent_path() / "add.xml").c_str()) && save_geom_quat(tmp_model_path.c_str());
	}
}

static void init_references()
{
	XmlRpc::XmlRpcValue receive_params;
	if (ros::param::get("~receive", receive_params))
	{
		tinyxml2::XMLDocument xml_doc;
		if (!load_XML(xml_doc, tmp_model_path.c_str()))
		{
			ROS_WARN("Failed to load file \"%s\"\n", tmp_model_path.c_str());
			return;
		}

		mtx.lock();
		tinyxml2::XMLElement *mujoco_element = xml_doc.FirstChildElement();

		tinyxml2::XMLElement *equality_element = xml_doc.NewElement("equality");
		mujoco_element->LinkEndChild(equality_element);

		tinyxml2::XMLElement *contact_element = xml_doc.NewElement("contact");
		mujoco_element->LinkEndChild(contact_element);

		tinyxml2::XMLElement *worldbody_element = xml_doc.NewElement("worldbody");
		mujoco_element->LinkEndChild(worldbody_element);

		for (const std::pair<std::string, XmlRpc::XmlRpcValue> &receive_param : receive_params)
		{
			
			const std::string body_name = receive_param.first;
			const std::string ref_body_name = receive_param.first + "_ref";
			
			const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, body_name.c_str());
			const int ref_body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, ref_body_name.c_str());
			
			if (body_id != -1 && ref_body_id == -1)
			{
				tinyxml2::XMLElement *ref_body_element;
				do_each_child_element(mujoco_element, "worldbody", [&xml_doc, &ref_body_element, body_name](tinyxml2::XMLElement *worldbody_element)
									  {
										for (tinyxml2::XMLElement *body_element = worldbody_element->FirstChildElement("body");
											body_element != nullptr;
											body_element = body_element->NextSiblingElement("body"))
										{
											if (body_element->Attribute("name") != nullptr && body_element->Attribute("name", body_name.c_str()))
											{
												for (tinyxml2::XMLElement *geom_element = body_element->FirstChildElement("geom");
													geom_element != nullptr;
													geom_element = geom_element->NextSiblingElement("geom"))
												{
													geom_element->SetAttribute("rgba", ".9 0 0 1");
												}
												ref_body_element = body_element->DeepClone(&xml_doc)->ToElement();
											} 
										}; });
										
				ref_body_element->SetAttribute("name", ref_body_name.c_str());

				ref_body_element->SetAttribute("mocap", "true");

				for (tinyxml2::XMLElement *geom_element = ref_body_element->FirstChildElement("geom");
					geom_element != nullptr;
					geom_element = geom_element->NextSiblingElement("geom"))
				{
					geom_element->SetAttribute("rgba", ".5 .5 .5 1");
				}

				std::vector<tinyxml2::XMLElement *> joint_elements;
				for (tinyxml2::XMLElement *joint_element = ref_body_element->FirstChildElement("joint");
					 joint_element != nullptr;
					 joint_element = joint_element->NextSiblingElement("joint"))
				{
					joint_elements.push_back(joint_element);
				}
				for (tinyxml2::XMLElement *joint_element = ref_body_element->FirstChildElement("freejoint");
					 joint_element != nullptr;
					 joint_element = joint_element->NextSiblingElement("freejoint"))
				{
					joint_elements.push_back(joint_element);
				}
				
				for (tinyxml2::XMLElement *joint_element : joint_elements)
				{
					ref_body_element->DeleteChild(joint_element);
				}

				worldbody_element->InsertEndChild(ref_body_element);

				tinyxml2::XMLElement *weld_element = xml_doc.NewElement("weld");
				equality_element->LinkEndChild(weld_element);

				weld_element->SetAttribute("body1", body_name.c_str());
				weld_element->SetAttribute("body2", ref_body_name.c_str());
				weld_element->SetAttribute("torquescale", 0.9);

				for (int each_body_id = 0; each_body_id < m->nbody; each_body_id++)
				{		
					tinyxml2::XMLElement *exclude_element = xml_doc.NewElement("exclude");
					contact_element->LinkEndChild(exclude_element);

					exclude_element->SetAttribute("body1", mj_id2name(m, mjtObj::mjOBJ_BODY, each_body_id));
					exclude_element->SetAttribute("body2", ref_body_name.c_str());	
				}
			}
		}

		if (!save_XML(xml_doc, tmp_model_path.c_str()))
		{
			ROS_WARN("Failed to save file \"%s\"\n", tmp_model_path.c_str());
		}

		mtx.unlock();

		load_tmp_model(true);
	}
}

void MjSim::init()
{
	init_tmp();
	load_tmp_model(true);
	ROS_INFO("Reload model in %s complete", model_path.c_str());
	set_joint_names();
	init_sensors();
	init_references();
	sim_start = d->time;
}

void MjSim::init_sensors()
{
	for (int sensor_id = 0; sensor_id < m->nsensor; sensor_id++)
	{
		std::string sensor_name;
		if (m->sensor_type[sensor_id] == mjtSensor::mjSENS_FORCE)
		{
			if (mj_id2name(m, mjtObj::mjOBJ_SENSOR, sensor_id) == nullptr)
			{
				ROS_WARN("Sensor with id %d doesn't have a name, create one...", sensor_id);
				sensor_name = "force_sensor_";
				sensor_name += mj_id2name(m, mjtObj::mjOBJ_SITE, m->sensor_objid[sensor_id]);
				ROS_WARN("Created sensor %s", sensor_name.c_str());
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
				ROS_WARN("Sensor with id %d doesn't have a name, create one...", sensor_id);
				sensor_name = "torque_sensor_";
				sensor_name += mj_id2name(m, mjtObj::mjOBJ_SITE, m->sensor_objid[sensor_id]);
				ROS_WARN("Created sensor %s", sensor_name.c_str());
			}
			else
			{
				sensor_name = mj_id2name(m, mjtObj::mjOBJ_SENSOR, sensor_id);
			}
		}
		else
		{
			ROS_WARN("Sensor with type_id %d not implemented, ignore...", m->sensor_type[sensor_id]);
			continue;
		}

		sensors[sensor_id] = sensor_name;
	}
}

bool MjSim::add_data()
{
	// Save current.xml
	save_XML(m, tmp_model_path.c_str());

	modify_xml(tmp_model_path.c_str());

	// Add add.xml to current.xml
	tinyxml2::XMLDocument current_xml_doc;
	if (!load_XML(current_xml_doc, tmp_model_path.c_str()))
	{
		ROS_WARN("Failed to load file \"%s\"\n", tmp_model_path.c_str());
		return false;
	}

	tinyxml2::XMLElement *current_element = current_xml_doc.FirstChildElement();
	tinyxml2::XMLElement *include_element = current_xml_doc.NewElement("include");
	include_element->SetAttribute("file", "add.xml");
	current_element->LinkEndChild(include_element);
	if (!save_XML(current_xml_doc, tmp_model_path.c_str()))
	{
		ROS_WARN("Failed to save file \"%s\"\n", tmp_model_path.c_str());
		return false;
	}

	return load_tmp_model(false);
}

bool MjSim::remove_body(const std::set<std::string> &body_names)
{
	// Save current.xml
	save_XML(m, tmp_model_path.c_str());

	// Modify current.xml
	modify_xml(tmp_model_path.c_str(), body_names);

	return load_tmp_model(false);
}

void MjSim::controller()
{
	mj_mulM(m, d, tau, ddq);
	for (const std::string &joint_name : MjSim::controlled_joints)
	{
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
		const int dof_id = m->jnt_dofadr[joint_id];
		tau[dof_id] += d->qfrc_bias[dof_id];
	}

	mju_copy(d->qfrc_applied, tau, m->nv);

	for (int dof_id = 0; dof_id < m->nv; dof_id++)
	{
		if (mju_abs(dq[dof_id]) > mjMINVAL)
		{
			d->qvel[dof_id] = dq[dof_id];
		}
	}

	mju_zero(ddq, m->nv);
	mju_zero(dq, m->nv);
}

void MjSim::set_odom_vels()
{
	for (const std::string &robot : MjSim::robot_names)
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
			if (joint_id != -1)
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_cos(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) - mju_cos(odom_x_joint_pos) * mju_sin(odom_z_joint_pos)) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_cos(odom_z_joint_pos) + mju_sin(odom_x_joint_pos) * mju_sin(odom_z_joint_pos));
			}
		}
		if (MjSim::add_odom_joints[robot]["lin_odom_y_joint"])
		{
			const std::string joint_name = robot + "_lin_odom_y_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id != -1)
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_cos(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * (mju_sin(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) + mju_cos(odom_x_joint_pos) * mju_cos(odom_z_joint_pos)) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * (mju_cos(odom_x_joint_pos) * mju_sin(odom_y_joint_pos) * mju_sin(odom_z_joint_pos) - mju_sin(odom_x_joint_pos) * mju_cos(odom_z_joint_pos));
			}
		}
		if (MjSim::add_odom_joints[robot]["lin_odom_z_joint"])
		{
			const std::string joint_name = robot + "_lin_odom_z_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id != -1)
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = -MjSim::odom_vels[robot + "_lin_odom_x_joint"] * mju_sin(odom_y_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_y_joint"] * mju_sin(odom_x_joint_pos) * mju_cos(odom_y_joint_pos) + MjSim::odom_vels[robot + "_lin_odom_z_joint"] * mju_cos(odom_x_joint_pos) * mju_cos(odom_y_joint_pos);
			}
		}
		if (MjSim::add_odom_joints[robot]["ang_odom_x_joint"])
		{
			const std::string joint_name = robot + "_ang_odom_x_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id != -1)
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_ang_odom_x_joint"];
			}
		}
		if (MjSim::add_odom_joints[robot]["ang_odom_y_joint"])
		{
			const std::string joint_name = robot + "_ang_odom_y_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id != -1)
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_ang_odom_y_joint"];
			}
		}
		if (MjSim::add_odom_joints[robot]["ang_odom_z_joint"])
		{
			const std::string joint_name = robot + "_ang_odom_z_joint";
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
			if (joint_id != -1)
			{
				const int dof_id = m->jnt_dofadr[joint_id];
				d->qvel[dof_id] = MjSim::odom_vels[robot + "_ang_odom_z_joint"];
			}
		}
	}
}