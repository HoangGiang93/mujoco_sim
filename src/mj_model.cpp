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

#include "mj_model.h"

#include <ros/package.h>

mjModel *m = NULL;
mjData *d = NULL;

std::mutex mtx;

double rtf = 0.0;

std::string tmp_model_name = "current.xml";

std::string add_model_name = "add.xml";

boost::filesystem::path model_path = ros::package::getPath("mujoco_sim") + "/model/tmp/robot.xml";

boost::filesystem::path tmp_model_path = ros::package::getPath("mujoco_sim") + "/model/tmp/";

boost::filesystem::path add_model_path = ros::package::getPath("mujoco_sim") + "/model/tmp/" + add_model_name;

boost::filesystem::path cache_model_path = model_path;

boost::filesystem::path world_path = ros::package::getPath("mujoco_sim") + "/model/world/empty.xml";

boost::filesystem::path tmp_world_path = tmp_model_path;

std::map<std::string, boost::filesystem::path> mesh_paths;