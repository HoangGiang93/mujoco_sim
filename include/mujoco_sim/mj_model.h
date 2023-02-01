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

#pragma once

#include <mujoco/mujoco.h>

#include <boost/filesystem.hpp>
#include <mutex>

// MuJoCo data structures
extern mjModel *m; // MuJoCo model
extern mjData *d;  // MuJoCo data

extern std::mutex mtx;

extern double rtf;

extern std::string tmp_model_name;

extern std::string add_model_name;

extern boost::filesystem::path model_path;

extern boost::filesystem::path tmp_model_path;

extern boost::filesystem::path add_model_path;

extern boost::filesystem::path cache_model_path;

extern boost::filesystem::path world_path;

extern boost::filesystem::path tmp_world_path;

extern std::map<std::string, boost::filesystem::path> mesh_paths;