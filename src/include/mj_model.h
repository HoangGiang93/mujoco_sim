#pragma once

#include "mujoco.h"

#include <experimental/filesystem>
#include <mutex>

// MuJoCo data structures
extern mjModel *m; // MuJoCo model
extern mjData *d;  // MuJoCo data

extern std::mutex mtx;

extern double rtf_des;

extern double rtf;

extern std::experimental::filesystem::path model_path;

extern std::experimental::filesystem::path tmp_model_path;