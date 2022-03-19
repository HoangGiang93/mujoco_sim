#pragma once

#include "mujoco.h"

#include <mutex>

// MuJoCo data structures
extern mjModel *m; // MuJoCo model
extern mjData *d;  // MuJoCo data

extern std::mutex mtx;

extern double rtf_des;

extern double rtf;

extern std::string model_name;