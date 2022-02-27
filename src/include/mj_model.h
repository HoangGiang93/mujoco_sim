#pragma once

#include "mujoco.h"
#include "mutex"

// MuJoCo data structures
extern mjModel *m; // MuJoCo model
extern mjData *d;  // MuJoCo data

extern std::mutex mtx;