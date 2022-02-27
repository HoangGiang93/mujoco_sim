#pragma once

#include "mj_model.h"
#include "vector"
#include "string"
#include "mj_ros.h"

class MjSim
{
public:
    MjSim();

public:
    void init();

    void sparse2dense(mjtNum *mat_dense, mjtNum *mat_sparse);

    void computed_torque_controller();

public:
    static std::vector<int> q_idx;
    static double sim_start;

    mjtNum *q_init;
    static mjtNum *q_ref;
    static mjtNum *dq_ref;
    static mjtNum *ddq_ref;

    static std::vector<double> Kp;
    static std::vector<double> Kv;
    static std::vector<double> Ki;

private:
    int n_dof;
    mjtNum *e_sum;
};