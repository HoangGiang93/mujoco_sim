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

    void computed_torque_controller();

public:
    static std::vector<std::string> q_names;
    static double sim_start;

    static std::map<std::string, mjtNum> q_inits;
    static std::map<std::string, mjtNum> q_refs;
    static std::map<std::string, mjtNum> dq_refs;
    static std::map<std::string, mjtNum> ddq_refs;

    static std::vector<double> Kp;
    static std::vector<double> Kv;
    static std::vector<double> Ki;

private:
    std::map<std::string, mjtNum> e_sum;
    mjtNum *u;
    mjtNum *tau;
};