#pragma once

#include "mj_model.h"

#include <map>
#include <vector>

class MjSim
{
public:
    MjSim() = default;

    ~MjSim();

public:
    void init();

    void init_tmp();

    static void init_malloc();

    static void add_data();

    void controller();

public:
    static std::vector<std::string> joint_names;

    static std::vector<std::string> link_names;

    static std::map<std::string, mjtNum> q_inits;

    static mjtNum sim_start;

    static mjtNum *tau;
};