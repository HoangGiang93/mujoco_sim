#pragma once

#include "mj_model.h"
#include "vector"
#include "string"
#include "map"

class MjSim
{
public:
    MjSim() = default;

public:
    void init();

    void init_malloc();

    void add_data(std::string data_xml_path);

    void controller();

public:
    static std::vector<std::string> q_names;

    static std::map<std::string, mjtNum> q_inits;

    static mjtNum *u;

    static mjtNum sim_start;

private:    
    mjtNum *tau;
};