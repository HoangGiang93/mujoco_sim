#include "mj_sim.h"
#include "iostream"

std::vector<std::string> MjSim::q_names;

std::map<std::string, mjtNum> MjSim::q_inits;

mjtNum *MjSim::u = NULL;

mjtNum MjSim::sim_start;

MjSim::MjSim()
{

}

void MjSim::init()
{
    u = (mjtNum *)malloc(m->nv * sizeof(mjtNum *)); mju_zero(u, m->nv);
    sim_start = d->time;
    
    tau = (mjtNum *)malloc(m->nv * sizeof(mjtNum *)); mju_zero(tau, m->nv);

    for (const std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        d->qpos[idx] = q_inits[q_name];
    }
    mj_forward(m, d);
}

void MjSim::controller()
{
    mj_mulM(m, d, tau, u);
    for (const std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        tau[idx] += d->qfrc_bias[idx];
    }
    
    mju_copy(d->qfrc_applied, tau, m->nv);
}