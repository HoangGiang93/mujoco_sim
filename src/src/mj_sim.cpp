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

    mjtNum q_init_array[m->nv] = {0.};
    for (std::string q_name : q_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, q_name.c_str());
        q_init_array[idx] = q_inits[q_name];
    }
    mju_copy(d->qpos, q_init_array, m->nq);
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