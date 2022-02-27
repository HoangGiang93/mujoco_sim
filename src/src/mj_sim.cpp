#include "mj_sim.h"

std::vector<int> MjSim::q_idx;

mjtNum *MjSim::q_ref = NULL;
mjtNum *MjSim::dq_ref = NULL;
mjtNum *MjSim::ddq_ref = NULL;

std::vector<double> MjSim::Kp;
std::vector<double> MjSim::Kv;
std::vector<double> MjSim::Ki;

double MjSim::sim_start = 0.;

MjSim::MjSim()
{

}

void MjSim::init()
{
    sim_start = d->time;
    n_dof = MjRos::n_dof;
    
    e_sum = (double *)malloc(n_dof * sizeof(double *));
    q_init = (double *)malloc(n_dof * sizeof(double *));
    q_ref = (double *)malloc(n_dof * sizeof(double *));
    dq_ref = (double *)malloc(n_dof * sizeof(double *));
    ddq_ref = (double *)malloc(n_dof * sizeof(double *));

    mju_zero(e_sum, n_dof);
    mju_zero(q_init, n_dof);
    mju_copy(q_ref, q_init, n_dof);
    mju_zero(dq_ref, n_dof);
    mju_zero(ddq_ref, n_dof);
    mju_copy(d->qpos, q_init, n_dof);
    
    mj_step(m, d);
}

void MjSim::sparse2dense(mjtNum *mat_dense, mjtNum *mat_sparse)
{
    int k_0 = 0;
    int k;
    for (int i = 0; i < n_dof; i++)
    {
        k_0 += i;
        k = k_0;
        for (int j = 0; j < n_dof; j++)
        {
            if (j < i)
            {
                mat_dense[n_dof * i + j] = mat_dense[n_dof * j + i];
            }
            else
            {
                if (j > i)
                {
                    k += j + 1;
                }
                mat_dense[n_dof * i + j] = mat_sparse[k];
            }
        }
    }
}

void MjSim::computed_torque_controller()
{
    mjtNum u[n_dof];
    for (int i = 0; i < n_dof; i++)
    {
        u[i] = Kp[i] * (q_ref[i] - d->qpos[i]) + Kv[i] * (dq_ref[i] - d->qvel[i]) + ddq_ref[i] + Ki[i] * e_sum[i];
        e_sum[i] += (q_ref[i] - d->qpos[i]) * m->opt.timestep;
    }
    mjtNum M_sparse[n_dof * (n_dof - 1) / 2 + n_dof] = {0.};
    mjtNum M_dense[n_dof * n_dof] = {0.};
    mjtNum tau[n_dof];
    mju_copy(M_sparse, d->qM, n_dof * (n_dof - 1) / 2 + n_dof);
    sparse2dense(M_dense, M_sparse);
    mju_mulMatVec(tau, M_dense, u, n_dof, n_dof);
    mju_addTo(tau, d->qfrc_bias, n_dof);
    mju_copy(d->qfrc_applied, tau, n_dof);
}