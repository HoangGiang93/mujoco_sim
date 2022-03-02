#include "mj_hw_interface.h"

MjHWInterface::MjHWInterface()
{
    n = ros::NodeHandle();
    if (!n.getParam("joint_names", MjSim::q_names))
    {
        mju_error_s("Couldn't find joint_names in %s/joint_names", n.getNamespace().c_str());
    }
    if (!n.getParam("init_positions", MjSim::q_inits))
    {
        mju_warning_s("Couldn't find joints and positions in %s/init_positions, set default to 0", n.getNamespace().c_str());
    }

    num_joints = MjSim::q_names.size();
    joint_positions.resize(num_joints, 0.);
    joint_velocities.resize(num_joints, 0.);
    joint_efforts.resize(num_joints, 0.);
    joint_efforts_command.resize(num_joints, 0.);

    for (std::size_t i = 0; i < num_joints; i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(MjSim::q_names[i], &joint_positions[i], &joint_velocities[i], &joint_efforts[i]);
        joint_state_interface.registerHandle(joint_state_handle);
        
        hardware_interface::JointHandle joint_handle_position(joint_state_interface.getHandle(MjSim::q_names[i]), &joint_efforts_command[i]);
        effort_joint_interface.registerHandle(joint_handle_position);
    }
    registerInterface(&joint_state_interface);
    registerInterface(&effort_joint_interface);
}

MjHWInterface::~MjHWInterface()
{

}

void MjHWInterface::read(const ros::Time& , const ros::Duration& period)
{
    for (std::size_t i = 0; i < MjSim::q_names.size(); i++)
    {
        const int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, MjSim::q_names[i].c_str());
        joint_positions[i] = d->qpos[idx];
        joint_velocities[i] = d->qvel[idx];
        joint_efforts[i] = d->qfrc_inverse[idx];
    }
}

void MjHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
    mju_zero(MjSim::u, m->nv);
    for (std::size_t i = 0; i < MjSim::q_names.size(); i++)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, MjSim::q_names[i].c_str());
        MjSim::u[idx] = joint_efforts_command[i];
    }
    
}