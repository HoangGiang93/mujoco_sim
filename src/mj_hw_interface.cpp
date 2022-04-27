// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mj_hw_interface.h"

MjHWInterface::MjHWInterface()
{
    std::size_t num_joints = MjSim::joint_names.size();
    joint_positions.resize(num_joints, 0.);
    joint_velocities.resize(num_joints, 0.);
    joint_efforts.resize(num_joints, 0.);

    // joint_positions_command.resize(num_joints, 0.);
    // joint_velocities_command.resize(num_joints, 0.);
    joint_efforts_command.resize(num_joints, 0.);

    for (std::size_t i = 0; i < num_joints; i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(MjSim::joint_names[i], &joint_positions[i], &joint_velocities[i], &joint_efforts[i]);
        joint_state_interface.registerHandle(joint_state_handle);

        // hardware_interface::JointHandle joint_handle_position(joint_state_interface.getHandle(MjSim::joint_names[i]), &joint_positions_command[i]);
        // position_joint_interface.registerHandle(joint_handle_position);

        // hardware_interface::JointHandle joint_handle_velocity(joint_state_interface.getHandle(MjSim::joint_names[i]), &joint_velocities_command[i]);
        // velocity_joint_interface.registerHandle(joint_handle_velocity);

        hardware_interface::JointHandle joint_handle_effort(joint_state_interface.getHandle(MjSim::joint_names[i]), &joint_efforts_command[i]);
        effort_joint_interface.registerHandle(joint_handle_effort);
    }
    registerInterface(&joint_state_interface);
    // registerInterface(&position_joint_interface);
    // registerInterface(&velocity_joint_interface);
    registerInterface(&effort_joint_interface);
}

MjHWInterface::~MjHWInterface()
{
}

void MjHWInterface::read()
{
    mj_inverse(m, d);
    for (std::size_t i = 0; i < MjSim::joint_names.size(); i++)
    {
        const int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, MjSim::joint_names[i].c_str());
        joint_positions[i] = d->qpos[idx];
        joint_velocities[i] = d->qvel[idx];
        joint_efforts[i] = d->qfrc_inverse[idx];
    }
}

void MjHWInterface::write()
{
    mjtNum u[m->nv] = {0.};
    for (std::size_t i = 0; i < MjSim::joint_names.size(); i++)
    {
        const int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, MjSim::joint_names[i].c_str());
        u[idx] = joint_efforts_command[i];
    }
    mj_mulM(m, d, MjSim::tau, u);
    for (const std::string joint_name : MjSim::joint_names)
    {
        int idx = mj_name2id(m, mjtObj::mjOBJ_JOINT, joint_name.c_str());
        MjSim::tau[idx] += d->qfrc_bias[idx];
    }
}