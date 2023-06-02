// Copyright (c) 2023, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#include "mj_socket.h"

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <jsoncpp/json/json.h>
#include <zmq.h>

std::string host = "tcp://127.0.0.1";

std::map<std::string, std::vector<std::string>> MjSocket::send_objects;

std::map<std::string, std::vector<std::string>> MjSocket::receive_objects;

MjSocket::~MjSocket()
{
	zmq_disconnect(socket_client, socket_client_addr.c_str());
}

void MjSocket::init(const int port)
{
	XmlRpc::XmlRpcValue receive_object_params;
    if (ros::param::get("~receive", receive_object_params))
    {
        std::string log = "Set receive_objects: ";
        for (const std::pair<std::string, XmlRpc::XmlRpcValue> &receive_object_param : receive_object_params)
        {
            log += receive_object_param.first + " ";
            receive_objects[receive_object_param.first] = {};
            ros::param::get("~receive/" + receive_object_param.first, receive_objects[receive_object_param.first]);
        }
        ROS_INFO("%s", log.c_str());
    }

    XmlRpc::XmlRpcValue send_object_params;
    if (ros::param::get("~send", send_object_params))
    {
        std::string log = "Set send_objects: ";
        for (const std::pair<std::string, XmlRpc::XmlRpcValue> &send_object_param : send_object_params)
        {

            std::vector<std::string> send_data;
            if (ros::param::get("~send/" + send_object_param.first, send_data))
            {
                if (send_object_param.first == "body")
                {
                    for (int body_id = 0; body_id < m->nbody; body_id++)
                    {
                        const char* body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
                        if (receive_objects.count(body_name) == 0 && m->body_mocapid[body_id] == -1 && m->body_dofnum[body_id] != 0)
                        {
                            log += std::string(body_name) + " ";
                            send_objects[body_name] = send_data;
                        }
                    }
                }
                else if (send_object_param.first == "joint_1D")
                {
                    for (int joint_id = 0; joint_id < m->njnt; joint_id++)
                    {
                        if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE || m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
                        {
                            const char* joint_name = mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id);
                            if (receive_objects.count(joint_name) == 0)
                            {
                                log += std::string(joint_name) + " ";
                                send_objects[joint_name] = send_data;
                            }
                        }
                    }
                }
            }
        }
        ROS_INFO("%s", log.c_str());
    }
	
	if (send_objects.size() > 0 || receive_objects.size() > 0)
	{
		ROS_INFO("Initializing the socket connection...");
		context = zmq_ctx_new();

        socket_client = zmq_socket(context, ZMQ_REQ);
		socket_client_addr = host + ":" + std::to_string(port);
		zmq_connect(socket_client, socket_client_addr.c_str());
	}
}

bool MjSocket::send_header()
{
	// Create JSON object and populate it
	Json::Value header_json;
	header_json["time"] = "microseconds";
	header_json["simulator"] = "mujoco";

	mtx.lock();
	for (const std::pair<std::string, std::vector<std::string>>& send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		for (const std::string &attribute : send_object.second)
		{
			if (strcmp(attribute.c_str(), "position"))
			{
				send_data_vec.push_back(&d->xpos[3 * body_id]);
				send_data_vec.push_back(&d->xpos[3 * body_id + 1]);
				send_data_vec.push_back(&d->xpos[3 * body_id + 2]);
			}
			else if (strcmp(attribute.c_str(), "quaternion"))
			{
				send_data_vec.push_back(&d->xquat[4 * body_id]);
				send_data_vec.push_back(&d->xquat[4 * body_id + 1]);
				send_data_vec.push_back(&d->xquat[4 * body_id + 2]);
				send_data_vec.push_back(&d->xquat[4 * body_id + 3]);
			}
			
			header_json["send"][send_object.first].append(attribute);
		}
	}
	mtx.unlock();
	send_data_size = 1 + send_data_vec.size();

	mtx.lock();
	for (const std::pair<std::string, std::vector<std::string>>& receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (receive_object.first + "_ref").c_str());
		const int mocap_id = m->body_mocapid[body_id];
		for (const std::string &attribute : receive_object.second)
		{
			if (strcmp(attribute.c_str(), "position"))
			{
				receive_data_vec.push_back(&d->mocap_pos[3 * mocap_id]);
				receive_data_vec.push_back(&d->mocap_pos[3 * mocap_id + 1]);
				receive_data_vec.push_back(&d->mocap_pos[3 * mocap_id + 2]);
			}
			else if (strcmp(attribute.c_str(), "quaternion"))
			{
				receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id]);
				receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id + 1]);
				receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id + 2]);
				receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id + 3]);
			}

			header_json["receive"][receive_object.first].append(attribute);
		}
	}
	mtx.unlock();
	receive_data_size = 1 + receive_data_vec.size();

	// Send JSON string over ZMQ
	const std::string header_str = header_json.toStyledString();
	zmq_send(socket_client, header_str.c_str(), header_str.size(), 0);

	// Receive buffer sizes over ZMQ
	size_t buffer[2];
	zmq_recv(socket_client, buffer, sizeof(buffer), 0);

	if (buffer[0] != send_data_size || buffer[1] != receive_data_size)
	{
		ROS_ERROR("Failed to initialize the socket header at %s: send_data_size(server = %ld != client = %ld), receive_data_size(server = %ld != client = %ld).", socket_client_addr.c_str(), buffer[0], send_data_size, buffer[1], receive_data_size);
		return false;
	}
	else
	{
		ROS_INFO("Initialized the socket header at %s successfully.", socket_client_addr.c_str());
		return true;
	}
}

void MjSocket::communicate()
{
	if (!send_header())
	{
		return;
	}

	ROS_INFO("Start communication on %s with a send_object of length %ld and a receive_object of length %ld", socket_client_addr.c_str(), send_data_size, receive_data_size);
	double send_buffer[send_data_size];
	double receive_buffer[receive_data_size];
	while (ros::ok())
	{
		send_buffer[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		
		for (size_t i = 0; i < send_data_size - 1; i++)
        {
            send_buffer[i + 1] = *send_data_vec[i];
        }
		
		zmq_send(socket_client, send_buffer, sizeof(send_buffer), 0);
		
		zmq_recv(socket_client, receive_buffer, sizeof(receive_buffer), 0);

		for (size_t i = 1; i < receive_data_size; i++)
        {
            *receive_data_vec[i - 1] = receive_buffer[i];
        }
	}
}