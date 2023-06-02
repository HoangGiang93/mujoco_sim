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

#include "../state_msgs.cpp"

zmq::context_t context;

std::string host = "tcp://127.0.0.1";

zmq::socket_t socket_header;

zmq::socket_t socket_data;

std::map<std::string, std::vector<std::string>> MjSocket::send_objects;

std::map<std::string, std::vector<std::string>> MjSocket::receive_objects;

size_t send_data_size = 1;
size_t receive_data_size = 1;

std::string socket_header_addr;
std::string socket_data_addr;

MjSocket::~MjSocket()
{
	socket_header.disconnect(socket_header_addr);
	socket_data.disconnect(socket_data_addr);
}

void MjSocket::init(const int header_port, const int data_port)
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
		context = zmq::context_t{1};

		socket_header = zmq::socket_t{context, zmq::socket_type::req};
		socket_header_addr = host + ":" + std::to_string(header_port);
		socket_header.connect(socket_header_addr);

		socket_data = zmq::socket_t{context, zmq::socket_type::req};
		socket_data_addr = host + ":" + std::to_string(data_port);
		socket_data.connect(socket_data_addr);
	}
}

bool MjSocket::send_header()
{
	// Create JSON object and populate it
	Json::Value header_json;
	header_json["time"] = "microseconds";
	header_json["simulator"] = "mujoco";

	send_data_size = 1;
	receive_data_size = 1;

	for (const std::pair<std::string, std::vector<std::string>>& send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		send_object_ids.push_back(body_id);
		for (const std::string &send_data : send_object.second)
		{
			header_json["send"][send_object.first].append(send_data);
			send_data_size += attribute_map[send_data].size();
		}
	}

	for (const std::pair<std::string, std::vector<std::string>>& receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		receive_object_ids.push_back(m->body_mocapid[mj_name2id(m, mjtObj::mjOBJ_BODY, (receive_object.first + "_ref").c_str())]);
		for (const std::string &receive_data : receive_object.second)
		{
			header_json["receive"][receive_object.first].append(receive_data);
			receive_data_size += attribute_map[receive_data].size();
		}
	}

	std::string header_str = header_json.toStyledString();

	// Send JSON string over ZMQ
	zmq::message_t request_header(header_str.size());
	memcpy(request_header.data(), header_str.c_str(), header_str.size());
	socket_header.send(request_header, zmq::send_flags::none);

	// Receive buffer sizes over ZMQ
	size_t buffer[2];
	zmq::message_t reply_header(sizeof(buffer));
	socket_header.recv(reply_header);
	memcpy(&buffer, reply_header.data(), sizeof(buffer));

	if (buffer[0] != send_data_size || buffer[1] != receive_data_size)
	{
		ROS_ERROR("Failed to initialize the socket header at %s: send_data_size(server = %ld != client = %ld), receive_data_size(server = %ld != client = %ld).", socket_header_addr.c_str(), buffer[0], send_data_size, buffer[1], receive_data_size);
		return false;
	}
	else
	{
		ROS_INFO("Initialized the socket header at %s successfully.", socket_header_addr.c_str());
		return true;
	}
}

void MjSocket::communicate()
{
	if (!send_header())
	{
		return;
	}

	ROS_INFO("Start communication on %s with a send_object of length %ld and a receive_object of length %ld", socket_data_addr.c_str(), send_data_size, receive_data_size);
	double send_buffer[send_data_size];
	double receive_buffer[receive_data_size];
	while (ros::ok())
	{
		send_buffer[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		int i = 1;
		for (const int body_id : send_object_ids)
		{
			send_buffer[i] = d->xpos[3 * body_id];
			send_buffer[i + 1] = d->xpos[3 * body_id + 1];
			send_buffer[i + 2] = d->xpos[3 * body_id + 2];
			send_buffer[i + 3] = d->xquat[4 * body_id];
			send_buffer[i + 4] = d->xquat[4 * body_id + 1];
			send_buffer[i + 5] = d->xquat[4 * body_id + 2];
			send_buffer[i + 6] = d->xquat[4 * body_id + 3];
			i += 7;
		}
		
		zmq::message_t request(sizeof(send_buffer));
		memcpy(request.data(), &send_buffer, sizeof(send_buffer));
		socket_data.send(request, zmq::send_flags::none);
		
		zmq::message_t reply(sizeof(receive_buffer));
		socket_data.recv(reply);
		memcpy(&receive_buffer, reply.data(), sizeof(receive_buffer));
		i = 1;		
		for (const int mocap_id : receive_object_ids)
		{
			d->mocap_pos[3 * mocap_id] = receive_buffer[i];
			d->mocap_pos[3 * mocap_id + 1] = receive_buffer[i + 1];
			d->mocap_pos[3 * mocap_id + 2] = receive_buffer[i + 2];
			d->mocap_quat[4 * mocap_id] = receive_buffer[i + 3];
			d->mocap_quat[4 * mocap_id + 1] = receive_buffer[i + 4];
			d->mocap_quat[4 * mocap_id + 2] = receive_buffer[i + 5];
			d->mocap_quat[4 * mocap_id + 3] = receive_buffer[i + 6];
			i += 7;
		}
	}
}