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

std::map<std::string, std::vector<std::string>> MjSocket::asked_objects;

int length_told_data = 1;
int length_asked_data = 1;

MjSocket::~MjSocket()
{
	
}

void MjSocket::init(const int port_header, const int port_data)
{
	context = zmq::context_t{1};

	socket_header = zmq::socket_t{context, zmq::socket_type::req};
	socket_header.connect(ip_addr + ":" + std::to_string(port_header));

	socket_data = zmq::socket_t{context, zmq::socket_type::req};
	socket_data.connect(ip_addr + ":" + std::to_string(port_data));

	send_header();
}

void MjSocket::send_header()
{
	// Create JSON object and populate it
	Json::Value header_json;
	header_json["time"] = "microseconds";
	header_json["simulator"] = "mujoco";

	Json::Value told_data_json(Json::arrayValue);
	told_data_json.append("position");
	told_data_json.append("quaternion");

	length_told_data = 1;
	length_asked_data = 1;

	for (int body_id = 1; body_id < m->nbody; body_id++)
	{
		const std::string body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
		if (asked_objects.count(body_name) == 0)
		{
			if (m->body_mocapid[body_id] == -1 && m->body_dofnum[body_id] != 0)
			{
				header_json["tells"][body_name] = told_data_json;
				length_told_data += 7;
				told_object_ids.push_back(body_id);
			}
		}
		else
		{
			Json::Value asked_data_json(Json::arrayValue);
			for (const std::string &asked_data : asked_objects[body_name])
			{
				asked_data_json.append(asked_data);
				if (asked_data == "position")
				{
					length_asked_data += 3;
				}
				else if (asked_data == "quaternion")
				{
					length_asked_data += 4;
				}
			}
			header_json["asks"][body_name] = asked_data_json;
			asked_object_ids.push_back(m->body_mocapid[mj_name2id(m, mjtObj::mjOBJ_BODY, (body_name + "_ref").c_str())]);
		}
	}

	std::string header_str = header_json.toStyledString();

	// Send JSON string over ZMQ
	zmq::message_t request_header(header_str.size());
	memcpy(request_header.data(), header_str.c_str(), header_str.size());
	socket_header.send(request_header, zmq::send_flags::none);
}

void MjSocket::communicate()
{
	zmq_sleep(1);
	double told_data[length_told_data];
	double asked_data[length_asked_data];
	while (ros::ok())
	{
		told_data[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		int i = 1;
		for (const int body_id : told_object_ids)
		{
			told_data[i] = d->xpos[3 * body_id];
			told_data[i + 1] = d->xpos[3 * body_id + 1];
			told_data[i + 2] = d->xpos[3 * body_id + 2];
			told_data[i + 3] = d->xquat[4 * body_id];
			told_data[i + 4] = d->xquat[4 * body_id + 1];
			told_data[i + 5] = d->xquat[4 * body_id + 2];
			told_data[i + 6] = d->xquat[4 * body_id + 3];
			i += 7;
		}

		zmq::message_t request(sizeof(told_data));
		memcpy(request.data(), &told_data, sizeof(told_data));
		socket_data.send(request, zmq::send_flags::none);

		zmq::message_t reply(sizeof(asked_data));
		socket_data.recv(reply);
		memcpy(&asked_data, reply.data(), sizeof(asked_data));

		i = 1;		
		for (const int mocap_id : asked_object_ids)
		{
			d->mocap_pos[3 * mocap_id] = asked_data[i];
			d->mocap_pos[3 * mocap_id + 1] = asked_data[i + 1];
			d->mocap_pos[3 * mocap_id + 2] = asked_data[i + 2];
			d->mocap_quat[4 * mocap_id] = asked_data[i + 3];
			d->mocap_quat[4 * mocap_id + 1] = asked_data[i + 4];
			d->mocap_quat[4 * mocap_id + 2] = asked_data[i + 5];
			d->mocap_quat[4 * mocap_id + 3] = asked_data[i + 6];
			i += 7;
		}
	}
}