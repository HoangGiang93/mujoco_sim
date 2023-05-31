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

std::map<std::string, std::vector<std::string>> MjSocket::subscribed_objects;

int length_published_data = 1;
int length_subscribed_data = 1;

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

	Json::Value published_data_json(Json::arrayValue);
	published_data_json.append("position");
	published_data_json.append("quaternion");

	length_published_data = 1;
	length_subscribed_data = 1;

	for (int body_id = 1; body_id < m->nbody; body_id++)
	{
		const std::string body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
		if (subscribed_objects.count(body_name) == 0)
		{
			if (m->body_mocapid[body_id] == -1 && m->body_dofnum[body_id] != 0)
			{
				header_json["published_objects"][body_name] = published_data_json;
				length_published_data += 7;
				published_object_ids.push_back(body_id);
			}
		}
		else
		{
			Json::Value subscribed_data_json(Json::arrayValue);
			for (const std::string &subscribed_data : subscribed_objects[body_name])
			{
				subscribed_data_json.append(subscribed_data);
				if (subscribed_data == "position")
				{
					length_subscribed_data += 3;
				}
				else if (subscribed_data == "quaternion")
				{
					length_subscribed_data += 4;
				}
			}
			header_json["subscribed_objects"][body_name] = subscribed_data_json;
			subscribed_object_ids.push_back(m->body_mocapid[mj_name2id(m, mjtObj::mjOBJ_BODY, (body_name + "_ref").c_str())]);
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
	double published_data[length_published_data];
	double subscribed_data[length_subscribed_data];
	while (ros::ok())
	{
		published_data[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		int i = 1;
		for (const int body_id : published_object_ids)
		{
			published_data[i] = d->xpos[3 * body_id];
			published_data[i + 1] = d->xpos[3 * body_id + 1];
			published_data[i + 2] = d->xpos[3 * body_id + 2];
			published_data[i + 3] = d->xquat[4 * body_id];
			published_data[i + 4] = d->xquat[4 * body_id + 1];
			published_data[i + 5] = d->xquat[4 * body_id + 2];
			published_data[i + 6] = d->xquat[4 * body_id + 3];
			i += 7;
		}

		zmq::message_t request(sizeof(published_data));
		memcpy(request.data(), &published_data, sizeof(published_data));
		socket_data.send(request, zmq::send_flags::none);

		zmq::message_t reply(sizeof(subscribed_data));
		socket_data.recv(reply);
		memcpy(&subscribed_data, reply.data(), sizeof(subscribed_data));

		i = 1;		
		for (const int mocap_id : subscribed_object_ids)
		{
			d->mocap_pos[3 * mocap_id] = subscribed_data[i];
			d->mocap_pos[3 * mocap_id + 1] = subscribed_data[i + 1];
			d->mocap_pos[3 * mocap_id + 2] = subscribed_data[i + 2];
			d->mocap_quat[4 * mocap_id] = subscribed_data[i + 3];
			d->mocap_quat[4 * mocap_id + 1] = subscribed_data[i + 4];
			d->mocap_quat[4 * mocap_id + 2] = subscribed_data[i + 5];
			d->mocap_quat[4 * mocap_id + 3] = subscribed_data[i + 6];
			i += 7;
		}
	}
}