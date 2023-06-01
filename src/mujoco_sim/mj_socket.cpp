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

std::map<std::string, std::vector<std::string>> MjSocket::publishers;

std::map<std::string, std::vector<std::string>> MjSocket::subscribers;

int length_published_data = 1;
int length_subscribed_data = 1;

std::string socket_header_addr;
std::string socket_data_addr;

MjSocket::~MjSocket()
{
	socket_header.disconnect(socket_header_addr);
	socket_data.disconnect(socket_data_addr);
}

void MjSocket::init(const int port_header, const int port_data)
{
	XmlRpc::XmlRpcValue subscriber_params;
    if (ros::param::get("~subscribers", subscriber_params))
    {
        std::string log = "Set subscribers: ";
        for (const std::pair<std::string, XmlRpc::XmlRpcValue> &subscriber_param : subscriber_params)
        {
            log += subscriber_param.first + " ";
            subscribers[subscriber_param.first] = {};
            ros::param::get("~subscribers/" + subscriber_param.first, subscribers[subscriber_param.first]);
        }
        ROS_INFO("%s", log.c_str());
    }

    XmlRpc::XmlRpcValue publisher_params;
    if (ros::param::get("~publishers", publisher_params))
    {
        std::string log = "Set publishers: ";
        for (const std::pair<std::string, XmlRpc::XmlRpcValue> &publisher_param : publisher_params)
        {

            std::vector<std::string> published_data;
            if (ros::param::get("~publishers/" + publisher_param.first, published_data))
            {
                if (publisher_param.first == "body")
                {
                    for (int body_id = 0; body_id < m->nbody; body_id++)
                    {
                        const char* body_name = mj_id2name(m, mjtObj::mjOBJ_BODY, body_id);
                        if (subscribers.count(body_name) == 0 && m->body_mocapid[body_id] == -1 && m->body_dofnum[body_id] != 0)
                        {
                            log += std::string(body_name) + " ";
                            publishers[body_name] = published_data;
                        }
                    }
                }
                else if (publisher_param.first == "joint_1D")
                {
                    for (int joint_id = 0; joint_id < m->njnt; joint_id++)
                    {
                        if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE || m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
                        {
                            const char* joint_name = mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id);
                            if (subscribers.count(joint_name) == 0)
                            {
                                log += std::string(joint_name) + " ";
                                publishers[joint_name] = published_data;
                            }
                        }
                    }
                }
            }
        }
        ROS_INFO("%s", log.c_str());
    }
	
	if (publishers.size() > 0 || subscribers.size() > 0)
	{
		ROS_INFO("Initializing the socket connection...");
		context = zmq::context_t{1};

		socket_header = zmq::socket_t{context, zmq::socket_type::req};
		socket_header_addr = host + ":" + std::to_string(port_header);
		socket_header.connect(socket_header_addr);

		socket_data = zmq::socket_t{context, zmq::socket_type::req};
		socket_data_addr = host + ":" + std::to_string(port_data);
		socket_data.connect(socket_data_addr);

		send_header();
		ROS_INFO("Initialized the socket connection with port (%d %d) successfully.", port_header, port_data);
	}
}

void MjSocket::send_header()
{
	// Create JSON object and populate it
	Json::Value header_json;
	header_json["time"] = "microseconds";
	header_json["simulator"] = "mujoco";

	length_published_data = 1;
	length_subscribed_data = 1;

	for (const std::pair<std::string, std::vector<std::string>>& publisher : publishers)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, publisher.first.c_str());
		publisher_ids.push_back(body_id);
		for (const std::string &publisher_data : publisher.second)
		{
			header_json["publishers"][publisher.first].append(publisher_data);
			length_published_data += attribute_map[publisher_data].size();
		}
	}

	for (const std::pair<std::string, std::vector<std::string>>& subscriber : subscribers)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, subscriber.first.c_str());
		subscriber_ids.push_back(m->body_mocapid[mj_name2id(m, mjtObj::mjOBJ_BODY, (subscriber.first + "_ref").c_str())]);
		for (const std::string &subscriber_data : subscriber.second)
		{
			header_json["subscribers"][subscriber.first].append(subscriber_data);
			length_subscribed_data += attribute_map[subscriber_data].size();
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
	ROS_INFO("Start communication on %s with a publisher of length %d and a subscriber of length %d", socket_data_addr.c_str(), length_published_data, length_subscribed_data);
	double published_data[length_published_data];
	double subscribed_data[length_subscribed_data];
	while (ros::ok())
	{
		published_data[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		int i = 1;
		for (const int body_id : publisher_ids)
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
		for (const int mocap_id : subscriber_ids)
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