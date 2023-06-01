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

#include <ros/ros.h>

#include <string>
#include <chrono>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <thread>
#include <mutex>

#include <zmq.hpp>

#include "state_msgs.cpp"

using namespace std::chrono_literals;

int port_header = 7500;
int port_data = 7600;

zmq::context_t context{1};

std::mutex mtx;

std::map<std::string, std::map<std::string, std::vector<double>>> send_objects;

void worker(const size_t thread_num, const Json::Value &json_header)
{
    Json::Value send_objects_json = json_header["send"];

    std::vector<double*> send_data_vec;
    mtx.lock();
    for (auto it = send_objects_json.begin(); it != send_objects_json.end(); ++it)
    {
        const std::string object_name = it.key().asString();
        send_objects[object_name] = {};
        for (const auto& attr : *it)
        {
            const std::string attribute_name = attr.asString();
            send_objects[object_name][attribute_name] = attribute_map[attribute_name];
            for (double &value : send_objects[object_name][attribute_name])
            {
                send_data_vec.push_back(&value);
            }
        }
    }
    mtx.unlock();

    Json::Value receive_objects_json = json_header["receive"];
    std::vector<double*> receive_data_vec;
    
    for (auto it = receive_objects_json.begin(); it != receive_objects_json.end(); ++it)
    {
        const std::string object_name = it.key().asString();
        while (send_objects.count(object_name) == 0)
        {
            zmq_sleep(0.1);
        }
        
        for (const auto& attr : *it)
        {
            const std::string attribute_name = attr.asString();
            for (double &value : send_objects[object_name][attribute_name])
            {
                receive_data_vec.push_back(&value);
            }
        }
    }

    zmq::socket_t socket_data{context, zmq::socket_type::rep};
    const std::string addr = "tcp://127.0.0.1:" + std::to_string(port_data + thread_num);
    socket_data.bind(addr);

    const size_t send_data_size = 1 + send_data_vec.size();
    double send_buffer[send_data_size];

    const size_t receive_data_size = 1 + receive_data_vec.size();
    double receive_buffer[receive_data_size];

    while (ros::ok()) 
    {
        zmq::message_t request(sizeof(send_buffer));
        socket_data.recv(request);
        memcpy(&send_buffer, request.data(), sizeof(send_buffer));

        for (size_t i = 0; i < send_data_size - 1; i++)
        {
            *send_data_vec[i] = send_buffer[i+1];
        }

        receive_buffer[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        for (size_t i = 1; i < receive_data_size; i++)
        {
            receive_buffer[i] = *receive_data_vec[i-1];
        }
        
        zmq::message_t reply(sizeof(receive_buffer));
        memcpy(reply.data(), &receive_buffer, sizeof(receive_buffer));
        socket_data.send(reply, zmq::send_flags::none);
    }

    socket_data.unbind(addr);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "state_server");

    if (argc > 2)
    {
        port_header = std::stoi(argv[1]);
        port_data = std::stoi(argv[2]);
    }

    std::vector<std::string> socket_addrs;
    std::vector<zmq::socket_t> socket_headers;
    std::vector<std::thread> workers;

    for (size_t i = 0;; i++)
    {
        socket_headers.push_back(zmq::socket_t(context, zmq::socket_type::rep));
        socket_addrs.push_back("tcp://127.0.0.1:" + std::to_string(port_header + i));
        socket_headers[i].bind(socket_addrs[i]);
        
        zmq::message_t reply_header;
        socket_headers[i].recv(reply_header);
    
        Json::Value json_header;
        Json::Reader reader;
        reader.parse(reply_header.to_string(), json_header);
        std::cout << json_header.toStyledString() << std::endl;
        
        workers.push_back(std::thread(worker, i, json_header));
    }

    for (std::thread& worker : workers) 
    {
        worker.join();
    }

    for (size_t i = 0; i < socket_addrs.size(); i++) 
    {
        socket_headers[i].unbind(socket_addrs[i]);
    }

    return 0;
}
