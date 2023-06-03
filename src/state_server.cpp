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

int port = 7500;

std::vector<std::string> socket_addrs;
std::vector<std::thread> workers;

zmq::context_t context{1};

std::mutex mtx;

std::vector<zmq::socket_t> socket_servers;

std::map<std::string, std::map<std::string, std::vector<double>>> send_objects;

void run_socket_server(const size_t thread_num);

void communicate(const size_t thread_num, Json::Value json_header)
{
    std::vector<double *> send_data_vec;
    std::vector<double *> receive_data_vec;

receive_header:

    Json::Value send_objects_json = json_header["send"];

    for (auto it = send_objects_json.begin(); it != send_objects_json.end(); ++it)
    {
        const std::string object_name = it.key().asString();
        mtx.lock();
        send_objects[object_name] = {};
        for (const auto &attr : *it)
        {
            const std::string attribute_name = attr.asString();
            send_objects[object_name][attribute_name] = attribute_map[attribute_name];
            for (double &value : send_objects[object_name][attribute_name])
            {
                send_data_vec.push_back(&value);
            }
        }
        mtx.unlock();
    }

    Json::Value receive_objects_json = json_header["receive"];
    
    for (auto it = receive_objects_json.begin(); it != receive_objects_json.end(); ++it)
    {
        const std::string object_name = it.key().asString();
        while (send_objects.count(object_name) == 0)
        {
            zmq_sleep(0.1);
        }

        for (const auto &attr : *it)
        {
            const std::string attribute_name = attr.asString();
            mtx.lock();
            for (double &value : send_objects[object_name][attribute_name])
            {
                receive_data_vec.push_back(&value);
            }
            mtx.unlock();
        }
    }

    const size_t send_buffer_size = 1 + send_data_vec.size();
    const size_t receive_buffer_size = 1 + receive_data_vec.size();

    // Send buffer sizes over ZMQ
    size_t buffer[2] = {send_buffer_size, receive_buffer_size};
    zmq::message_t reply_header(sizeof(buffer));
    memcpy(reply_header.data(), buffer, sizeof(buffer));
    socket_servers[thread_num].send(reply_header, zmq::send_flags::none);

    double *send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
	double *receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));

    while (ros::ok())
    {
        // Receive send_data over ZMQ
        zmq::message_t request(send_buffer_size);
        socket_servers[thread_num].recv(request);
        
        if (request.to_string()[0] == '{')
        {
            Json::Reader reader;
            reader.parse(request.to_string(), json_header);
            std::cout << json_header.toStyledString() << std::endl;

            send_data_vec.clear();
            receive_data_vec.clear();

            goto receive_header;
        }
        
        memcpy(send_buffer, request.data(), send_buffer_size * sizeof(double));
        
        const double delay_ms = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() - *send_buffer) / 1000.0;
        
        for (size_t i = 0; i < send_buffer_size - 1; i++)
        {
            *send_data_vec[i] = send_buffer[i + 1];
        }

        *receive_buffer = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        for (size_t i = 0; i < receive_buffer_size - 1; i++)
        {
            receive_buffer[i + 1] = *receive_data_vec[i];
        }

        // Send receive_data over ZMQ
        zmq::message_t reply(receive_buffer_size * sizeof(double));
        memcpy(reply.data(), receive_buffer, receive_buffer_size * sizeof(double));
        socket_servers[thread_num].send(reply, zmq::send_flags::none);
    }

    free(send_buffer);
    free(receive_buffer);
}

void run_socket_server(const size_t thread_num)
{
    // Receive JSON string over ZMQ
    zmq::message_t request_header;
    socket_servers[thread_num].recv(request_header);

    Json::Value json_header;
    Json::Reader reader;
    reader.parse(request_header.to_string(), json_header);
    std::cout << json_header.toStyledString() << std::endl;

    workers[thread_num] = std::thread(communicate, thread_num, json_header);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_server");

    if (argc > 1)
    {
        port = std::stoi(argv[1]);
    }

    for (size_t thread_num = 0;; thread_num++)
    {
        socket_servers.push_back(zmq::socket_t(context, zmq::socket_type::rep));
        socket_addrs.push_back("tcp://127.0.0.1:" + std::to_string(port + thread_num));
        socket_servers[thread_num].bind(socket_addrs[thread_num]);
        workers.push_back(std::thread());
        run_socket_server(thread_num);
    }

    for (std::thread &worker : workers)
    {
        worker.join();
    }

    for (size_t i = 0; i < socket_addrs.size(); i++)
    {
        socket_servers[i].unbind(socket_addrs[i]);
        socket_servers[i].close();
    }

    return 0;
}
