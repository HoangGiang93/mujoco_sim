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
#include <map>
#include <thread>
#include <mutex>

#include <zmq.hpp>

using namespace std::chrono_literals;

zmq::context_t context{1};

std::mutex mtx;

struct attribute
{
    double position[3] = {0.0, 0.0, 0.0};
    double quaternion[4] = {0.0, 0.0, 0.0, 1.0};
};

std::map<std::string, attribute> published_object_data;


void worker(const size_t thread_num, const Json::Value &json_header)
{
    Json::Value published_objects = json_header["published_objects"];

    std::vector<double*> published_object_data_vec;
    mtx.lock();
    for (auto it = published_objects.begin(); it != published_objects.end(); ++it)
    {
        const std::string object_name = it.key().asString();
        published_object_data[object_name] = attribute();
        for (const auto& attr : *it)
        {
            if (attr.asString() == "position")
            {
                published_object_data_vec.push_back(&published_object_data[object_name].position[0]);
                published_object_data_vec.push_back(&published_object_data[object_name].position[1]);
                published_object_data_vec.push_back(&published_object_data[object_name].position[2]);
            }
            else if (attr.asString() == "quaternion")
            {
                published_object_data_vec.push_back(&published_object_data[object_name].quaternion[0]);
                published_object_data_vec.push_back(&published_object_data[object_name].quaternion[1]);
                published_object_data_vec.push_back(&published_object_data[object_name].quaternion[2]);
                published_object_data_vec.push_back(&published_object_data[object_name].quaternion[3]);
            }
        }
    }
    mtx.unlock();

    Json::Value subscribed_objects = json_header["subscribed_objects"];
    std::vector<double*> subscribed_object_data_vec;
    
    for (auto it = subscribed_objects.begin(); it != subscribed_objects.end(); ++it)
    {
        const std::string object_name = it.key().asString();
        while (published_object_data.count(object_name) == 0)
        {
            // std::cout << "Wait for " << object_name << std::endl;
        }
        
        for (const auto& attr : *it)
        {
            if (attr.asString() == "position")
            {
                subscribed_object_data_vec.push_back(&published_object_data[object_name].position[0]);
                subscribed_object_data_vec.push_back(&published_object_data[object_name].position[1]);
                subscribed_object_data_vec.push_back(&published_object_data[object_name].position[2]);
            }
            else if (attr.asString() == "quaternion")
            {
                subscribed_object_data_vec.push_back(&published_object_data[object_name].quaternion[0]);
                subscribed_object_data_vec.push_back(&published_object_data[object_name].quaternion[1]);
                subscribed_object_data_vec.push_back(&published_object_data[object_name].quaternion[2]);
                subscribed_object_data_vec.push_back(&published_object_data[object_name].quaternion[3]);
            }
        }
    }



    zmq::socket_t socket_data{context, zmq::socket_type::rep};
    socket_data.bind("tcp://127.0.0.1:" + std::to_string(7600 + thread_num));

    const size_t published_data_size = 1 + published_object_data_vec.size();
    double published_buffer[published_data_size];

    const size_t subscribed_data_size = 1 + subscribed_object_data_vec.size();
    double subscribed_buffer[subscribed_data_size];

    while (true) 
    {
        zmq::message_t request(sizeof(published_buffer));
        socket_data.recv(request);
        memcpy(&published_buffer, request.data(), sizeof(published_buffer));

        for (size_t i = 0; i < published_data_size - 1; i++)
        {
            *published_object_data_vec[i] = published_buffer[i+1];
        }

        subscribed_buffer[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        for (size_t i = 1; i < subscribed_data_size; i++)
        {
            subscribed_buffer[i] = *subscribed_object_data_vec[i-1];
        }
        
        zmq::message_t reply(sizeof(subscribed_buffer));
        memcpy(reply.data(), &subscribed_buffer, sizeof(subscribed_buffer));
        socket_data.send(reply, zmq::send_flags::none);
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "state_server");

    std::vector<zmq::socket_t> socket_headers;
    std::vector<std::thread> workers;

    for (size_t i = 0;; i++)
    {
        socket_headers.push_back(zmq::socket_t(context, zmq::socket_type::rep));
        socket_headers[i].bind("tcp://127.0.0.1:" + std::to_string(7500 + i));
        
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

    return 0;
}
