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

#pragma once

#include "mj_model.h"

#include <set>
#include <zmq.hpp>

class MjSocket
{
public:
    MjSocket(const MjSocket &) = delete;

    void operator=(MjSocket const &) = delete;

    static MjSocket &get_instance()
    {
        static MjSocket mj_socket;
        return mj_socket;
    }

public:
    /**
     * @brief Initialize the socket
     *
     */
    void init(const int port_header, const int port_data);

    /**
     * @brief Send the header
     * 
     */
    void send_header();

    /**
     * @brief Communicate with the server
     * 
     */
    void communicate();

public:
    std::vector<int> told_object_ids;

    std::vector<int> asked_object_ids;

    static std::map<std::string, std::vector<std::string>> asked_objects;

private:
    zmq::context_t context;

    std::string ip_addr = "tcp://127.0.0.1";

    zmq::socket_t socket_header;

    zmq::socket_t socket_data;

private:
    MjSocket() = default;

    ~MjSocket();
};