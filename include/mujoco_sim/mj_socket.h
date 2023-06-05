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
    void init(const int port);

    /**
     * @brief Send the header
     * 
     * @return true if success
     * @return false if failed
     */
    bool send_header();

    /**
     * @brief Communicate with the server
     * 
     */
    void communicate();

public:
    static std::map<std::string, std::vector<std::string>> send_objects;

    static std::map<std::string, std::vector<std::string>> receive_objects;

    static bool enable;

private:
    std::vector<mjtNum *> send_data_vec;

    std::vector<mjtNum *> receive_data_vec;

    void* context;

	void *socket_client;

	size_t send_buffer_size = 1;

	size_t receive_buffer_size = 1;

    double *send_buffer;
    
	double *receive_buffer;

    std::string socket_client_addr;

private:
    MjSocket() = default;

    ~MjSocket();
};