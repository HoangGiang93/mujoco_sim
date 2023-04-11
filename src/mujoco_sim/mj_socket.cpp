// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

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
#include "mj_model.h"

#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <ctime>

#include <jsoncpp/json/json.h>

#include <thread>

sockaddr_un addr;

char buffer[BUFFER_SIZE];

std::string socket_name = SOCKET_NAME;

int connection_socket = 0;

fd_set rfds;
int monitored_fds[MAX_CLIENT_SUPPORTED] = {0};

std::vector<std::thread> threads;

void send_msg_thread(int data_socket)
{
    Json::Value data_json;
    data_json["header"]["seq"] = 0;
    data_json["header"]["frame_id"] = "mujoco";
    while (ros::ok())
    {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        long int now_int = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        int now_nsecs = now_int % 1000000000;
        int now_secs = now_int / 1000000000;
        data_json["header"]["seq"] = data_json["header"]["seq"].asInt() + 1;
        data_json["header"]["stamp"]["secs"] = now_secs;
        data_json["header"]["stamp"]["nsecs"] = now_nsecs;

        for (int i = 0; i < (m->nbody / 10) + 1; i++)
        {
            data_json["body"].clear();
            for (int body_id = i * 10; body_id < (i + 1) * 10 && body_id < m->nbody; body_id++)
            {
                const char *name =  mj_id2name(m, mjOBJ_BODY, body_id);
                data_json["body"][name]["position"].append(d->xpos[3 * body_id]);
                data_json["body"][name]["position"].append(d->xpos[3 * body_id + 1]);
                data_json["body"][name]["position"].append(d->xpos[3 * body_id + 2]);
                data_json["body"][name]["quaternion"].append(d->xquat[4 * body_id]);
                data_json["body"][name]["quaternion"].append(d->xquat[4 * body_id + 1]);
                data_json["body"][name]["quaternion"].append(d->xquat[4 * body_id + 2]);
                data_json["body"][name]["quaternion"].append(d->xquat[4 * body_id + 3]);
            }

            std::string data_str = data_json.toStyledString();
            if (data_str.size() > BUFFER_SIZE)
            {
                ROS_WARN("Data too big (%ld byte)", data_str.size());
            }
            
            if (write(data_socket, data_str.data(), BUFFER_SIZE) == -1)
            {
                return;
            }
        }
        usleep(1000000);
    }
}

void start_socket()
{
    socket_name += "joint_state";
    unlink(socket_name.c_str());

    /* Create Master socket. */
    connection_socket = socket(AF_UNIX, SOCK_STREAM, 0);
    if (connection_socket == -1)
    {
        ROS_ERROR("socket() call failed");
        return;
    }
    ROS_INFO("Master socket %d created", connection_socket);

    /* Specify the socket Credentials */
    memset(&addr, 0, sizeof(struct sockaddr_un)); // Initializes the data to 0
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_name.c_str(), sizeof(addr.sun_path) - 1);

    /* Bind socket to socket name.*/
    if (bind(connection_socket, (const struct sockaddr *)&addr,
             sizeof(struct sockaddr_un)) == -1)
    {
        ROS_ERROR("bind() call failed");
        return;
    }
    ROS_INFO("bind() call succeed");

    if (listen(connection_socket, 10) == -1)
    {
        ROS_ERROR("listen() call failed");
        return;
    }
    ROS_INFO("listen() call succeed");

    for (int &monitored_fd : monitored_fds)
    {
        monitored_fd = -1;
    }

    monitored_fds[0] = connection_socket;
}

void run_socket()
{
    /* This is the main loop for handling connections. */
    while (ros::ok())
    {
        FD_ZERO(&rfds);
        for (int monitored_fd : monitored_fds)
        {
            if (monitored_fd != -1)
            {
                FD_SET(monitored_fd, &rfds);
            }
        }

        timeval time_out{1, 0};
        int ret = select(FD_SETSIZE, &rfds, NULL, NULL, &time_out);
        if (ret == -1)
        {
            ROS_ERROR("select() call failed");
            return;
        }

        if (ret == 0)
        {
            continue;
        }

        if (FD_ISSET(connection_socket, &rfds))
        {
            int data_socket = accept(connection_socket, NULL, NULL);
            if (data_socket == -1)
            {
                ROS_ERROR("accept() call failed");
                return;
            }

            for (int &monitored_fd : monitored_fds)
            {
                if (monitored_fd == -1)
                {
                    monitored_fd = data_socket;
                    break;
                }
            }

            threads.push_back(std::thread(&send_msg_thread, data_socket));
            ROS_INFO("Open new connection, number of connections: %ld", threads.size());
        }
        else
        {
            /* Find the client which has send the data request */
            for (int i = 1; i < MAX_CLIENT_SUPPORTED; i++)
            {
                if (FD_ISSET(monitored_fds[i], &rfds))
                {
                    close(monitored_fds[i]);
                    monitored_fds[i] = -1;
                    threads[i - 1].join();
                    threads.erase(threads.begin() + i - 1);
                    ROS_INFO("Close connection %d, number of connections: %ld", i, threads.size());
                }
            }
        }
    }
}

void close_socket()
{
    for (size_t i = 0; i < threads.size(); i++)
    {
        threads[i].join();
        threads.erase(threads.begin() + i);
        printf("Close connection %ld, number of connections: %ld\n", i, threads.size());
    }

    /*close the master socket*/
    close(connection_socket);
    printf("Connection closed..\n");

    /* Server should release resources before getting terminated. */
    unlink(socket_name.c_str());
}