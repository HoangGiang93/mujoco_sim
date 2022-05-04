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

#include "mj_visual.h"

mjvCamera MjVisual::cam;  // abstract camera
mjvOption MjVisual::opt;  // visualization options
mjvScene MjVisual::scn;   // abstract scene
mjrContext MjVisual::con; // custom GPU context

bool MjVisual::button_left = false;
bool MjVisual::button_middle = false;
bool MjVisual::button_right = false;
double MjVisual::lastx = 0;
double MjVisual::lasty = 0;

MjVisual::~MjVisual()
{
    terminate();
}

void MjVisual::init()
{
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);              // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150); // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetCursorPosCallback(window, &MjVisual::mouse_move);
    glfwSetMouseButtonCallback(window, &MjVisual::mouse_button);
    glfwSetScrollCallback(window, &MjVisual::scroll);

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 1.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
}

void MjVisual::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // determine action based on mouse button
    mjtMouse action;
    if (button_middle)
        action = mjMOUSE_MOVE_H;
    else if (button_left)
        action = mjMOUSE_ROTATE_H;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / width * 2, dy / height, &scn, &cam);
}

void MjVisual::mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// scroll callback
void MjVisual::scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

bool MjVisual::is_window_closed()
{
    return glfwWindowShouldClose(window);
}

void MjVisual::render(double sim_time, double ros_time)
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // print simulation time
    mjrRect rect1 = {0, viewport.height - 50, 300, 50};
    mjrRect rect2 = {0, viewport.height - 100, 300, 50};
    mjrRect rect3 = {0, viewport.height - 150, 300, 50};
    mjrRect rect4 = {0, viewport.height - 200, 300, 50};
    std::string sim_time_text = "Simulation time: " + std::to_string(sim_time);
    std::string ros_time_text = "ROS time: " + std::to_string(ros_time);
    std::string rtf_text = "Real-time factor: " + std::to_string(rtf_actual);
    std::string time_step_text = "Time step: " + std::to_string(m->opt.timestep);

    mjr_label(rect1, 0, sim_time_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect2, 0, ros_time_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect3, 0, rtf_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);
    mjr_label(rect4, 0, time_step_text.c_str(), 1, 1, 1, 0.2, 1, 1, 1, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void MjVisual::terminate()
{
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
}
