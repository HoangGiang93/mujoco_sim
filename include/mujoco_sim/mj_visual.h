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

#pragma once

#include "mj_sim.h"

#include <GLFW/glfw3.h>

class MjVisual
{
public:
    MjVisual() = default;

    ~MjVisual();

public:
    /**
     * @brief Initialize the window
     *
     */
    void init();

    /**
     * @brief mouse button callback
     *
     * @param window
     * @param button
     * @param act
     * @param mods
     */
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);

    /**
     * @brief mouse move callback
     *
     * @param window
     * @param xpos
     * @param ypos
     */
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);

    /**
     * @brief scroll callback
     *
     * @param window
     * @param xoffset
     * @param yoffset
     */
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    /**
     * @brief Return true if window is closed
     *
     */
    bool is_window_closed();

    /**
     * @brief Render the simulation with OpenGL
     *
     * @param sim_time
     * @param ros_time
     */
    void render(double sim_time, double ros_time);

    /**
     * @brief Free visualization storage
     *
     */
    void terminate();

public:
    GLFWwindow *window = NULL;

private:
    static mjvCamera cam;  // abstract camera
    static mjvOption opt;  // visualization options
    static mjvScene scn;   // abstract scene
    static mjrContext con; // custom GPU context

    // mouse interaction
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;
};