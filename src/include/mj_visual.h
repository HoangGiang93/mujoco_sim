#pragma once

#include "mj_sim.h"

#include <glfw3.h>

class MjVisual
{
public:
    MjVisual() = default;

    ~MjVisual();

public:
    void init();

    // mouse button callback
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);

    // mouse move callback
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);

    // scroll callback
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    GLFWwindow *init_visualization();

    bool is_window_closed();

    void render(double sim_time, double ros_time);

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