// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include <tinyxml.h>

#include "glfw3.h"
#include "mujoco.h"

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
int idx = 0;

void load_data(const char *filename)
{
  // load and compile model
  char error[1000] = "Could not load binary model";

  m = mj_loadXML(filename, 0, error, 1000);
  if (!m)
  {
    mju_error_s("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);
}

void add_data(const char *filename)
{
  TiXmlDocument doc("../model/test/current.xml");
	if (doc.LoadFile())
	{
		TiXmlElement* pElem = doc.FirstChildElement()->FirstChildElement();

    while (strcmp(pElem->Value(), "worldbody") != 0 && pElem != nullptr)
    {
      pElem = pElem->NextSiblingElement();
    }

    TiXmlElement* body = pElem->FirstChildElement();
    
    while (body != nullptr)
    {
      if (strcmp(body->Value(), "body") == 0)
      {
        TiXmlAttribute* name = body->FirstAttribute();
        while (name != nullptr)
        {
          if (strcmp(name->Name(), "name") == 0)
          {
            printf("%s\n", name->Value());
            name->SetValue(std::string(name->Value()) + "_" + std::to_string(idx++));
            break;
          }
          name = name->Next();
        }
      }
      body = body->NextSiblingElement();
    }
    doc.SaveFile("../model/test/test.xml");
	}
	else
	{
		printf("Failed to load file \"%s\"\n", "../model/test/current.xml");
	}

  // FILE *fptr;
  // int i = 0;
  // char buff[255];
  // char content[10000];



  // fptr = fopen("../model/test/current.xml", "r");
  
  // if (fgets(buff, 255, fptr))
  // {
  //   strcat(content, buff);
  // }
  // strcat(content, "\t<include file=\"");
  // strcat(content, filename);
  // strcat(content, "\"/>\n");
  // while (fgets(buff, 255, fptr))
  // {
  //   strcat(content, buff);
  // }
  // fclose(fptr);

  // fptr = fopen("../model/test/new.xml", "w");
  // fprintf(fptr, "%s", content);
  // fclose(fptr);

  // load and compile model
  // char error[1000] = "Could not load binary model";
  // mjModel*m_new = mj_loadXML("../model/test/new.xml", 0, error, 1000);
  // if (!m_new) {
  //   mju_error_s("Load model error: %s", error);
  // }

  // // make data
  // mjData *d_new = mj_makeData(m_new);
  // d_new->time = d->time;

  // mju_copy(d_new->qpos, d->qpos, m->nq);

  // mju_copy(d_new->qvel, d->qvel, m->nv);
  // mju_copy(d_new->qacc_warmstart, d->qacc_warmstart, m->nv);
  // mju_copy(d_new->qfrc_applied, d->qfrc_applied, m->nv);
  // mju_copy(d_new->qacc, d->qacc, m->nv);

  // mju_copy(d_new->act, d->act, m->na);
  // mju_copy(d_new->act_dot, d->act_dot, m->na);
  // mju_copy(d_new->qfrc_applied, d->qfrc_applied, m->na);
  // mju_copy(d_new->qacc, d->qacc, m->na);
  
  // mju_copy(d_new->xfrc_applied, d->xfrc_applied, m->nbody * 6);
  
  // mju_copy(d_new->mocap_pos,  d->mocap_pos,  3*m->nmocap);
  // mju_copy(d_new->mocap_quat, d->mocap_quat, 4*m->nmocap);

  // mju_copy(d_new->userdata, d->userdata, m->nuserdata);

  // mju_copy(d_new->sensordata, d->sensordata, m->nsensordata);
  
  // d = d_new;
  // m = mj_copyModel(NULL, m_new);
}

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    char error[1000] = "Could not load binary model";
    mj_saveLastXML("../model/test/current.xml", m, error, 1000);
    add_data("ball.xml");
  }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// main function
int main(int argc, const char **argv)
{
  load_data("../model/test/hello.xml");

  // init GLFW
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window))
  {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0)
    {
      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
