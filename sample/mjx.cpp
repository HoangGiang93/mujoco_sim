/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


#include "mex.h"
#include "mujoco.h"
#include "mjxmacro.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"

#include <mutex>
#include <thread>

using namespace std;


//--------------------------- basic.cpp with thread/mutex -------------------------------

// background thread
thread background_thread;
thread::id background_id;
bool initialized = false;
bool exitrequest = false;

// main data structures: protected with mutex
mutex mtx;
mjModel* m = NULL;
mjData* d = NULL;

// visualization data structures
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // only on press
    if( act!=GLFW_PRESS )
        return;

    // toggle visualization flag
    for( int i=0; i<mjNVISFLAG; i++ )
        if( key==mjVISSTRING[i][2][0] )
            opt.flags[i] = !opt.flags[i];

    // toggle rendering flag
    for( int i=0; i<mjNRNDFLAG; i++ )
        if( key==mjRNDSTRING[i][2][0] )
            scn.flags[i] = !scn.flags[i];

    // toggle geom/site group
    for( int i=0; i<mjNGROUP; i++ )
        if( key==i+'0')
        {
            if( mods & GLFW_MOD_SHIFT )
                opt.sitegroup[i] = !opt.sitegroup[i];
            else
                opt.geomgroup[i] = !opt.geomgroup[i];
        }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    lock_guard<mutex> guard(mtx);
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    lock_guard<mutex> guard(mtx);
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// background thread: render, process events
void background(GLFWwindow* window)
{
    // save thread id
    background_id = this_thread::get_id();

    // associate OpenGL context with this thread, enable v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context, set default camera view
    {
        lock_guard<mutex> guard(mtx);
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        // scale view
        cam.lookat[0] = m->stat.center[0];
        cam.lookat[1] = m->stat.center[1];
        cam.lookat[2] = m->stat.center[2];
        cam.distance = 1.5 * m->stat.extent;
    }

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // run until asked to exit (ignore window close button)
    while( !exitrequest )
    {
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene
        {
            lock_guard<mutex> guard(mtx);
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        }

        // render
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // uninstall GLFW callbacks
    glfwSetKeyCallback(window, NULL);
    glfwSetCursorPosCallback(window, NULL);
    glfwSetMouseButtonCallback(window, NULL);
    glfwSetScrollCallback(window, NULL);

    // free visualization data structures
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
}


//--------------------------- MATLAB-specific code --------------------------------------

// error
void mju_MATLAB_error(const char* msg)
{
    mexErrMsgTxt(msg);
}


// warning
void mju_MATLAB_warning(const char* msg)
{
    // ignore warnings from background thread because MEX API is not thread-safe
    if( background_id==this_thread::get_id() )
        return;

    mexWarnMsgTxt(msg);
}


// allocate memory with mxMalloc, make persistent
void* mju_MATLAB_malloc(size_t sz)
{
    void* ptr = mxMalloc(sz);
    mexMakeMemoryPersistent(ptr);
    return ptr;
}


// free memory with mxFree
void mju_MATLAB_free(void* ptr)
{
    mxFree(ptr);
}


// exit function
void exitFunction(void)
{
    if( initialized )
    {
        // wait for thread to exit
        exitrequest = true;
        background_thread.join();

        // delete model and data, deactivate
        mj_deleteData(d);
        mj_deleteModel(m);
        mj_deactivate();

        // close GLFW
        #ifdef _WIN32
            glfwTerminate();        // causes crash with Linux NVidia drivers
        #endif

        // clear globals
        m = NULL;
        d = NULL;
        initialized = false;
        exitrequest = false;

        // unlock mex so MATLAB can remove it from memory
        mexUnlock();
    }
}


// find mjData field with given name, return pointer and dimensions
double* findfield(const char* name, int* nr, int* nc)
{
    // prepare constants for NC
    int nv = m->nv;
    int njmax = m->njmax;

    // find field
    #define X(TYPE, NAME, NR, NC)                               \
        if( !strcmp(#NAME, name) && !strcmp(#TYPE, "mjtNum") )  \
        {                                                       \
            *nr = m->NR;                                        \
            *nc = NC;                                           \
            return (double*)d->NAME;                            \
        }

        MJDATA_POINTERS
    #undef X

    // not found or type not mjtNum
    return NULL;
}


// help string
static char helpstr[] = 
"USAGE:\n"
"  [result] = mjx(command, [parameters]);\n"
"EXAMPLES:\n"
"  mjx('load', '../model/humanoid.xml');\n"
"  myxpos = mjx('get', 'xpos');\n"
"  mjx('set', 'xpos', myxpos);\n"
"COMMANDS AND PARAMETERS:\n"
"  load file      : initialize and load model file\n"
"  info           : return mjData info\n"
"  get field      : return mjData numeric field\n"
"  set field data : set mjData numeric field to given data\n"
"  reset [key]    : call mj_reset, optional keyframe to reset to\n"
"  step [number]  : call mj_step, optional number of time steps\n"
"  forward        : call mj_forward\n"
"  inverse        : call mj_inverse\n"
"  exit or quit   : terminate\n\n";


// entry point
void mexFunction(int nout, mxArray* pout[], int nin, const mxArray* pin[])
{
    char filename[100], command[100], fieldname[100];

    // no inputs: print help, return
    if( !nin )
    {
        mexPrintf(helpstr);
        return;
    }

    // get command string
    if( mxGetClassID(pin[0])!=mxCHAR_CLASS )
    {
        mexPrintf("first argument must be command string\n");
        return;
    }
    mxGetString(pin[0], command, 100);

    //---------------------------- initialize and load model file
    if( !strcmp(command, "load") )
    {
        // check for repeated initialization
        if( initialized )
           mexErrMsgTxt("already initialized");

        // get filename
        if( nin<2 || mxGetClassID(pin[1])!=mxCHAR_CLASS )
           mexErrMsgTxt("second argument must be filename");
        mxGetString(pin[1], filename, 100);

        // set MATLAB handlers
        mju_user_error = mju_MATLAB_error;
        mju_user_warning = mju_MATLAB_warning;
        mju_user_malloc = mju_MATLAB_malloc;
        mju_user_free = mju_MATLAB_free;

        // activate MuJoCo
        mj_activate("mjkey.txt");

        // load and compile model
        char error[1000] = "Could not load binary model";
        if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
            m = mj_loadModel(filename, 0);
        else
            m = mj_loadXML(filename, 0, error, 1000);
        if( !m )
            mju_error_s("Load model error: %s", error);

        // make data and update
        d = mj_makeData(m);
        mj_forward(m, d);

        // init GLFW
        if( !glfwInit() )
            mju_error("Could not initialize GLFW");

        // create window
        GLFWvidmode vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());
        GLFWwindow* wnd = glfwCreateWindow(vmode.height/2, vmode.height/2, 
                                           "mjx", NULL, NULL);

        // finish initialization
        mexAtExit(exitFunction);
        mexLock();
        initialized = true;

        // start background processing
        exitrequest = false;
        background_thread = thread(background, wnd);
        return;
    }

    //---------------------------- terminate
    else if( !strcmp(command, "exit") || !strcmp(command, "quit") )
    {
        exitFunction();
        return;
    }

    // the remaining commands require initialization
    if( !initialized )
        mexErrMsgTxt("not initialized");

    //---------------------------- return mjData info
    else if( !strcmp(command, "info") )
    {
        // create MATLAB structure
        const char* names[5] = {"time", "ne", "nf", "nefc", "ncon"};
        pout[0] = mxCreateStructMatrix(1, 1, 5, names);

        // copy info from mjMdata to MATLAB structure
        lock_guard<mutex> guard(mtx);
        mxSetField(pout[0], 0, "time", mxCreateDoubleScalar((double)d->time));
        mxSetField(pout[0], 0, "ne", mxCreateDoubleScalar((double)d->ne));
        mxSetField(pout[0], 0, "nf", mxCreateDoubleScalar((double)d->nf));
        mxSetField(pout[0], 0, "nefc", mxCreateDoubleScalar((double)d->nefc));
        mxSetField(pout[0], 0, "ncon", mxCreateDoubleScalar((double)d->ncon));
    }

    //---------------------------- get/set mjData field
    else if( !strcmp(command, "get") || !strcmp(command, "set") )
    {
        lock_guard<mutex> guard(mtx);

        // get field name
        if( nin<2 || !mxIsClass(pin[1], "char") )
            mexErrMsgTxt("field name expected");
        mxGetString(pin[1], fieldname, 100);

        // find field
        int nr = 0, nc = 0;
        double* fielddata = findfield(fieldname, &nr, &nc);
        if( !fielddata )
            mexErrMsgTxt("invalid field name");

        // get
        if( !strcmp(command, "get") )
        {   
            // create MATLAB matrix and copy data (assuming mjtNum is double)
            pout[0] = mxCreateDoubleMatrix(nc, nr, mxREAL);
            memcpy(mxGetPr(pout[0]), fielddata, nr*nc*sizeof(double));
        }

        // set
        else
        {
            // require data argument
            if( nin<3 || !mxIsNumeric(pin[2]) )
                mexErrMsgTxt("field name expected");

            // check data dimensions
            const mwSize* sz = mxGetDimensions(pin[2]);
            if( mxGetNumberOfDimensions(pin[2])!=2 || sz[0]!=nc || sz[1]!=nr )
                mexErrMsgTxt("invalid data dimensions");

            // copy data (assuming mjtNum is double)
            memcpy(fielddata, mxGetPr(pin[2]), nr*nc*sizeof(double));
        }
    }

    //---------------------------- reset [key]
    else if( !strcmp(command, "reset") )
    {
        lock_guard<mutex> guard(mtx);

        // no key
        if( nin<2 )
            mj_resetData(m, d);

        // key specified
        else
        {
            // check argument, get key
            if( !mxIsNumeric(pin[1]) )
                mexErrMsgTxt("numeric optional argument expected");
            int key = mju_round(mxGetScalar(pin[1]));
            if( key<0 || key>=m->nkey )
                mexErrMsgTxt("invalid key");

            // reset to specified key
            mj_resetDataKeyframe(m, d, key);
        }

        // update rendering
        mj_forward(m, d);
    }

    //---------------------------- step [number]
    else if( !strcmp(command, "step") )
    {
        lock_guard<mutex> guard(mtx);

        // no number
        if( nin<2 )
            mj_step(m, d);

        // number of steps specified
        else
        {
            // check argument, get number of steps
            if( !mxIsNumeric(pin[1]) )
                mexErrMsgTxt("numeric optional argument expected");
            int number = mju_round(mxGetScalar(pin[1]));
            if( number<0 )
                mexErrMsgTxt("invalid nunber");

            // run for specified number of steps
            for( int i=0; i<number; i++ )
                mj_step(m, d);
        }
    }

    //---------------------------- forward
    else if( !strcmp(command, "forward") )
    {
        lock_guard<mutex> guard(mtx);
        mj_forward(m, d);
    }

    //---------------------------- inverse
    else if( !strcmp(command, "inverse") )
    {
        lock_guard<mutex> guard(mtx);
        mj_inverse(m, d);
    }

    // undefined command
    else
        mexErrMsgTxt("undefined command string");
}
