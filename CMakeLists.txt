cmake_minimum_required(VERSION 3.10)
project(mujoco_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mujoco_msgs
  roslib
  controller_manager
  tf2_ros
  urdf
)

find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set output file
    set(DOXYGEN_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

set(MUJOCO mujoco)

add_custom_command(OUTPUT ${MUJOCO}_build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND $(MAKE) mujoco
  VERBATIM
)
add_custom_target(${MUJOCO} DEPENDS ${MUJOCO}_build)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/mujoco_sim
  # LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
  # DEPENDS 
)

###########
## Build ##
###########

set(MUJOCO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/build/mujoco-2.3.5)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/mujoco_sim
  ${MUJOCO_SOURCE_DIR}/include
)

set(MUJOCO_COMPILE_NODE mujoco_compile_node)
add_executable(${MUJOCO_COMPILE_NODE} src/mujoco_compile.cpp)
add_dependencies(${MUJOCO_COMPILE_NODE} ${MUJOCO})
target_link_libraries(${MUJOCO_COMPILE_NODE}
  ${catkin_LIBRARIES}
  ${MUJOCO_SOURCE_DIR}/lib/libmujoco.so
  tinyxml2
)

set(MUJOCO_SIM_HEADLESS_NODE mujoco_sim_headless_node)
add_executable(${MUJOCO_SIM_HEADLESS_NODE} src/mujoco_sim_headless.cpp)
add_dependencies(${MUJOCO_SIM_HEADLESS_NODE} ${MUJOCO})
add_library(${MUJOCO_SIM_HEADLESS_NODE}_lib
  ${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_hw_interface.cpp 
  ${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_ros.cpp
  ${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_model.cpp
  ${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_sim.cpp
  ${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_socket.cpp
)
add_dependencies(${MUJOCO_SIM_HEADLESS_NODE}_lib ${MUJOCO})
target_link_libraries(${MUJOCO_SIM_HEADLESS_NODE}
  ${catkin_LIBRARIES}
  ${MUJOCO_SIM_HEADLESS_NODE}_lib
  ${MUJOCO_SOURCE_DIR}/lib/libmujoco.so
  $ENV{ROS_ROOT}/../../lib/liburdf.so
  tinyxml2
  zmq
  jsoncpp
)

set(MUJOCO_SIM_NODE mujoco_sim_node)
add_executable(${MUJOCO_SIM_NODE} src/mujoco_sim.cpp)
add_dependencies(${MUJOCO_SIM_NODE} ${MUJOCO})
add_library(${MUJOCO_SIM_NODE}_lib
  ${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_visual.cpp
)
add_dependencies(${MUJOCO_SIM_NODE}_lib ${MUJOCO})
target_link_libraries(${MUJOCO_SIM_NODE}
  ${catkin_LIBRARIES}
  ${MUJOCO_SIM_HEADLESS_NODE}_lib
  ${MUJOCO_SIM_NODE}_lib
  ${MUJOCO_SOURCE_DIR}/lib/libmujoco.so
  $ENV{ROS_ROOT}/../../lib/liburdf.so
  tinyxml2
  zmq
  jsoncpp
  glfw
  GL
)

add_executable(state_server_node src/state_server.cpp)
target_link_libraries(state_server_node
  ${catkin_LIBRARIES}
  zmq
  jsoncpp
)

catkin_install_python(PROGRAMS script/mujoco_to_usd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})