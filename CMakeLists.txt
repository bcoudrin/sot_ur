cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/base.cmake)
include(cmake/python.cmake)
if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
  set(CMAKE_INSTALL_LIBDIR lib)
endif()
include(cmake/GNUInstallDirs.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
ADD_REQUIRED_DEPENDENCY("jrl-mal")
ADD_REQUIRED_DEPENDENCY("dynamic-graph")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ADD_REQUIRED_DEPENDENCY("sot-core")
ADD_REQUIRED_DEPENDENCY("jrl-dynamics-urdf >= 2.0.1-14")

# This is required by the Python prologue.
ADD_REQUIRED_DEPENDENCY("sot-dynamic")


# Handle rpath necessary to handle ROS multiplace packages
# libraries inclusion
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Python
INCLUDE(cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
LINK_DIRECTORIES(${PYTHON_LIBRARY_DIRS})

set(SOURCES
    src/ur_device.cpp
    src/ur_sot_controller.cpp
    src/ur_controller_plugin.cpp)

set(HEADERS
    include/sot_ur/ur_device.h
    include/sot_ur/ur_sot_controller.h
    include/sot_ur/ur_controller_plugin.h)

rosbuild_add_library(sot_ur ${HEADERS} ${SOURCES})

PKG_CONFIG_USE_DEPENDENCY(sot_ur "dynamic-graph")
PKG_CONFIG_USE_DEPENDENCY(sot_ur "sot-core")
PKG_CONFIG_USE_DEPENDENCY(sot_ur "dynamic-graph-python")
PKG_CONFIG_USE_DEPENDENCY(sot_ur "jrl-dynamics-urdf")
install(TARGETS sot_ur DESTINATION ${CMAKE_INSTALL_LIBDIR})

ADD_SUBDIRECTORY(src)

