# Copyright: (C) 2022 Istituto Italiano di Tecnologia | Robotics, Brains and Cognitive Science
# Authors: Letícia Berto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)


SET(KEYWORD "motivatedAutonomousAgent")
PROJECT(${KEYWORD})

set(CMAKE_CXX_STANDARD 11)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})


#search for includes
INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
        ${YARP_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH
        ${CMAKE_MODULE_PATH}
        ${YARP_MODULE_PATH}
        ${ICUB_MODULE_PATH}
        ${ICUBCONTRIB_MODULE_PATH}
        )

include(ICUBcontribHelpers)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix() #set CMAKE_INSTALL_PREFIX to the ICUBcontrib one; print warning if user modifies it

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_definitions(${YARP_DEFINES})
include(YarpInstallationHelpers)

set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")


add_subdirectory(src)
add_subdirectory(app)

icubcontrib_add_uninstall_target()   