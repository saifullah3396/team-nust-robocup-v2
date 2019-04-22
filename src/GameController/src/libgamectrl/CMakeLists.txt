cmake_minimum_required(VERSION 2.8)
project(gamectrl)

find_package(qibuild)

include($ENV{PATH_TO_TEAM_NUST_DIR}/make/cmake/common.cmake)
include_directories(../../include)

set(_srcs
    GameCtrl.cpp
    UdpComm.h
    UdpComm.cpp)

if(${ROBOT_VERSION} STREQUAL "V6")
add_executable(gamectrl ${_srcs})
qi_use_lib(gamectrl QI BOOST)
else()
add_library(gamectrl SHARED ${_srcs})
qi_use_lib(gamectrl ALCOMMON BOOST)
endif()
