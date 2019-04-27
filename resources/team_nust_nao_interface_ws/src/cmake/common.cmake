cmake_minimum_required(VERSION 2.8.12)

####################################
# Options
####################################
SET (ROBOT_VERSION "V5" CACHE STRING "Robot version")

MESSAGE(STATUS "ROBOT VERSION: " ${ROBOT_VERSION})

# Linking for V6 Robots
if(${ROBOT_VERSION} STREQUAL "V6")
  add_definitions(-DV6_CROSS_BUILD) #allow sse instructions
endif()
