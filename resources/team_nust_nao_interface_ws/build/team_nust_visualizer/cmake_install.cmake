# Install script for directory: /home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_visualizer

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/installspace/team_nust_visualizer.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_visualizer/cmake" TYPE FILE FILES
    "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/installspace/team_nust_visualizerConfig.cmake"
    "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer/catkin_generated/installspace/team_nust_visualizerConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_visualizer" TYPE FILE FILES "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_visualizer/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_visualizer" TYPE DIRECTORY FILES "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_visualizer/launch/")
endif()

