# Install script for directory: /home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_visualizer_plugins

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer_plugins/catkin_generated/installspace/team_nust_visualizer_plugins.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_visualizer_plugins/cmake" TYPE FILE FILES
    "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer_plugins/catkin_generated/installspace/team_nust_visualizer_pluginsConfig.cmake"
    "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/build/team_nust_visualizer_plugins/catkin_generated/installspace/team_nust_visualizer_pluginsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_visualizer_plugins" TYPE FILE FILES "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_visualizer_plugins/package.xml")
endif()

