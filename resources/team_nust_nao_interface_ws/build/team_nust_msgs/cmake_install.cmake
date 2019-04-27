# Install script for directory: /home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_msgs/msg" TYPE FILE FILES
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_msgs/cmake" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_msgs/catkin_generated/installspace/team_nust_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/devel/include/team_nust_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/devel/share/common-lisp/ros/team_nust_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/devel/lib/python2.7/dist-packages/team_nust_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/devel/lib/python2.7/dist-packages/team_nust_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_msgs/catkin_generated/installspace/team_nust_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_msgs/cmake" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_msgs/catkin_generated/installspace/team_nust_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_msgs/cmake" TYPE FILE FILES
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_msgs/catkin_generated/installspace/team_nust_msgsConfig.cmake"
    "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/build/team_nust_msgs/catkin_generated/installspace/team_nust_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team_nust_msgs" TYPE FILE FILES "/home/muptii/Documents/robocup/team-nust-robocup-v2/resources/team_nust_nao_interface_ws/src/team_nust_msgs/package.xml")
endif()

