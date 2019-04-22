cmake_minimum_required(VERSION 2.8.3)
project(team_nust_visualizer)

include($ENV{PATH_TO_TEAM_NUST_DIR}/resources/team_nust_nao_interface_ws/src/cmake/common.cmake)

find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   geometry_msgs
   humanoid_nav_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS launch
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
