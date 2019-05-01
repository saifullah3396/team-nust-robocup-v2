cmake_minimum_required(VERSION 2.8.3)
project(team_nust_network_handler)

include($ENV{PATH_TO_TEAM_NUST_DIR}/resources/team_nust_nao_interface_ws/src/cmake/common.cmake)

add_compile_options(-std=c++11 -DVISUALIZER_BUILD -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")
find_package(qibuild)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   humanoid_nav_msgs
)

if(${ROBOT_VERSION} STREQUAL "V6")
  set(Boost_LIBRARIES
    $ENV{PATH_TO_TEAM_NUST_DIR}/resources/boost_1_59_0/stage/lib/libboost_system.so 
    $ENV{PATH_TO_TEAM_NUST_DIR}/resources/boost_1_59_0/stage/lib/libboost_thread.so 
    $ENV{PATH_TO_TEAM_NUST_DIR}/resources/boost_1_59_0/stage/lib/libboost_serialization.so 
    $ENV{PATH_TO_TEAM_NUST_DIR}/resources/boost_1_59_0/stage/lib/libboost_iostreams.so)

  set(Boost_INCLUDE_DIRS $ENV{PATH_TO_TEAM_NUST_DIR}/resources/boost_1_59_0/)

  link_directories(${PATH_TO_TEAM_NUST_DIR}/resources/boost_1_59_0/stage/lib/)
else()
  find_package(Boost REQUIRED COMPONENTS thread system serialization iostreams)
endif()

MESSAGE(STATUS ${Boost_LIBRARIES})

catkin_python_setup()
catkin_package(
  INCLUDE_DIRS launch
)

set (TNRS_UTILS_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/src/Utils")
set (JSON_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/resources/jsoncpp")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  $ENV{PATH_TO_TEAM_NUST_DIR}/src/
  $ENV{PATH_TO_TEAM_NUST_DIR}/src/TNRSModules/
  $ENV{PATH_TO_TEAM_NUST_DIR}/resources/
  $ENV{PATH_TO_TEAM_NUST_DIR}/resources/jsoncpp/include/
)

file (GLOB TNRS_UTILS_SRC
  "${TNRS_UTILS_DIR}/src/ConfigManager.cpp"
  "${TNRS_UTILS_DIR}/src/PrintUtils.cpp"
  "${TNRS_UTILS_DIR}/src/DataUtils.cpp"
)

file (GLOB JSON_SRC
  "${JSON_DIR}/src/lib_json/*.cpp"
)

add_executable(
  team_nust_network_handler_node
  src/tcp_client.cpp
  src/network_handler.cpp
  src/network_handler_node.cpp
  ${TNRS_UTILS_SRC}
  ${JSON_SRC})

add_dependencies(team_nust_network_handler_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(team_nust_network_handler_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
