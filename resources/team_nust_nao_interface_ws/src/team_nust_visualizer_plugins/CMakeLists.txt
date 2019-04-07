cmake_minimum_required(VERSION 2.8.3)
project(team_nust_visualizer_plugins)

add_compile_options(-std=c++11 -DVISUALIZER_BUILD -DROOT_DIR="$ENV{PATH_TO_TEAM_NUST_DIR}/src")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

set (TNRS_UTILS_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/src/Utils")
set (JSON_DIR "$ENV{PATH_TO_TEAM_NUST_DIR}/resources/jsoncpp")

qt4_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/console.h
  include/${PROJECT_NAME}/tnrs_robot_state.h
  include/${PROJECT_NAME}/camera_calibration.h
  include/${PROJECT_NAME}/color_calibration.h
  include/${PROJECT_NAME}/sensors_info.h
)

file (GLOB TNRS_UTILS_SRC
  "${TNRS_UTILS_DIR}/src/ConfigManager.cpp"
  "${TNRS_UTILS_DIR}/src/PrintUtils.cpp"
  "${TNRS_UTILS_DIR}/src/DataUtils.cpp"
  "${TNRS_UTILS_DIR}/src/DataHolders/Camera.cpp"
)

file (GLOB JSON_SRC
  "${JSON_DIR}/src/lib_json/*.cpp"
)

set(SOURCE_FILES
  src/console.cpp
  src/tnrs_robot_state.cpp
  src/camera_calibration.cpp
  src/color_calibration.cpp
  src/sensors_info.cpp
  ${MOC_FILES}
  ${TNRS_UTILS_SRC}
  ${JSON_SRC}
)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs team_nust_msgs
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  $ENV{PATH_TO_TEAM_NUST_DIR}/src/
  $ENV{PATH_TO_TEAM_NUST_DIR}/src/TNRSModules/
  $ENV{PATH_TO_TEAM_NUST_DIR}/resources/
  $ENV{PATH_TO_TEAM_NUST_DIR}/resources/jsoncpp/include/
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${team_nust_msgs_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${QT_LIBRARIES} 
  libv4l1.so
  libv4l2.so
  libv4lconvert.so)
  
