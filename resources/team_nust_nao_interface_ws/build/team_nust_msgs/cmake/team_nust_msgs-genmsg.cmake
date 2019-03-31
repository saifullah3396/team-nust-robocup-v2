# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "team_nust_msgs: 14 messages, 0 services")

set(MSG_I_FLAGS "-Iteam_nust_msgs:/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Ihumanoid_nav_msgs:/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(team_nust_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg" "geometry_msgs/Pose2D:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg" "geometry_msgs/Point:geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg" "geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Quaternion:geometry_msgs/Pose2D:geometry_msgs/Transform:std_msgs/Header"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg" "geometry_msgs/Point:team_nust_msgs/TeamRobot:std_msgs/Header:geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg" "std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg" "humanoid_nav_msgs/StepTarget:std_msgs/Header:geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg" "geometry_msgs/Point:std_msgs/Header:team_nust_msgs/Landmark"
)

get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg" NAME_WE)
add_custom_target(_team_nust_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team_nust_msgs" "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg" "team_nust_msgs/Obstacle:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg/StepTarget.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_cpp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(team_nust_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(team_nust_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(team_nust_msgs_generate_messages team_nust_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_cpp _team_nust_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team_nust_msgs_gencpp)
add_dependencies(team_nust_msgs_gencpp team_nust_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team_nust_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg/StepTarget.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_eus(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(team_nust_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(team_nust_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(team_nust_msgs_generate_messages team_nust_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_eus _team_nust_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team_nust_msgs_geneus)
add_dependencies(team_nust_msgs_geneus team_nust_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team_nust_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg/StepTarget.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_lisp(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(team_nust_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(team_nust_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(team_nust_msgs_generate_messages team_nust_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_lisp _team_nust_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team_nust_msgs_genlisp)
add_dependencies(team_nust_msgs_genlisp team_nust_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team_nust_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg/StepTarget.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)
_generate_msg_py(team_nust_msgs
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(team_nust_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(team_nust_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(team_nust_msgs_generate_messages team_nust_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/GoalInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Landmark.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamRobot.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/JointInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/LocalizationState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/SensorState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamNUSTState.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/TeamInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BehaviorInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/BallInfo.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/StepTargetArr.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsLandmarks.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/sensei/team-nust-robocup/resources/team_nust_nao_interface_ws/src/team_nust_msgs/msg/ObsObstacles.msg" NAME_WE)
add_dependencies(team_nust_msgs_generate_messages_py _team_nust_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team_nust_msgs_genpy)
add_dependencies(team_nust_msgs_genpy team_nust_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team_nust_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team_nust_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(team_nust_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(team_nust_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET humanoid_nav_msgs_generate_messages_cpp)
  add_dependencies(team_nust_msgs_generate_messages_cpp humanoid_nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team_nust_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(team_nust_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(team_nust_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET humanoid_nav_msgs_generate_messages_eus)
  add_dependencies(team_nust_msgs_generate_messages_eus humanoid_nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team_nust_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(team_nust_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(team_nust_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET humanoid_nav_msgs_generate_messages_lisp)
  add_dependencies(team_nust_msgs_generate_messages_lisp humanoid_nav_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team_nust_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(team_nust_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(team_nust_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET humanoid_nav_msgs_generate_messages_py)
  add_dependencies(team_nust_msgs_generate_messages_py humanoid_nav_msgs_generate_messages_py)
endif()
