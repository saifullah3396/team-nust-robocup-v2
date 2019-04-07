cmake_minimum_required(VERSION 2.8.3)
project(team_nust_msgs)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   humanoid_nav_msgs
   message_generation
)

add_message_files(
  FILES
  JointInfo.msg
  SensorState.msg
  BallInfo.msg
  GoalInfo.msg
  TeamInfo.msg
  TeamRobot.msg
  TeamNUSTState.msg
  Obstacle.msg
  ObsObstacles.msg
  LocalizationState.msg
  Landmark.msg
  ObsLandmarks.msg
  StepTargetArr.msg
  BehaviorInfo.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  humanoid_nav_msgs
)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime
   )