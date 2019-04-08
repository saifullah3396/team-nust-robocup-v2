///< Input Variables
#define MOTION_PERIOD_IN(Base) IVAR(int, Base::Input::motionThreadPeriod)
#define PLANNING_PERIOD_IN(Base) IVAR(int, Base::Input::planningThreadPeriod)
#define GB_PERIOD_IN(Base) IVAR(int, Base::Input::gbThreadPeriod)
#define VISION_PERIOD_IN(Base) IVAR(int, Base::Input::visionThreadPeriod)
#define LOCALIZATION_PERIOD_IN(Base) IVAR(int, Base::Input::localizationThreadPeriod)
#define GAME_COMM_PERIOD_IN(Base) IVAR(int, Base::Input::gameCommThreadPeriod)
#define USER_COMM_PERIOD_IN(Base) IVAR(int, Base::Input::userCommThreadPeriod)
#define MOTION_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::motionThreadPeriod)
#define PLANNING_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::planningThreadTimeTaken)
#define GB_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::gbThreadTimeTaken)
#define VISION_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::visionThreadTimeTaken)
#define LOCALIZATION_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::localizationThreadTimeTaken)
#define GAME_COMM_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::gameCommThreadTimeTaken)
#define USER_COMM_TIME_TAKEN_IN(Base) IVAR(int, Base::Input::userCommThreadTimeTaken)
#define HEART_BEAT_IN(Base) IVAR(int, Base::Input::heartBeat)
#define JOINT_POSITIONS_IN(Base) IVAR(vector<float>, Base::Input::jointPositionSensors)
#define JOINT_STIFFNESSES_IN(Base) IVAR(vector<float>, Base::Input::jointStiffnessSensors)
#define JOINT_TEMPERATURES_IN(Base) IVAR(vector<float>, Base::Input::jointTemperatureSensors)
#define JOINT_CURRENTS_IN(Base) IVAR(vector<float>, Base::Input::jointCurrentSensors)
#define HAND_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::handSensors)
#define TOUCH_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::touchSensors)
#define SWITCH_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::switchSensors)
#define BATTERY_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::batterySensors)
#define INERTIAL_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::inertialSensors)
#define FSR_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::fsrSensors)
#define SONAR_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::sonarSensors)
#define LED_SENSORS_IN(Base) IVAR(vector<float>, Base::Input::ledSensors)
#define BALL_INFO_IN(Base) IVAR(BallInfo<float>, Base::Input::ballInfo)
#define WORLD_BALL_INFO_IN(Base) IVAR(WorldBallInfo<float>, Base::Input::worldBallInfo)
#define GOAL_INFO_IN(Base) IVAR(GoalInfo<float>, Base::Input::goalInfo)
#define KICK_TARGET_IN(Base) IVAR(cv::Point_<float>, Base::Input::kickTarget)
#define FOOT_ON_GROUND_IN(Base) IVAR(RobotFeet, Base::Input::footOnGround)
#define UPPER_CAM_TRANS_IN(Base) IVAR(Matrix4f, Base::Input::upperCamInFeet)
#define LOWER_CAM_TRANS_IN(Base) IVAR(Matrix4f, Base::Input::lowerCamInFeet)
#define L_FOOT_TRANS_IN(Base) IVAR(Matrix4f, Base::Input::lFootOnGround)
#define R_FOOT_TRANS_IN(Base) IVAR(Matrix4f, Base::Input::rFootOnGround)
#define OBSTACLES_OBS_IN(Base) IVAR(ObsObstacles<float>, Base::Input::obstaclesObs)
#define OBSTACLES_COMM_IN(Base) IVAR(ObsObstacles<float>, Base::Input::obstaclesComm)
#define ROBOT_POSE_2D_IN(Base) IVAR(RobotPose2D<float>, Base::Input::robotPose2D)
#define LAST_POSE_2D_IN(Base) IVAR(RobotPose2D<float>, Base::Input::lastKnownPose2D)
#define FIELD_WIDTH_IN(Base) IVAR(float, Base::Input::fieldWidth)
#define FIELD_HEIGHT_IN(Base) IVAR(float, Base::Input::fieldHeight)
#define OCCUPANCY_MAP_IN(Base) IVAR(OccupancyMap<float>, Base::Input::occupancyMap)
#define STIFFNESS_STATE_IN(Base) IVAR(StiffnessState, Base::Input::stiffnessState)
#define POSTURE_STATE_IN(Base) IVAR(PostureState, Base::Input::postureState)
#define PLANNING_STATE_IN(Base) IVAR((PlanningState, Base::Input::planningState)
#define ROBOT_FALLEN_IN(Base) IVAR(bool, Base::Input::robotFallen)
#define ROBOT_IN_MOTION_IN(Base) IVAR(bool, Base::Input::robotInMotion)
#define STEP_LEG_IN(Base) IVAR(RobotFeet, Base::Input::currentStepLeg)
#define ROBOT_LOCALIZED_IN(Base) IVAR(bool, Base::Input::robotLocalized)
#define POSITION_CONFIDENCE_IN(Base) IVAR(int, Base::Input::positionConfidence)
#define SIDE_CONFIDENCE_IN(Base) IVAR(int, Base::Input::sideConfidence)
#define ON_SIDE_LINE_IN(Base) IVAR(bool, Base::Input::robotOnSideLine)
#define LOCALIZE_LAST_KNOWN_IN(Base) IVAR(bool, Base::Input::localizeWithLastKnown)
#define ROBOCUP_ROLE_IN(Base) IVAR(int, Base::Input::robocupRole)
#define ROBOT_INTENTION_IN(Base) IVAR(int, Base::Input::robotIntention)
#define N_FOOTSTEPS_IN(Base) IVAR(int, Base::Input::nFootsteps)
#define MOVE_TARGET_IN(Base) IVAR(RobotPose2D<float>, Base::Input::moveTarget)
#define GAME_DATA_IN(Base) IVAR(RoboCupGameControlData, Base::Input::gameData)
#define PLAYER_NUMBER_IN(Base) IVAR(int, Base::Input::playerNumber)
#define TEAM_NUMBER_IN(Base) IVAR(int, Base::Input::teamNumber)
#define TEAM_PORT_IN(Base) IVAR(int, Base::Input::teamPort)
#define TEAM_COLOR_IN(Base) IVAR(int, Base::Input::teamColor)
#define TEAM_ROBOTS_IN(Base) IVAR(vector<TeamRobot<float> >, Base::Input::teamRobots)
#define WHISTLE_DETECTED_IN(Base) IVAR(bool, Base::Input::whistleDetected)
#define LANDMARKS_FOUND_IN(Base) IVAR(bool, Base::Input::landmarksFound)
#define GB_INFO_IN(Base) IVAR(BehaviorInfo, Base::Input::gBehaviorInfo)
#define MB_INFO_IN(Base) IVAR(BehaviorInfoMap, Base::Input::mBehaviorInfo)
#define PB_INFO_IN(Base) IVAR(BehaviorInfo, Base::Input::pBehaviorInfo)

#define MOTION_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::motionThreadPeriod)
#define PLANNING_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::planningThreadPeriod)
#define GB_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::gbThreadPeriod)
#define VISION_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::visionThreadPeriod)
#define LOCALIZATION_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::localizationThreadPeriod)
#define GAME_COMM_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::gameCommThreadPeriod)
#define USER_COMM_PERIOD_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::userCommThreadPeriod)
#define MOTION_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::motionThreadTimeTaken)
#define PLANNING_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::planningThreadTimeTaken)
#define GB_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::gbThreadTimeTaken)
#define VISION_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::visionThreadTimeTaken)
#define LOCALIZATION_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::localizationThreadTimeTaken)
#define GAME_COMM_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::gameCommThreadTimeTaken)
#define USER_COMM_TIME_TAKEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::userCommThreadTimeTaken)
#define HEART_BEAT_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::heartBeat)
#define JOINT_POSITIONS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::jointPositionSensors)
#define JOINT_STIFFNESSES_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::jointStiffnessSensors)
#define JOINT_TEMPERATURES_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::jointTemperatureSensors)
#define JOINT_CURRENTS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::jointCurrentSensors)
#define TOUCH_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::touchSensors)
#define HAND_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::handSensors)
#define SWITCH_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::switchSensors)
#define BATTERY_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::batterySensors)
#define INERTIAL_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::inertialSensors)
#define FSR_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::fsrSensors)
#define SONAR_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::sonarSensors)
#define LED_SENSORS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<float>, Base::Input::ledSensors)
#define BALL_INFO_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, BallInfo<float>, Base::Input::ballInfo)
#define WORLD_BALL_INFO_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, WorldBallInfo<float>, Base::Input::worldBallInfo)
#define GOAL_INFO_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, GoalInfo<float>, Base::Input::goalInfo)
#define KICK_TARGET_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, cv::Point_<float>, Base::Input::kickTarget)
#define FOOT_ON_GROUND_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, RobotFeet, Base::Input::footOnGround)
#define UPPER_CAM_TRANS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, Matrix4f, Base::Input::upperCamInFeet)
#define LOWER_CAM_TRANS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, Matrix4f, Base::Input::lowerCamInFeet)
#define L_FOOT_TRANS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, Matrix4f, Base::Input::lFootOnGround)
#define R_FOOT_TRANS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, Matrix4f, Base::Input::rFootOnGround)
#define OBSTACLES_OBS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, ObsObstacles<float>, Base::Input::obstaclesObs)
#define OBSTACLES_COMM_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, ObsObstacles<float>, Base::Input::obstaclesComm)
#define ROBOT_POSE_2D_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, RobotPose2D<float>, Base::Input::robotPose2D)
#define LAST_POSE_2D_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, RobotPose2D<float>, Base::Input::lastKnownPose2D)
#define FIELD_WIDTH_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, float, Base::Input::fieldWidth)
#define FIELD_HEIGHT_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, float, Base::Input::fieldHeight)
#define OCCUPANCY_MAP_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, OccupancyMap<float>, Base::Input::occupancyMap)
#define STIFFNESS_STATE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, StiffnessState, Base::Input::stiffnessState)
#define POSTURE_STATE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, PostureState, Base::Input::postureState)
#define PLANNING_STATE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, (PlanningState, Base::Input::planningState)
#define ROBOT_FALLEN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::robotFallen)
#define ROBOT_IN_MOTION_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::robotInMotion)
#define STEP_LEG_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, RobotFeet, Base::Input::currentStepLeg)
#define ROBOT_LOCALIZED_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::robotLocalized)
#define POSITION_CONFIDENCE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::positionConfidence)
#define SIDE_CONFIDENCE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::sideConfidence)
#define ON_SIDE_LINE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::robotOnSideLine)
#define LOCALIZE_LAST_KNOWN_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::localizeWithLastKnown)
#define ROBOCUP_ROLE_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::robocupRole)
#define ROBOT_INTENTION_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::robotIntention)
#define N_FOOTSTEPS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::nFootsteps)
#define MOVE_TARGET_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, RobotPose2D<float>, Base::Input::moveTarget)
#define GAME_DATA_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, RoboCupGameControlData, Base::Input::gameData)
#define PLAYER_NUMBER_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::playerNumber)
#define TEAM_NUMBER_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::teamNumber)
#define TEAM_PORT_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::teamPort)
#define TEAM_COLOR_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, int, Base::Input::teamColor)
#define TEAM_ROBOTS_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, vector<TeamRobot<float> >, Base::Input::teamRobots)
#define WHISTLE_DETECTED_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::whistleDetected)
#define LANDMARKS_FOUND_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, bool, Base::Input::landmarksFound)
#define GB_INFO_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, BehaviorInfo, Base::Input::gBehaviorInfo)
#define MB_INFO_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, BehaviorInfoMap, Base::Input::mBehaviorInfo)
#define PB_INFO_IN_REL(Base, BasePtr) IVAR_REL(BasePtr, BehaviorInfo, Base::Input::pBehaviorInfo)

///< Output Variables
#define MOTION_PERIOD_OUT(Base) OVAR(int, Base::Output::motionThreadPeriod)
#define PLANNING_PERIOD_OUT(Base) OVAR(int, Base::Output::planningThreadPeriod)
#define GB_PERIOD_OUT(Base) OVAR(int, Base::Output::gbThreadPeriod)
#define VISION_PERIOD_OUT(Base) OVAR(int, Base::Output::visionThreadPeriod)
#define LOCALIZATION_PERIOD_OUT(Base) OVAR(int, Base::Output::localizationThreadPeriod)
#define GAME_COMM_PERIOD_OUT(Base) OVAR(int, Base::Output::gameCommThreadPeriod)
#define USER_COMM_PERIOD_OUT(Base) OVAR(int, Base::Output::userCommThreadPeriod)
#define MOTION_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::motionThreadTimeTaken)
#define PLANNING_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::planningThreadTimeTaken)
#define GB_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::gbThreadTimeTaken)
#define VISION_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::visionThreadTimeTaken)
#define LOCALIZATION_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::localizationThreadTimeTaken)
#define GAME_COMM_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::gameCommThreadTimeTaken)
#define USER_COMM_TIME_TAKEN_OUT(Base) OVAR(int, Base::Output::userCommThreadTimeTaken)
#define HEART_BEAT_OUT(Base) OVAR(int, Base::Output::heartBeat)
#define JOINT_POSITIONS_OUT(Base) OVAR(vector<float>, Base::Output::jointPositionSensors)
#define JOINT_STIFFNESSES_OUT(Base) OVAR(vector<float>, Base::Output::jointStiffnessSensors)
#define JOINT_TEMPERATURES_OUT(Base) OVAR(vector<float>, Base::Output::jointTemperatureSensors)
#define JOINT_CURRENTS_OUT(Base) OVAR(vector<float>, Base::Output::jointCurrentSensors)
#define TOUCH_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::touchSensors)
#define HAND_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::handSensors)
#define SWITCH_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::switchSensors)
#define BATTERY_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::batterySensors)
#define INERTIAL_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::inertialSensors)
#define FSR_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::fsrSensors)
#define SONAR_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::sonarSensors)
#define LED_SENSORS_OUT(Base) OVAR(vector<float>, Base::Output::ledSensors)
#define BALL_INFO_OUT(Base) OVAR(BallInfo<float>, Base::Output::ballInfo)
#define WORLD_BALL_INFO_OUT(Base) OVAR(WorldBallInfo<float>, Base::Output::worldBallInfo)
#define GOAL_INFO_OUT(Base) OVAR(GoalInfo<float>, Base::Output::goalInfo)
#define KICK_TARGET_OUT(Base) OVAR(cv::Point_<float>, Base::Output::kickTarget)
#define FOOT_ON_GROUND_OUT(Base) OVAR(RobotFeet, Base::Output::footOnGround)
#define UPPER_CAM_TRANS_OUT(Base) OVAR(Matrix4f, Base::Output::upperCamInFeet)
#define LOWER_CAM_TRANS_OUT(Base) OVAR(Matrix4f, Base::Output::lowerCamInFeet)
#define L_FOOT_TRANS_OUT(Base) OVAR(Matrix4f, Base::Output::lFootOnGround)
#define R_FOOT_TRANS_OUT(Base) OVAR(Matrix4f, Base::Output::rFootOnGround)
#define OBSTACLES_OBS_OUT(Base) OVAR(ObsObstacles<float>, Base::Output::obstaclesObs)
#define OBSTACLES_COMM_OUT(Base) OVAR(ObsObstacles<float>, Base::Output::obstaclesComm)
#define ROBOT_POSE_2D_OUT(Base) OVAR(RobotPose2D<float>, Base::Output::robotPose2D)
#define LAST_POSE_2D_OUT(Base) OVAR(RobotPose2D<float>, Base::Output::lastKnownPose2D)
#define FIELD_WIDTH_OUT(Base) OVAR(float, Base::Output::fieldWidth)
#define FIELD_HEIGHT_OUT(Base) OVAR(float, Base::Output::fieldHeight)
#define OCCUPANCY_MAP_OUT(Base) OVAR(OccupancyMap<float>, Base::Output::occupancyMap)
#define STIFFNESS_STATE_OUT(Base) OVAR(StiffnessState, Base::Output::stiffnessState)
#define POSTURE_STATE_OUT(Base) OVAR(PostureState, Base::Output::postureState)
#define PLANNING_STATE_OUT(Base) OVAR(PlanningState, Base::Output::planningState)
#define ROBOT_FALLEN_OUT(Base) OVAR(bool, Base::Output::robotFallen)
#define ROBOT_IN_MOTION_OUT(Base) OVAR(bool, Base::Output::robotInMotion)
#define STEP_LEG_OUT(Base) OVAR(RobotFeet, Base::Output::currentStepLeg)
#define ROBOT_LOCALIZED_OUT(Base) OVAR(bool, Base::Output::robotLocalized)
#define POSITION_CONFIDENCE_OUT(Base) OVAR(int, Base::Output::positionConfidence)
#define SIDE_CONFIDENCE_OUT(Base) OVAR(int, Base::Output::sideConfidence)
#define ON_SIDE_LINE_OUT(Base) OVAR(bool, Base::Output::robotOnSideLine)
#define LOCALIZE_LAST_KNOWN_OUT(Base) OVAR(bool, Base::Output::localizeWithLastKnown)
#define ROBOCUP_ROLE_OUT(Base) OVAR(int, Base::Output::robocupRole)
#define ROBOT_INTENTION_OUT(Base) OVAR(int, Base::Output::robotIntention)
#define N_FOOTSTEPS_OUT(Base) OVAR(int, Base::Output::nFootsteps)
#define MOVE_TARGET_OUT(Base) OVAR(RobotPose2D<float>, Base::Output::moveTarget)
#define GAME_DATA_OUT(Base) OVAR(RoboCupGameControlData, Base::Output::gameData)
#define PLAYER_NUMBER_OUT(Base) OVAR(int, Base::Output::playerNumber)
#define TEAM_NUMBER_OUT(Base) OVAR(int, Base::Output::teamNumber)
#define TEAM_PORT_OUT(Base) OVAR(int, Base::Output::teamPort)
#define TEAM_COLOR_OUT(Base) OVAR(int, Base::Output::teamColor)
#define TEAM_ROBOTS_OUT(Base) OVAR(vector<TeamRobot<float> >, Base::Output::teamRobots)
#define WHISTLE_DETECTED_OUT(Base) OVAR(bool, Base::Output::whistleDetected)
#define LANDMARKS_FOUND_OUT(Base) OVAR(bool, Base::Output::landmarksFound)
#define GB_INFO_OUT(Base) OVAR(BehaviorInfo, Base::Output::gBehaviorInfo)
#define MB_INFO_OUT(Base) OVAR(BehaviorInfoMap, Base::Output::mBehaviorInfo)
#define PB_INFO_OUT(Base) OVAR(BehaviorInfo, Base::Output::pBehaviorInfo)

///< Output Variables
#define MOTION_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::motionThreadPeriod)
#define PLANNING_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::planningThreadPeriod)
#define GB_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::gbThreadPeriod)
#define VISION_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::visionThreadPeriod)
#define LOCALIZATION_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::localizationThreadPeriod)
#define GAME_COMM_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::gameCommThreadPeriod)
#define USER_COMM_PERIOD_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::userCommThreadPeriod)
#define MOTION_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::motionThreadTimeTaken)
#define PLANNING_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::planningThreadTimeTaken)
#define GB_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::gbThreadTimeTaken)
#define VISION_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::visionThreadTimeTaken)
#define LOCALIZATION_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::localizationThreadTimeTaken)
#define GAME_COMM_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::gameCommThreadTimeTaken)
#define USER_COMM_TIME_TAKEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::userCommThreadTimeTaken)
#define HEART_BEAT_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::heartBeat)
#define JOINT_POSITIONS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::jointPositionSensors)
#define JOINT_STIFFNESSES_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::jointStiffnessSensors)
#define JOINT_TEMPERATURES_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::jointTemperatureSensors)
#define JOINT_CURRENTS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::jointCurrentSensors)
#define HAND_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::handSensors)
#define TOUCH_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::touchSensors)
#define SWITCH_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::switchSensors)
#define BATTERY_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::batterySensors)
#define INERTIAL_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::inertialSensors)
#define FSR_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::fsrSensors)
#define SONAR_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::sonarSensors)
#define LED_SENSORS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<float>, Base::Output::ledSensors)
#define BALL_INFO_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, BallInfo<float>, Base::Output::ballInfo)
#define WORLD_BALL_INFO_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, WorldBallInfo<float>, Base::Output::worldBallInfo)
#define GOAL_INFO_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, GoalInfo<float>, Base::Output::goalInfo)
#define KICK_TARGET_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, cv::Point_<float>, Base::Output::kickTarget)
#define FOOT_ON_GROUND_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, RobotFeet, Base::Output::footOnGround)
#define UPPER_CAM_TRANS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, Matrix4f, Base::Output::upperCamInFeet)
#define LOWER_CAM_TRANS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, Matrix4f, Base::Output::lowerCamInFeet)
#define L_FOOT_TRANS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, Matrix4f, Base::Output::lFootOnGround)
#define R_FOOT_TRANS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, Matrix4f, Base::Output::rFootOnGround)
#define OBSTACLES_OBS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, ObsObstacles<float>, Base::Output::obstaclesObs)
#define OBSTACLES_COMM_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, ObsObstacles<float>, Base::Output::obstaclesComm)
#define ROBOT_POSE_2D_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, RobotPose2D<float>, Base::Output::robotPose2D)
#define LAST_POSE_2D_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, RobotPose2D<float>, Base::Output::lastKnownPose2D)
#define FIELD_WIDTH_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, float, Base::Output::fieldWidth)
#define FIELD_HEIGHT_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, float, Base::Output::fieldHeight)
#define OCCUPANCY_MAP_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, OccupancyMap<float>, Base::Output::occupancyMap)
#define STIFFNESS_STATE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, StiffnessState, Base::Output::stiffnessState)
#define POSTURE_STATE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, PostureState, Base::Output::postureState)
#define PLANNING_STATE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, PlanningState, Base::Output::planningState)
#define ROBOT_FALLEN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::robotFallen)
#define ROBOT_IN_MOTION_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::robotInMotion)
#define STEP_LEG_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, RobotFeet, Base::Output::currentStepLeg)
#define ROBOT_LOCALIZED_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::robotLocalized)
#define POSITION_CONFIDENCE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::positionConfidence)
#define SIDE_CONFIDENCE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::sideConfidence)
#define ON_SIDE_LINE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::robotOnSideLine)
#define LOCALIZE_LAST_KNOWN_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::localizeWithLastKnown)
#define ROBOCUP_ROLE_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::robocupRole)
#define ROBOT_INTENTION_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::robotIntention)
#define N_FOOTSTEPS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::nFootsteps)
#define MOVE_TARGET_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, RobotPose2D<float>, Base::Output::moveTarget)
#define GAME_DATA_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, RoboCupGameControlData, Base::Output::gameData)
#define PLAYER_NUMBER_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::playerNumber)
#define TEAM_NUMBER_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::teamNumber)
#define TEAM_PORT_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::teamPort)
#define TEAM_COLOR_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, int, Base::Output::teamColor)
#define TEAM_ROBOTS_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, vector<TeamRobot<float> >, Base::Output::teamRobots)
#define WHISTLE_DETECTED_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::whistleDetected)
#define LANDMARKS_FOUND_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, bool, Base::Output::landmarksFound)
#define GB_INFO_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, BehaviorInfo, Base::Output::gBehaviorInfo)
#define MB_INFO_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, BehaviorInfoMap, Base::Output::mBehaviorInfo)
#define PB_INFO_OUT_REL(Base, BasePtr) OVAR_REL(BasePtr, BehaviorInfo, Base::Output::pBehaviorInfo)
