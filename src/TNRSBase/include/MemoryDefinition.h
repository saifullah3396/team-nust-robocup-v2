/**
 * @file TNRSBase/include/MemoryDefinition.h
 *
 * This file declares the enumeration ids for all the input/output
 * variables of the sharedmemory
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

/**
 * Enumeration for all the variables defined in shared memory
 *
 * @enum MemoryVariableIds
 */
enum class MemoryVariableIds : unsigned int
{
  ///< MotionModule Thread Period
  ///< Input: MotionModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  motionThreadPeriod = 0,
  ///< PlanningModule Thread Period
  ///< Input: CameraModule
  ///< Output: None
  ///< Variable Type: int
  planningThreadPeriod,
  ///< GeneralBehaviors Module Thread Period
  ///< Input: GBModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  sbThreadPeriod,
  ///< VisionModule Thread Period
  ///< Input: VisionModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  visionThreadPeriod,
  ///< LocalizationModule Thread Period
  ///< Input: LocalizationModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  localizationThreadPeriod,
  ///< GameCommModule Thread Period
  ///< Input: CommModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  gameCommThreadPeriod,
  ///< UserCommModule Thread Period
  ///< Input: CommModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  userCommThreadPeriod,
  ///< HeartBeat for connection status
  ///< Input: None,
  ///< Output: CommModule
  ///< Variable Type: int
  heartBeat,
  ///< Joint position sensor values
  ///< Input: MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  jointPositionSensors,
  ///< Joint stiffness sensor values
  ///< Input: MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  jointStiffnessSensors,
  ///< Joint temperature sensor values
  ///< Input: PlanningModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  jointTemperatureSensors,
  ///< Joint current sensor values
  ///< Input: PlanningModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  jointCurrentSensors,
  ///< Hand sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  handSensors,
  ///< Touch sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  touchSensors,
  ///< Switch sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  switchSensors,
  ///< Battery sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  batterySensors,
  ///< Inertial sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  inertialSensors,
  ///< Force sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  fsrSensors,
  ///< Sonar sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  sonarSensors,
  ///< Led sensor values
  ///< Input: PlanningModule/MotionModule
  ///< Output: ControlModule
  ///< Variable Type: vector<float>
  ledSensors,
  ///< Ball information struct
  ///< Input: MotionModule/PlanningModule
  ///< Output: VisionModule
  ///< Variable Type: BallInfo
  ballInfo,
  ///< Ball information struct from other teammates
  ///< Input: MotionModule/PlanningModule
  ///< Output: VisionModule
  ///< Variable Type: worldBallInfo
  worldBallInfo,
  ///< Goal information struct
  ///< Input: LocalizationModule/PlanningModule
  ///< Output: VisionModule
  ///< Variable Type: GoalInfo
  goalInfo,
  ///< Target position for the kick
  ///< Input: CommModule
  ///< Output: MotionModule
  ///< Variable Type: Point2f
  kickTarget,
  ///< Robot foot on ground
  ///< Input: None
  ///< Output: MotionModule
  ///< Variable Type: int
  footOnGround,
  ///< Transformation matrix from standing feet to upper camera
  ///< Input: VisionModule
  ///< Output: MotionModule
  ///< Variable Type: Eigen::Matrix4f
  upperCamInFeet,
  ///< Transformation matrix from standing feet to lower camera
  ///< Input: VisionModule
  ///< Output: MotionModule
  ///< Variable Type: Eigen::Matrix4f
  lowerCamInFeet,
  ///< Transformation matrix from Torso to left foot on ground
  ///< Input: PlanningModule
  ///< Output: MotionModule
  ///< Variable Type: Eigen::Matrix4f
  lFootOnGround,
  ///< Transformation matrix from Torso to right foot on ground
  ///< Input: PlanningModule
  ///< Output: MotionModule
  ///< Variable Type: Eigen::Matrix4f
  rFootOnGround,
  ///< Possible obstacles in the field
  ///< Input: LocalizationModule
  ///< Output: VisionModule
  ///< Variable Type: vector<Obstacle>
  obstaclesObs,
  ///< Obstacles in the field based on team communication data
  ///< Input: LocalizationModule
  ///< Output: CommModule
  ///< Variable Type: vector<Obstacle>
  obstaclesComm,
  ///< Current robot pose in the environment
  ///< Input: MotionModule/PlanningModule
  ///< Output: LocalizationModule
  ///< Variable Type: RobotPose2D
  robotPose2D,
  ///< Last known robot pose in the environment
  ///< Input: MotionModule/PlanningModule
  ///< Output: LocalizationModule
  ///< Variable Type: RobotPose2D
  lastKnownPose2D,
  ///< Field width in meters
  ///< Input: VisionModule/LocalizationModule
  ///< Output: SharedMemory
  ///< Variable Type: OccupancyGrid
  fieldWidth,
  ///< Field width in meters
  ///< Input: VisionModule/LocalizationModule
  ///< Output: SharedMemory
  ///< Variable Type: OccupancyGrid
  fieldHeight,
  ///< Current environment occupancy map
  ///< Input: MotionModule/PlanningModule
  ///< Output: LocalizationModule
  ///< Variable Type: OccupancyGrid
  occupancyMap,
  ///< Id of current state of the robot stiffnesses
  ///< Input: PlanningModule
  ///< Output: GBModule
  ///< Variable Type: unsigned int
  stiffnessState,
  ///< Current robot posture id
  ///< Input: PlanningModule
  ///< Output: MotionModule
  ///< Variable Type: unsigned int
  postureState,
  ///< Id of current state of robot planning module
  ///< Input: LocalizationModule
  ///< Output: PlanningModule
  ///< Variable Type: unsigned int
  planningState,
  ///< Whether the robot is fallen
  ///< Input: PlanningModule
  ///< Output: MotionModule
  ///< Variable Type: bool
  robotFallen,
  ///< Robot is in motion or walk
  ///< Input: LocalizationModule//PlanningModule
  ///< Output: MotionModule
  ///< Variable Type: bool
  robotInMotion,
  ///< Robot stepping foot while in walk
  ///< Input: MotionModule
  ///< Output: MotionModule
  ///< Variable Type: bool
  currentStepLeg,
  ///< Localization module asserts that the robot is localized
  ///< Input: PlanningModule
  ///< Output: LocalizationModule
  ///< Variable Type: bool
  robotLocalized,
  ///< Position confidence of the robot
  ///< Input: CommModule/PlanningModule
  ///< Output: LocalizationModule
  ///< Variable Type: bool
  positionConfidence,
  ///< Side confidence of the robot
  ///< Input: CommModule/PlanningModule
  ///< Output: LocalizationModule
  ///< Variable Type: bool
  sideConfidence,
  ///< Whether the robot is standing on our half sideline
  ///< Input: LocalizationModule/VisionModule
  ///< Output: PlanningModule
  ///< Variable Type: bool
  robotOnSideLine,
  ///< Whether the robot's last known position should be used to get
  ///< robot's state estimate
  ///< Input: LocalizationModule
  ///< Output: PlanningModule
  ///< Variable Type: bool
  localizeWithLastKnown,
  ///< Robot gameplay role id
  ///< Input: LocalizationModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  robocupRole,
  ///< Current intention of the robot during gameplay
  ///< Input: CommModule
  ///< Output: PlanningModule
  ///< Variable Type: int
  robotIntention,
  ///< Number of steps the robot has taken using ALMotionProxy's walk.
  ///< Useful for tracking which footstep is the robot taking currently.
  ///< Input: MotionModule/Localization
  ///< Output: ControlModule
  ///< Variable Type: int
  nFootsteps,
  ///< The goal position to be reached by the MovementModule
  ///< Input: CommModule
  ///< Output: PlanningModule
  ///< Variable Type: RobotPose2D<float>
  moveTarget,
  ///< Game state data
  ///< Input: PlanningModule
  ///< Output: ControlModule
  ///< Variable Type: RoboCupGameControlData
  gameData,
  ///< Player number during gameplay
  ///< Input: ControlModule/CommModule
  ///< Output: Memory
  ///< Variable Type: int
  playerNumber,
  ///< Team number assigned during gameplay
  ///< Input: ControlModule/CommModule
  ///< Output: Memory
  ///< Variable Type: int
  teamNumber,
  ///< Team color assigned during gameplay
  ///< Input: ControlModule/CommModule
  ///< Output: Memory
  ///< Variable Type: int
  teamPort,
  ///< Team port assigned during gameplay
  ///< Input: ControlModule/CommModule
  ///< Output: Memory
  ///< Variable Type: int
  teamColor,
  ///< Team robots data defined by a vector of TeamRobot struct
  ///< Input: PlanningModule
  ///< Output: CommModule
  ///< Variable Type: vector<TeamRobot>
  teamRobots,
  ///< Whislte detection flag
  ///< Input: PlanningModule
  ///< Output: GBModule
  ///< Variable Type: bool
  whistleDetected,
  ///< Whether any landmark is found
  ///< Input: MotionModule
  ///< Output: VisionModule
  ///< Variable Type: bool
  landmarksFound,
  ///< Currently running static behavior info
  ///< Input: Any module
  ///< Output: GBModule
  ///< Variable Type: bool
  gBehaviorInfo,
  ///< Currently running motion behavior info
  ///< Input: Any module
  ///< Output: MotionModule
  ///< Variable Type: bool
  mBehaviorInfo,
  ///< Currently running planning behavior info
  ///< Input: Any module
  ///< Output: PlanningModule
  ///< Variable Type: bool
  pBehaviorInfo,
  ///< Total number of variables
  count
};
