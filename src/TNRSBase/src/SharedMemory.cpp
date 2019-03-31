/**
 * @file TNRSBase/src/SharedMemory.cpp
 *
 * This file implements the class SharedMemory
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/BaseModule.h"
#include "TNRSBase/include/MemoryVariable.h"
#include "TNRSBase/include/SharedMemory.h"
#include "Utils/include/DataHolders/BehaviorInfo.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/SPLStandardMessage.h"
#include "Utils/include/DataHolders/StiffnessState.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/JsonUtils.h"

typedef map<unsigned, BehaviorInfo> BehaviorInfoMap;

void
SharedMemory::init()
{
  variables.assign(toUType(MemoryVariableIds::count), NULL);
  int mtPeriod, ptPeriod, sbtPeriod, vtPeriod, ltPeriod, ucommtPeriod, gcommtPeriod;
  GET_CONFIG(
    "BaseModules",
    (int, MotionModule.period, mtPeriod),
    (int, PlanningModule.period, ptPeriod),
    (int, GBModule.period, sbtPeriod),
    (int, VisionModule.period, vtPeriod),
    (int, LocalizationModule.period, ltPeriod),
    (int, UserCommModule.period, ucommtPeriod),
    (int, GameCommModule.period, gcommtPeriod),
  )
  DEFINE_VARIABLE(int, motionThreadPeriod, mtPeriod); //15
  DEFINE_VARIABLE(int, planningThreadPeriod, ptPeriod);
  DEFINE_VARIABLE(int, sbThreadPeriod, sbtPeriod);
  DEFINE_VARIABLE(int, visionThreadPeriod, vtPeriod);
  DEFINE_VARIABLE(int, localizationThreadPeriod, ltPeriod);
  DEFINE_VARIABLE(int, userCommThreadPeriod, ucommtPeriod);
  DEFINE_VARIABLE(int, gameCommThreadPeriod, gcommtPeriod);
  DEFINE_VARIABLE(int, heartBeat, 0);
  DEFINE_VARIABLE(
    vector<float>,
    jointPositionSensors,
    vector<float>(toUType(Joints::count)));
  DEFINE_VARIABLE(
    vector<float>,
    jointStiffnessSensors,
    vector<float>(toUType(Joints::count)));
  DEFINE_VARIABLE(
    vector<float>,
    jointTemperatureSensors,
    vector<float>(toUType(Joints::count)));
  DEFINE_VARIABLE(
    vector<float>,
    jointCurrentSensors,
    vector<float>(toUType(Joints::count)));
  DEFINE_VARIABLE(
    vector<float>,
    handSensors,
    vector<float>(toUType(RobotHands::count)));
  DEFINE_VARIABLE(
    vector<float>,
    touchSensors,
    vector<float>(toUType(TouchSensors::count)));
  DEFINE_VARIABLE(
    vector<float>,
    switchSensors,
    vector<float>(toUType(SwitchSensors::count)));
  DEFINE_VARIABLE(
    vector<float>,
    batterySensors,
    vector<float>(toUType(BatterySensors::count)));
  DEFINE_VARIABLE(
    vector<float>,
    inertialSensors,
    vector<float>(toUType(InertialSensors::count)));
  DEFINE_VARIABLE(
    vector<float>,
    sonarSensors,
    vector<float>(toUType(SonarSensors::count)));
  DEFINE_VARIABLE(
    vector<float>,
    fsrSensors,
    vector<float>(toUType(FsrSensors::count)));
  DEFINE_VARIABLE(
    vector<float>,
    ledSensors,
    vector<float>(toUType(LedActuators::count)));
  //DECLARE_VARIABLE(vector<float>, imuDataFilterOutput, vector<float>(1));
  float ballRadius;
  GET_CONFIG(
    "EnvProperties",
    (float, ballRadius, ballRadius),
  )
  DEFINE_VARIABLE(BallInfo<float>, ballInfo, BallInfo<float>());
  DEFINE_VARIABLE(WorldBallInfo<float>, worldBallInfo, WorldBallInfo<float>());
  DEFINE_VARIABLE(GoalInfo<float>, goalInfo, GoalInfo<float>());
  DEFINE_VARIABLE(cv::Point_<float>, kickTarget, cv::Point_<float>());
  DEFINE_VARIABLE(RobotFeet, footOnGround, RobotFeet::lFoot);
  DEFINE_VARIABLE(Matrix4f, upperCamInFeet, Matrix4f::Identity());
  DEFINE_VARIABLE(Matrix4f, lowerCamInFeet, Matrix4f::Identity());
  DEFINE_VARIABLE(Matrix4f, lFootOnGround, Matrix4f::Identity());
  DEFINE_VARIABLE(Matrix4f, rFootOnGround, Matrix4f::Identity());
  DEFINE_VARIABLE(ObsObstacles<float>, obstaclesObs, ObsObstacles<float>());
  DEFINE_VARIABLE(ObsObstacles<float>, obstaclesComm, ObsObstacles<float>());
  DEFINE_VARIABLE(
    RobotPose2D<float>,
    robotPose2D,
    RobotPose2D<float>(0.0, 0.0, 0.0));
  DEFINE_VARIABLE(
    RobotPose2D<float>,
    lastKnownPose2D,
    RobotPose2D<float>(-1e3, -1e3, -1e3));
  DECLARE_VARIABLE(float, fieldWidth);
  DECLARE_VARIABLE(float, fieldHeight);
  DECLARE_VARIABLE(OccupancyMap<float>, occupancyMap);
  DEFINE_VARIABLE(StiffnessState, stiffnessState, StiffnessState::unknown);
  DEFINE_VARIABLE(PostureState, postureState, PostureState::unknown);
  DEFINE_VARIABLE(PlanningState, planningState, PlanningState::unknown);
  DEFINE_VARIABLE(bool, robotFallen, false);
  DEFINE_VARIABLE(bool, robotInMotion, false);
  DEFINE_VARIABLE(RobotFeet, currentStepLeg, RobotFeet::unknown); // No leg
  DEFINE_VARIABLE(bool, robotLocalized, false);
  DEFINE_VARIABLE(int, positionConfidence, 0);
  DEFINE_VARIABLE(int, sideConfidence, 0);
  DEFINE_VARIABLE(bool, robotOnSideLine, false);
  DEFINE_VARIABLE(bool, localizeWithLastKnown, false);
  DEFINE_VARIABLE(bool, landmarksFound, false);
  DEFINE_VARIABLE(int, robocupRole, -1); // None defined
  DEFINE_VARIABLE(int, robotIntention, 0);
  DEFINE_VARIABLE(int, nFootsteps, 0);
  DEFINE_VARIABLE(
    RobotPose2D<float>,
    moveTarget,
    RobotPose2D<float>(0.0, 0.0, 0.0));
  DECLARE_VARIABLE(RoboCupGameControlData, gameData);
  int pNumber;
  int tNumber;
  int tPort;
  int tColor;
  GET_CONFIG(
    "TeamInfo",
    (int, playerNumber, pNumber),
    (int, teamNumber, tNumber),
    (int, teamPort, tPort),
    (int, teamColor, tColor), )
  DEFINE_VARIABLE(int, playerNumber, pNumber);
  DEFINE_VARIABLE(int, teamNumber, tNumber);
  DEFINE_VARIABLE(int, teamPort, tPort);
  DEFINE_VARIABLE(int, teamColor, tColor);
  DEFINE_VARIABLE(
    vector<TeamRobot<float> >,
    teamRobots,
    vector<TeamRobot<float> >(SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS));
  DEFINE_VARIABLE(bool, whistleDetected, false);
  DEFINE_VARIABLE(BehaviorInfo, gBehaviorInfo, BehaviorInfo());
  DEFINE_VARIABLE(BehaviorInfoMap, mBehaviorInfo, BehaviorInfoMap());
  DEFINE_VARIABLE(BehaviorInfo, pBehaviorInfo, BehaviorInfo());
}

Json::Value SharedMemory::getJson()
{
  Json::Value root;
  for (const auto& v : variables) {
    JSON_ASSIGN(root, v->getVariableName(), v->getJson());
  }
  return root;
}
