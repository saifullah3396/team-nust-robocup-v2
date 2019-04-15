/**
 * @file PlanningModule/src/PlanningModule.cpp
 *
 * This file implements the class for behavior planning.
 * All the functions and algorithms for adding intelligence to the
 * robot will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#ifdef V6_CROSS_BUILD
#include <qi/anyvalue.hpp>
#endif
#include <boost/make_shared.hpp>
#include "TNRSBase/include/MemoryIOMacros.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "PlanningModule/include/PlanningModule.h"
#include "PlanningModule/include/PBManager.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/WorldBallTracker.h"
#include "Utils/include/PathPlanner/PathPlanner.h"
#include "Utils/include/PathPlanner/GridMap2D.h"
#include "BehaviorConfigs/include/PBConfigs/PBStartupConfig.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/BehaviorInfo.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/StiffnessState.h"
#include "Utils/include/DataHolders/TeamRobot.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

using namespace PathPlannerSpace;

#ifndef V6_CROSS_BUILD
DEFINE_INPUT_CONNECTOR(PlanningModule,
  (int, planningThreadPeriod),
  (bool, landmarksFound),
  (StiffnessState, stiffnessState),
  (PostureState, postureState),
  (RobotPose2D<float>, robotPose2D),
  (BallInfo<float>, ballInfo),
  (bool, robotLocalized),
  (vector<float>, jointPositionSensors),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, inertialSensors),
  (vector<float>, fsrSensors),
  (vector<float>, ledSensors),
  (bool, whistleDetected),
  (bool, robotFallen),
  (int, playerNumber),
  (int, teamNumber),
  (int, teamColor),
  (GoalInfo<float>, goalInfo),
  (vector<TeamRobot<float> >, teamRobots),
  (ObsObstacles<float>, obstaclesObs),
  (BehaviorInfo, gBehaviorInfo),
  (BehaviorInfoMap, mBehaviorInfo),
  (OccupancyMap<float>, occupancyMap),
  (int, nFootsteps),
  (Matrix4f, lFootOnGround),
  (Matrix4f, rFootOnGround),
  (bool, robotInMotion),
  (int, footOnGround),
);

DEFINE_OUTPUT_CONNECTOR(PlanningModule,
  (int, planningThreadTimeTaken),
  (PlanningState, planningState),
  (int, robocupRole),
  (int, robotIntention),
  (bool, robotOnSideLine),
  (bool, localizeWithLastKnown),
  (BehaviorInfo, pBehaviorInfo),
  (vector<float>, jointTemperatureSensors),
  (vector<float>, jointCurrentSensors),
  (vector<float>, touchSensors),
  (vector<float>, switchSensors),
  (vector<float>, batterySensors),
  (vector<float>, sonarSensors),
  (RoboCupGameControlData, gameData),
  (RobotPose2D<float>, moveTarget),
  (WorldBallInfo<float>, worldBallInfo),
);
#else
DEFINE_INPUT_CONNECTOR(PlanningModule,
  (int, planningThreadPeriod),
  (bool, landmarksFound),
  (StiffnessState, stiffnessState),
  (PostureState, postureState),
  (RobotPose2D<float>, robotPose2D),
  (BallInfo<float>, ballInfo),
  (bool, robotLocalized),
  (vector<float>, jointPositionSensors),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, inertialSensors),
  (vector<float>, fsrSensors),
  (vector<float>, ledSensors),
  (bool, whistleDetected),
  (bool, robotFallen),
  (int, playerNumber),
  (int, teamNumber),
  (int, teamColor),
  (GoalInfo<float>, goalInfo),
  (vector<TeamRobot<float> >, teamRobots),
  (ObsObstacles<float>, obstaclesObs),
  (BehaviorInfo, gBehaviorInfo),
  (BehaviorInfoMap, mBehaviorInfo),
  (OccupancyMap<float>, occupancyMap),
  (int, nFootsteps),
  (Matrix4f, lFootOnGround),
  (Matrix4f, rFootOnGround),
  (bool, robotInMotion),
  (int, footOnGround),
  (vector<float>, touchSensors),
  (vector<float>, switchSensors),
);

DEFINE_OUTPUT_CONNECTOR(PlanningModule,
  (int, planningThreadTimeTaken),
  (PlanningState, planningState),
  (int, robocupRole),
  (int, robotIntention),
  (bool, robotOnSideLine),
  (bool, localizeWithLastKnown),
  (BehaviorInfo, pBehaviorInfo),
  #ifndef REALTIME_LOLA_AVAILABLE
  (vector<float>, jointTemperatureSensors),
  (vector<float>, jointCurrentSensors),
  (vector<float>, touchSensors),
  (vector<float>, switchSensors),
  (vector<float>, batterySensors),
  (vector<float>, sonarSensors),
  #endif
  (RoboCupGameControlData, gameData),
  (RobotPose2D<float>, moveTarget),
  (WorldBallInfo<float>, worldBallInfo),
);
#endif

#ifndef V6_CROSS_BUILD
  PlanningModule::PlanningModule(void* parent, const ALMemoryProxyPtr& memoryProxy) :
    BaseModule(
      parent,
      TNSPLModules::planning,
      "PlanningModule"
    ), memoryProxy(memoryProxy)
  {
  }
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    PlanningModule::PlanningModule(void* parent, const qi::AnyObject& memoryProxy) :
      BaseModule(
        parent,
        TNSPLModules::planning,
        "PlanningModule"
      ), memoryProxy(memoryProxy)
    {
    }
  #else
    PlanningModule::PlanningModule(void* parent) :
      BaseModule(parent, TNSPLModules::planning, "PlanningModule")
    {
    }
  #endif
#endif

void PlanningModule::setThreadPeriod()
{
  setPeriodMinMS(PLANNING_PERIOD_IN(PlanningModule));
}

void PlanningModule::setThreadTimeTaken()
{
  PLANNING_TIME_TAKEN_OUT(PlanningModule) = lastIterationTimeMS;
}

void PlanningModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void PlanningModule::init()
{
  LOG_INFO("Initializing planning behavior manager...")
  pbManager = boost::make_shared<PBManager>(this);
  LOG_INFO("Initializing roboCup data handles...")
  setupRoboCupDataHandler();
  LOG_INFO("Initializing planning module sensor layers...")
  #ifndef V6_CROSS_BUILD
    sensorLayers.resize(toUType(PlanningSensors::count));
    sensorLayers[toUType(PlanningSensors::jointTemps)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::joints) + toUType(JointSensorTypes::temp),
        OVAR_PTR(vector<float>, PlanningModule::Output::jointTemperatureSensors),
        memoryProxy);
    sensorLayers[toUType(PlanningSensors::jointCurrents)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::joints) + toUType(JointSensorTypes::current),
        OVAR_PTR(vector<float>, PlanningModule::Output::jointCurrentSensors),
        memoryProxy);
    sensorLayers[toUType(PlanningSensors::touchSensors)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::touchSensors),
        OVAR_PTR(vector<float>, PlanningModule::Output::touchSensors),
        memoryProxy);
    sensorLayers[toUType(PlanningSensors::switchSensors)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::switchSensors),
        OVAR_PTR(vector<float>, PlanningModule::Output::switchSensors),
        memoryProxy);
    sensorLayers[toUType(PlanningSensors::batterSensors)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::batterySensors),
        OVAR_PTR(vector<float>, PlanningModule::Output::batterySensors),
        memoryProxy);
    sensorsUpdate();
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      sensorLayers.resize(toUType(PlanningSensors::count));
      sensorLayers[toUType(PlanningSensors::jointTemps)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::joints) + toUType(JointSensorTypes::temp),
          OVAR_PTR(vector<float>, PlanningModule::Output::jointTemperatureSensors),
          memoryProxy);
      sensorLayers[toUType(PlanningSensors::jointCurrents)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::joints) + toUType(JointSensorTypes::current),
          OVAR_PTR(vector<float>, PlanningModule::Output::jointCurrentSensors),
          memoryProxy);
      sensorLayers[toUType(PlanningSensors::touchSensors)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::touchSensors),
          OVAR_PTR(vector<float>, PlanningModule::Output::touchSensors),
          memoryProxy);
      sensorLayers[toUType(PlanningSensors::switchSensors)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::switchSensors),
          OVAR_PTR(vector<float>, PlanningModule::Output::switchSensors),
          memoryProxy);
      sensorLayers[toUType(PlanningSensors::batterSensors)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::batterySensors),
          OVAR_PTR(vector<float>, PlanningModule::Output::batterySensors),
          memoryProxy);
      sensorsUpdate();
    #endif
  #endif
  LOG_INFO("Initializing world ball tracker...")
  wbTracker = boost::make_shared <WorldBallTracker> (this);
  wbTracker->init();
  ///< Create path planner
  LOG_INFO("Initializing PathPlanner...")
  pathPlanner = PathPlannerPtr(new PathPlanner());
  pathPlanner->setMapPtr(
    boost::make_shared <GridMap2D>(IVAR_PTR(OccupancyMap<float>, PlanningModule::Input::occupancyMap))
  );
  LOG_INFO("Setting request for RobotStartup behavior...")
  auto json = JsonUtils::readJson(ConfigManager::getPBConfigsPath() + "RobotStartup/RequestBehavior.json");
  PBConfigPtr planningConfig = boost::static_pointer_cast<PBConfig>(BehaviorConfig::makeFromJson(json));
  PlanningRequestPtr planningRequest =
    boost::make_shared<RequestPlanningBehavior>(planningConfig);
  addRequest(planningRequest); // publish to itself
  LOG_INFO("Initializing PlanningModule Output Variables...")
  PLANNING_STATE_OUT(PlanningModule) = PlanningState::startup;
  ROBOCUP_ROLE_OUT(PlanningModule) = -1;
  ROBOT_INTENTION_OUT(PlanningModule) = -1;
  ON_SIDE_LINE_OUT(PlanningModule) = false;
  LOCALIZE_LAST_KNOWN_OUT(PlanningModule) = false;
  PB_INFO_OUT(PlanningModule) = BehaviorInfo();
  #ifndef V6_CROSS_BUILD
    JOINT_TEMPERATURES_OUT(PlanningModule) = vector<float>(toUType(Joints::count), 0.f);
    JOINT_CURRENTS_OUT(PlanningModule) = vector<float>(toUType(Joints::count), 0.f);
    TOUCH_SENSORS_OUT(PlanningModule) = vector<float>(toUType(TouchSensors::count), 0.f);
    SWITCH_SENSORS_OUT(PlanningModule) = vector<float>(toUType(SwitchSensors::count), 0.f);
    BATTERY_SENSORS_OUT(PlanningModule) = vector<float>(toUType(BatterySensors::count), 0.f);
    SONAR_SENSORS_OUT(PlanningModule) = vector<float>(toUType(SonarSensors::count), 0.f);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      JOINT_TEMPERATURES_OUT(PlanningModule) = vector<float>(toUType(Joints::count), 0.f);
      JOINT_CURRENTS_OUT(PlanningModule) = vector<float>(toUType(Joints::count), 0.f);
      TOUCH_SENSORS_OUT(PlanningModule) = vector<float>(toUType(TouchSensors::count), 0.f);
      SWITCH_SENSORS_OUT(PlanningModule) = vector<float>(toUType(SwitchSensors::count), 0.f);
      BATTERY_SENSORS_OUT(PlanningModule) = vector<float>(toUType(BatterySensors::count), 0.f);
      SONAR_SENSORS_OUT(PlanningModule) = vector<float>(toUType(SonarSensors::count), 0.f);
    #endif
  #endif
  GAME_DATA_OUT(PlanningModule) = RoboCupGameControlData();
  MOVE_TARGET_OUT(PlanningModule) = RobotPose2D<float>(0.0, 0.0, 0.0);
}

void PlanningModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <PlanningRequest>(request)) {
    auto reqId = request->getRequestId();
    if (reqId == toUType(PlanningRequestIds::behaviorRequest)) {
      auto rpb =
        boost::static_pointer_cast<RequestPlanningBehavior>(request);
      pbManager->manageRequest(rpb);
    } else if (reqId == toUType(PlanningRequestIds::killBehavior)) {
      pbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void PlanningModule::mainRoutine()
{
  //pathPlanner->updateMap();
  #ifndef V6_CROSS_BUILD
    sensorsUpdate();
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      sensorsUpdate();
    #endif
  #endif
  updateWorldBallInfo();
  pbManager->update();
  PB_INFO_OUT(PlanningModule) = pbManager->getBehaviorInfo();
}

#ifndef V6_CROSS_BUILD
  void PlanningModule::sensorsUpdate()
  {
    for (size_t i = 0; i < sensorLayers.size(); ++i) {
      sensorLayers[i]->update();
    }
    RoboCupGameControlData gameControlData;
    AL::ALValue value = memoryProxy->getData("GameCtrl/RoboCupGameControlData");
    if (value.isBinary() && value.getSize() == sizeof(RoboCupGameControlData))
      memcpy(&gameControlData, value, sizeof(RoboCupGameControlData));
    gameControlData.teams[1].teamColour = 2;
    GAME_DATA_OUT(PlanningModule) = gameControlData;
  }
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    void PlanningModule::sensorsUpdate()
    {
      for (size_t i = 0; i < sensorLayers.size(); ++i) {
        sensorLayers[i]->update();
      }
      RoboCupGameControlData gameControlData;
      AL::ALValue value = memoryProxy.call<AL::ALValue>("getData", "GameCtrl/RoboCupGameControlData");
      if (value.isBinary() && value.getSize() == sizeof(RoboCupGameControlData))
        memcpy(&gameControlData, value, sizeof(RoboCupGameControlData));
      gameControlData.teams[1].teamColour = 2;
      GAME_DATA_OUT(PlanningModule) = gameControlData;
    }
  #endif
#endif

void PlanningModule::updateWorldBallInfo()
{
  vector<float> measurements;
  wbTracker->predict();
  if (ROBOT_LOCALIZED_IN(PlanningModule) && BALL_INFO_IN(PlanningModule).found) {
    // Position is translated & rotated with robot's angle
    const auto& robotPose2D = ROBOT_POSE_2D_IN(PlanningModule);
    auto posWorld = robotPose2D.transform(BALL_INFO_IN(PlanningModule).posRel);
    measurements.push_back(posWorld.x);
    measurements.push_back(posWorld.y);
  }
  if (measurements.empty()) {
    for (const auto& tr : TEAM_ROBOTS_IN(PlanningModule)) {
      if (!measurements.empty()) break;
      if (!tr.dataReceived) continue;
      if (tr.ballAge > 0 && tr.ballAge < 0.50f) {
        if (tr.positionConfidence >= 50 && tr.ballAge <= 1.f) {
          auto posRel = tr.ballPos;
          auto rP = tr.pose;
          auto ct = cos(rP.getTheta());
          auto st = sin(rP.getTheta());
          Point2f teamPosWorld;
          // Position is translated & rotated with robot's angle
          teamPosWorld.x = rP.getX() + posRel.x * ct - posRel.y * st;
          teamPosWorld.y = rP.getY() + posRel.x * st + posRel.y * ct;
          measurements.push_back(teamPosWorld.x / 1e3);
          measurements.push_back(teamPosWorld.y / 1e3);
        }
      }
    }
  }
  wbTracker->updateFilter(measurements);
  Mat ballState = wbTracker->getEstimatedState();
  WorldBallInfo<float> wbInfo;
  wbInfo.found = wbTracker->getBallFound();
  wbInfo.posWorld.x = ballState.at<float>(0);
  wbInfo.posWorld.y = ballState.at<float>(1);
  wbInfo.velWorld.x = ballState.at<float>(2);
  wbInfo.velWorld.y = ballState.at<float>(3);
  WORLD_BALL_INFO_OUT(PlanningModule) = wbInfo;
}

#ifndef V6_CROSS_BUILD
  void PlanningModule::setupRoboCupDataHandler()
  {
    #ifdef MODULE_IS_REMOTE
    RoboCupGameControlData gameCtrlData;
    AL::ALValue value((const char*) &gameCtrlData, sizeof(gameCtrlData));
    memoryProxy->insertData("GameCtrl/RoboCupGameControlData", value);
    #endif
    memoryProxy->insertData(
      "GameCtrl/teamNumber",
      (int) TEAM_NUMBER_IN(PlanningModule));
    memoryProxy->insertData(
      "GameCtrl/teamColour",
      (int) TEAM_COLOR_IN(PlanningModule));
    memoryProxy->insertData(
      "GameCtrl/playerNumber",
      (int) PLAYER_NUMBER_IN(PlanningModule));
  }
#else
  #ifndef REALTIME_LOLA_AVAILABLE
  void PlanningModule::setupRoboCupDataHandler()
  {
    //! This data is received through game comm module if lola is used
    //! and actuations are done through lola module
    #ifndef REALTIME_LOLA_AVAILABLE
      memoryProxy.call<void>("insertData", "GameCtrl/teamNumber", (int) TEAM_NUMBER_IN(PlanningModule));
      memoryProxy.call<void>("insertData", "GameCtrl/teamColour", (int) TEAM_COLOR_IN(PlanningModule));
      memoryProxy.call<void>("insertData", "GameCtrl/playerNumber", (int) PLAYER_NUMBER_IN(PlanningModule));
    #endif
  }
  #endif
#endif

PathPlannerSpace::PathPlannerPtr PlanningModule::getPathPlanner()
{
  return pathPlanner;
}
