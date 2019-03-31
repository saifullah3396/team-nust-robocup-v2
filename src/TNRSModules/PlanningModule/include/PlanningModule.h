/**
 * @file PlanningModule/include/PlanningModule.h
 *
 * This file declares the class PlanningModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#ifndef V6_CROSS_BUILD
#include <alproxies/almemoryproxy.h> //! Not available in SDK 2.8
#endif
#include "ControlModule/include/HardwareLayer.h"
#include "TNRSBase/include/BaseIncludes.h"

#ifndef V6_CROSS_BUILD
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
#endif
namespace PathPlannerSpace
{
  class PathPlanner;
  typedef boost::shared_ptr<PathPlanner> PathPlannerPtr;
}
class WorldBallTracker;
typedef map<unsigned, BehaviorInfo> BehaviorInfoMap;
class PBManager;
typedef boost::shared_ptr<PBManager> PBManagerPtr;

/**
 * @class PlanningModule
 * @brief The class for behavior planning. All the functions and
 *   algorithms for adding intelligence to the robot are defined
 *   under this module.
 */
class PlanningModule : public BaseModule
{
  DECLARE_INPUT_CONNECTOR(
    planningThreadPeriod,
    landmarksFound,
    stiffnessState,
    postureState,
    robotPose2D,
    ballInfo,
    robotLocalized,
    jointPositionSensors,
    jointStiffnessSensors,
    inertialSensors,
    fsrSensors,
    ledSensors,
    whistleDetected,
    robotFallen,
    playerNumber,
    teamNumber,
    teamColor,
    goalInfo,
    teamRobots,
    obstaclesObs,
    gBehaviorInfo,
    mBehaviorInfo,
    occupancyMap,
    nFootsteps,
    lFootOnGround,
    rFootOnGround,
    robotInMotion,
    footOnGround
   )
  DECLARE_OUTPUT_CONNECTOR(
    planningState,
    robocupRole,
    robotIntention,
    robotOnSideLine,
    localizeWithLastKnown,
    pBehaviorInfo,
    jointTemperatureSensors,
    jointCurrentSensors,
    touchSensors,
    switchSensors,
    batterySensors,
    sonarSensors,
    gameData,
    moveTarget,
    worldBallInfo
  )
public:
  /**
   * Constructor
   *
   * @param parent: Pointer to parent
   */
  #ifndef V6_CROSS_BUILD
  PlanningModule(void* parent, const ALMemoryProxyPtr& memoryProxy);
  #else
  PlanningModule(void* parent, const qi::AnyObject& memoryProxy); //! New syntax
  #endif

  /**
   * Destructor
   */
  ~PlanningModule()
  {
    delete inputConnector;
    delete outputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void init();

  /**
   * Derived from BaseModule
   */
  void mainRoutine();

  /**
   * Derived from BaseModule
   */
  void handleRequests();

  /**
   * Derived from BaseModule
   */
  void initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void setThreadPeriod();

  /**
   * Gets the pointer to path planner
   *
   * @return PathPlannerPtr
   */
  PathPlannerSpace::PathPlannerPtr getPathPlanner();

  /**
   * Updates the ball data in world using robots own and other robots data
   *
   * @return void
   */
  void
  updateWorldBallInfo();
private:
  /**
   * Updates sensor values from NaoQi ALMemory to our local
   * shared memory
   */
  void sensorsUpdate();

  /**
   * Sets up the team configuration data to be inserted in ALMemory
   * for robocup game controller
   *
   * @return void
   */
  void
  setupRoboCupDataHandler();

  //! Planning behaviors manager
  PBManagerPtr pbManager;

  //! Vector of pointer to SensorLayer objects
  vector<SensorLayerPtr> sensorLayers;

  //! Pointer to base path planner
  PathPlannerSpace::PathPlannerPtr pathPlanner;

  #ifndef V6_CROSS_BUILD
  //! Pointer to NaoQi internal memory proxy
  ALMemoryProxyPtr memoryProxy;
  #else
  qi::AnyObject memoryProxy;
  #endif

  //! Team ball tracker class object.
  boost::shared_ptr<WorldBallTracker> wbTracker;

  enum class PlanningSensors : unsigned {
    jointTemps,
    jointCurrents,
    touchSensors,
    switchSensors,
    batterSensors,
    count
  };
};
