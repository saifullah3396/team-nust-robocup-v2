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
#include <alproxies/almemoryproxy.h> ///< Not available in SDK 2.8
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
  #ifndef V6_CROSS_BUILD
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
   );
  DECLARE_OUTPUT_CONNECTOR(
    planningThreadTimeTaken,
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
  );
  #else
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
    footOnGround,
    touchSensors,
    switchSensors
   );
  DECLARE_OUTPUT_CONNECTOR(
    planningThreadTimeTaken,
    planningState,
    robocupRole,
    robotIntention,
    robotOnSideLine,
    localizeWithLastKnown,
    pBehaviorInfo,
    #ifndef REALTIME_LOLA_AVAILABLE
    jointTemperatureSensors,
    jointCurrentSensors,
    touchSensors,
    switchSensors,
    batterySensors,
    sonarSensors,
    #endif
    gameData,
    moveTarget,
    worldBallInfo
  );
  #endif
public:
  /**
   * Constructor
   *
   * @param parent: Pointer to parent
   */
  #ifndef V6_CROSS_BUILD
    PlanningModule(void* parent, const ALMemoryProxyPtr& memoryProxy);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      PlanningModule(void* parent, const qi::AnyObject& memoryProxy); ///< New syntax
    #else
      PlanningModule(void* parent); ///< New syntax
    #endif
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
   * @brief init See BaseModule::init()
   */
  void init() final;

  /**
   * @brief mainRoutine See BaseModule::mainRoutine()
   */
  void mainRoutine() final;

  /**
   * @brief handleRequests See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * @brief initMemoryConn See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * @brief setThreadPeriod See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

  /**
   * @brief setThreadTimeTaken See BaseModule::setThreadTimeTaken()
   */
  void setThreadTimeTaken() final;

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
  void updateWorldBallInfo();
private:
  #ifndef V6_CROSS_BUILD
    /**
     * Updates sensor values from NaoQi ALMemory to our local
     * shared memory
     */
    void sensorsUpdate();
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      /**
       * Updates sensor values from NaoQi ALMemory to our local
       * shared memory
       */
      void sensorsUpdate();
    #endif
  #endif

  #ifndef V6_CROSS_BUILD
  /**
   * Sets up the team configuration data to be inserted in ALMemory
   * for robocup game controller
   *
   * @return void
   */
  void setupRoboCupDataHandler();
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      void setupRoboCupDataHandler();
    #endif
  #endif

  ///< Planning behaviors manager
  PBManagerPtr pbManager;

  #ifndef V6_CROSS_BUILD
    ///< Vector of pointer to SensorLayer objects
    vector<SensorLayerPtr> sensorLayers;
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      ///< Vector of pointer to SensorLayer objects
      vector<SensorLayerPtr> sensorLayers;
    #endif
  #endif

  ///< Pointer to base path planner
  PathPlannerSpace::PathPlannerPtr pathPlanner;

  #ifndef V6_CROSS_BUILD
    ///< Pointer to NaoQi internal memory proxy
    ALMemoryProxyPtr memoryProxy;
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      qi::AnyObject memoryProxy;
    #endif
  #endif

  ///< Team ball tracker class object.
  boost::shared_ptr<WorldBallTracker> wbTracker;

  #ifndef V6_CROSS_BUILD
    enum class PlanningSensors : unsigned {
      jointTemps,
      jointCurrents,
      touchSensors,
      switchSensors,
      batterSensors,
      count
    };
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      enum class PlanningSensors : unsigned {
        jointTemps,
        jointCurrents,
        touchSensors,
        switchSensors,
        batterSensors,
        count
      };
    #endif
  #endif
};
