/**
 * @file PlanningBehaviors/RobotStartup/Types/RequestBehavior.h
 *
 * This file declares the class RequestBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h"

struct RequestBehaviorConfig;

/** 
 * @class RequestBehavior
 * @brief The class for requesting a particular planning behavior at
 *   startup
 */
class RequestBehavior : public RobotStartup
{
public:
  /**
   * @brief RequestBehavior Constructor
   * @param planningModule Pointer to base planning module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  RequestBehavior(
    PlanningModule* planningModule, 
    const boost::shared_ptr<RequestBehaviorConfig>& config);

  /**
   * @brief ~RequestBehavior Destructor
   */
  ~RequestBehavior() final {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief update See Behavior::update()
   */
  void update() final;

  /**
   * @brief finish See Behavior::finish()
   */
  void finish() final;

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() final {}

private:
  /**
   * @brief getBehaviorCast Returns the config casted as
   *   RequestBehaviorConfig
   */
  boost::shared_ptr<RequestBehaviorConfig> getBehaviorCast();

  /**
   * @brief setStartPosture Sets the start posture given in configuration.
   *   Does nothing in case the posture is not recognized
   */ 
  void setStartPosture();
  
  //! Posture to reach on startup before requesting the desired behavior
  PostureState startPosture;

protected:
  DECLARE_FSM(fsm, RequestBehavior);
  DECLARE_FSM_STATE(RequestBehavior, SetPosture, setPosture, onRun);
  DECLARE_FSM_STATE(RequestBehavior, ChestButtonWait, chestButtonWait, onRun);
  DECLARE_FSM_STATE(RequestBehavior, StartRequested, startRequested, onRun);

  enum MBManagerIds {
    MOTION_1
  };
};

typedef boost::shared_ptr<RequestBehavior> RequestBehaviorPtr;
