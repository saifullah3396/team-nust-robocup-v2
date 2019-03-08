/**
 * @file PlanningModule/PlanningBehaviors/GoalKeeper.h
 *
 * This file declares the class GoalKeeper.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017 
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviors/Robocup/Types/Soccer.h"

/** 
 * @class GoalKeeper
 * @brief Class for defining the goal keeper behavior.
 */
class GoalKeeper : public Soccer
{
public:
  /**
   * Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   */
  GoalKeeper(
    PlanningModule* planningModule,
    const BehaviorConfigPtr& config) :
    Soccer(planningModule, config, "GoalKeeper"),
    interceptTarget(RobotPose2D<float>(NAN, NAN, NAN)),
    lastDiveTargetType(-1)
  {
    DEFINE_FSM_STATE(Soccer, ReactGoalKeeper, react)
    DEFINE_FSM_STATE(Soccer, Dive, dive)
    DEFINE_FSM_STATE(Soccer, PostDive, postDive)
    DEFINE_FSM_STATE(Soccer, InterceptBall, interceptBall)
  }

  /**
   * Default destructor for this class.
   */
  ~GoalKeeper()
  {
  }

  /**
   * Derived from Behavior
   */
  bool initiate() final;

private:
  /**
   * Returns the config casted as GoalKeeperConfigPtr
   */
  boost::shared_ptr<GoalKeeperConfig> getBehaviorCast();

protected:
  struct ReactGoalKeeper : public React
  {
    ReactGoalKeeper(GoalKeeper* bPtr) : React(bPtr) {}
    virtual void onRun() final;
  };
  DECLARE_FSM_STATE(Soccer, Dive, dive, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, PostDive, postDive, onRun,)
  DECLARE_FSM_STATE(Soccer, InterceptBall, interceptBall, onStart, onRun,)

  bool ballInRange() final;
  bool ballIncoming(Vector2f& endPosition, float& timeToReach);
  bool divePossible();

  RobotPose2D<float> interceptTarget;
  int lastDiveTargetType;

  const float interceptBallPosTol = 0.05f; //! 0.1 meters
  const float diveReactionTime = 10.f; //! 5 secs
};

typedef boost::shared_ptr<GoalKeeper> GoalKeeperPtr;
