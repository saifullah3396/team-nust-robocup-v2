/**
 * @file PlanningModule/PlanningBehaviors/Robocup/Types/Soccer.h
 *
 * This file declares the class Soccer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 May 2017
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "PlanningModule/include/PlanningBehaviors/Robocup/Robocup.h"

/**
 * @class Soccer
 * @brief Class for defining the soccer gameplay
 */
class Soccer : public Robocup
{
public:
  /**
   * Constructor
   *
   * @param planningModule: pointer to parent planning module
   * @param config: Configuration of this behavior
   * @param name: Name of this behavior
   */
  Soccer(
    PlanningModule* planningModule,
    const BehaviorConfigPtr& config,
    const string& name = "Soccer") :
    Robocup(planningModule, config, name),
    lastKickTarget(Point2f(0.f, 0.f)),
    goingToTarget(GoingToTarget::NONE),
    goalPosTol(0.25f), //! 0.25 meters
    goalAngleTol(0.261666667), //! 15 degrees in radians
    ballKickDist(0.25f), //! 0.25 meters
    ballMovedVelMin(0.5f)//! 0.5 meters
  {
    DEFINE_FSM_STATE(Soccer, SetPosture, setPosture)
    DEFINE_FSM_STATE(Soccer, Localize, localize)
    DEFINE_FSM_STATE(Soccer, GoToPosition, goToPosition)
    DEFINE_FSM_STATE(Soccer, PlayBall, playBall)
    DEFINE_FSM_STATE(Soccer, AlignToKick, alignToKick)
    DEFINE_FSM_STATE(Soccer, KickBall, kickBall)
    DEFINE_FSM_STATE(Soccer, FallRecovery, fallRecovery)
    DEFINE_FSM_STATE(Soccer, Getup, getup)
    DEFINE_FSM_STATE(Soccer, WaitForPenalty, waitForPenalty)
    DEFINE_FSM(fsm, Soccer, setPosture)
  }

  virtual ~Soccer() {}

  bool initiate();
  void update() final;
  void finish() final;
  void loadExternalConfig() final;
protected:
  DECLARE_FSM(fsm, Soccer)
  DECLARE_FSM_STATE(Soccer, Localize, localize, onStart, onRun, onStop,)
  DECLARE_FSM_STATE(Soccer, SetPosture, setPosture, onRun)
  struct React : public FSMState<Soccer>
  {
    React(Soccer* bPtr) : FSMState<Soccer>(bPtr) {}
    virtual void onRun() = 0;
  };
  unique_ptr<React> react;
  DECLARE_FSM_STATE(Soccer, GoToPosition, goToPosition, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, PlayBall, playBall, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, AlignToKick, alignToKick, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, KickBall, kickBall, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, FallRecovery, fallRecovery, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, Getup, getup, onStart, onRun,)
  DECLARE_FSM_STATE(Soccer, WaitForPenalty, waitForPenalty, onStart, onRun,)

  //! Pure
  virtual bool shouldPlayBall();
  virtual bool ballInRange() = 0;
  virtual void goToBall(const float& distFromBall);
  virtual Point2f findBallKickTarget();
  virtual void findBestBallAlignment(RobotPose2D<float>& alignPosition);
  virtual bool alignedToKick();
  virtual bool behindObstacle(const Point2f& target);

  const float goalPosTol = 0.25f; //! 0.25 meters
  const float goalAngleTol = 0.261666667; //! 15 degrees in radians
  const float ballKickDist = 0.25f; //! 0.2 meters
  const float ballMovedVelMin = 0.5f; //! 0.5 meters / s
  Point2f lastKickTarget;

  static float coeffDamping;
  static float coeffFriction;

  enum class GoingToTarget : unsigned {
    initPosition,
    ball,
    kickAlignment,
    interceptBall,
    none
  } goingToTarget;

private:
  boost::shared_ptr<PBRobocupConfig> getBehaviorCast();
};
