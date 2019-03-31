/**
 * @file MotionModule/BallThrow/Types/WBBallThrow.h
 *
 * This file declares the class WBBallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "MotionModule/include/BallThrow/BallThrow.h"
#include "MotionModule/include/TrajectoryPlanner/JointInterpolator.h"

struct WBBallThrowConfig;

/**
 * @class WBBallThrow
 * @brief A class for defining a ball throw based on whole body motion
 *   of the robot
 */
template <typename Scalar>
class   WBBallThrow : public BallThrow<Scalar>, public JointInterpolator<Scalar>
{
public:
  /**
   * @brief WBBallThrow Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  WBBallThrow(
    MotionModule* motionModule,
    const boost::shared_ptr<WBBallThrowConfig>& config);

  /**
   * @brief ~WBBallThrow Destructor
   */
  ~WBBallThrow()
  {
  }

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
private:
  /**
   * @brief getBehaviorCast Returns the cast of its
   *   config as WBBallThrowConfig
   */
  boost::shared_ptr<WBBallThrowConfig> getBehaviorCast();

  /**
   * @brief executeArmsTrajs Executes the planned arm trajectories
   * @param jointTrajectories Input arm trajectories
   * @param stepTime Time step of update
   */
  void executeArmsTrajs(
    const vector<vector<Scalar> >& jointTrajectories,
    const Scalar& stepTime);

  //! Finite state machine for this behavior
  DECLARE_FSM(fsm, WBBallThrow<Scalar>)

  //! GrabBall: Grab ball action
  DECLARE_FSM_STATE(WBBallThrow<Scalar>, GrabBall, grabBall, onStart, onRun,)

  //! Retract: Ball retraction action
  DECLARE_FSM_STATE(WBBallThrow<Scalar>, Retract, retract, onStart, onRun,)

  //! ThrowBall: Throw ball action
  DECLARE_FSM_STATE(WBBallThrow<Scalar>, ThrowBall, throwBall, onStart, onRun,)

  //! Joint trajectories from cubic spline
  vector < vector<Scalar> > jointTrajectories;

  //! Time to grab the ball
  Scalar timeToGrab = {1.0};

  //! Time to retract
  Scalar timeToRetract = {3.0};

  //! Time to throw
  Scalar timeToThrow = {1.0};

  //! Time of execution of a state
  Scalar execTime = {0.0};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<WBBallThrow<MType> > WBBallThrowPtr;
