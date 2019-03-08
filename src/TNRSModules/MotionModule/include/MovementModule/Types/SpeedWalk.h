/**
 * @file MotionModule/MovementModule/Types/SpeedWalk.h
 *
 * This file declares the class SpeedWalk
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <deque>
#include "BehaviorManager/include/StateMachineMacros.h"
#include "MotionModule/include/MovementModule/MovementModule.h"

template <typename Scalar>
struct WalkParameters;
template <typename Scalar>
class WalkZmpRefGen;
template <typename Scalar>
struct TNRSFootstep;
struct SpeedWalkConfig;

/**
 * @class SpeedWalk
 * @brief A class for that defines a robot walk based on input desired speed
 */
template <typename Scalar>
class SpeedWalk : public MovementModule<Scalar>
{
public:
  /**
   * @brief SpeedWalk Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  SpeedWalk(
    MotionModule* motionModule,
    const boost::shared_ptr<SpeedWalkConfig>& config);

  /**
   * @brief ~SpeedWalk Destructor
   */
  ~SpeedWalk() final {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

  /**
   * @brief reinitiate See Behavior::reinitiate()
   * @param cfg New configuration
   */
  virtual void reinitiate(const BehaviorConfigPtr& cfg);

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
   * @brief getBehaviorCast Returns the cast of config to SpeedWalkConfigPtr
   */
  boost::shared_ptr<SpeedWalkConfig> getBehaviorCast();
  boost::shared_ptr<WalkParameters<Scalar> > params;

  void addStep(
    const boost::shared_ptr<TNRSFootstep<Scalar> >& fs);
  boost::shared_ptr<TNRSFootstep<Scalar> >& getFrontStep();
  boost::shared_ptr<TNRSFootstep<Scalar> >& getBackStep();
  void popStep();

  /**
   * @brief computeStepFromTo Computes the transformation of a robot step
   *   from other leg step and returns the step
   * @param from The step from which transformation is generated
   * @param to The new step by other leg
   * @param transFromTo Transformation matrix
   * @return TNRSFootstep
   */
  TNRSFootstep<Scalar> computeStepFromTo(
    const TNRSFootstep<Scalar>& from,
    const TNRSFootstep<Scalar>& to,
    Eigen::Matrix<Scalar, 4, 4>& transFromTo);

  /**
   * @brief stepVelToPose Generates a step update based on
   *   velocity input
   * @param vi Desired normalized velocity of the robot
   * @return Step within constraints for given velocity
   */
  RobotPose2D<Scalar> stepVelToPose(const VelocityInput<Scalar>& vi);

  /**
   * @brief addFirstStep Add first step of robot using the initial position
   *   of the step from global base chain
   */
  void addFirstStep();

  /**
   * @brief genNextStep Generates the next step in sequence based on
   *   desired velocity input and updates the steps queue
   */
  void genNextStep();


  //! Finite state machine for this behavior
  DECLARE_FSM(fsm, SpeedWalk<Scalar>)

  //! PlanStep: Plans where should the next two steps be placed
  DECLARE_FSM_STATE(SpeedWalk<Scalar>, PlanSteps, planSteps, onRun,)

  //! ExecuteDSS: Executes the current step motion for double support phase
  DECLARE_FSM_STATE(SpeedWalk<Scalar>, ExecuteDSS, executeDSS, onRun,)

  //! ExecuteSSS: Executes the current step motion for single support phase
  DECLARE_FSM_STATE(SpeedWalk<Scalar>, ExecuteSSS, executeSSS, onRun,)

  //! Foot steps queue
  std::deque<boost::shared_ptr<TNRSFootstep<Scalar>>> stepsQueue;

  //! Walk zmp reference generator
  boost::shared_ptr<WalkZmpRefGen<Scalar> > walkZmpRefGen;
};

typedef boost::shared_ptr<SpeedWalk<MType> > SpeedWalkPtr;
