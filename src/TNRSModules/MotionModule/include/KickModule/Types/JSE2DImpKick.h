/**
 * @file MotionModule/include/KickModule/JSE2DImpKick.h
 *
 * This file declares the class JSE2DImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include <json/json.h>
#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"

template <typename Scalar>
class KickImpact2DSolver;
class JointCmdsRecorder;
struct JSE2DImpKickConfig;

/**
 * @class JSE2DImpKick
 * @brief An omni-directional kicking behavior that uses joint-space
 *   optimization and momentum conservation in 2D to generate kicking motion
 *   on estimated state of an incoming ball
 */
template <typename Scalar>
class JSE2DImpKick : public JointSpaceKick<Scalar>
{
public:
  /**
   * @brief JSE2DImpKick Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  JSE2DImpKick(
    MotionModule* motionModule,
    const boost::shared_ptr<JSE2DImpKickConfig>& config);

  /**
   * @brief ~JSE2DImpKick Destructor
   */
  ~JSE2DImpKick() final {}

  /**
   * @brief initiate See Behavior::initiate()
   */
  bool initiate() final;

private:
  /**
   * @brief getBehaviorCast Returns the cast of config to JSE2DImpKickConfigPtr
   */ 
  boost::shared_ptr<JSE2DImpKickConfig> getBehaviorCast();

  struct JSE2DPlanKick : public JointSpaceKick<Scalar>::PlanKick
  {
    JSE2DPlanKick(JSE2DImpKick<Scalar>* bPtr) :
      JointSpaceKick<Scalar>::PlanKick(bPtr)
    {}
    virtual void onRun() final;
  };

  DECLARE_FSM_STATE(JointSpaceKick<Scalar>, WaitForExecution, waitForExecution, onRun,)

  /**
   * @brief setupKickBase See JointSpaceKick::setupKickBase()
   */ 
  void setupKickBase() final;
  
  /**
   * @brief solveForImpact Solves for the best impact conditions
   *   to find impact pose and velocity of the end-effector
   */ 
  void solveForImpact() final;
  
  //! Ball initial velocity
  Matrix<Scalar, 3, 1> ballVelocity;
  
  //! Best impact conditions solver
  boost::shared_ptr<KickImpact2DSolver<Scalar>> kickImpact2DSolver;
  friend class KickImpact2DSolver<Scalar>;
};

typedef boost::shared_ptr<JSE2DImpKick<MType> > JSE2DImpKickPtr;
