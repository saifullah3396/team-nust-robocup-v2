/**
 * @file MotionModule/include/MovementModule/Types/NaoqiMoveTo.h
 *
 * This file declares the class NaoqiMoveTo
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "BehaviorManager/include/StateMachineMacros.h"
#include "MotionModule/include/MovementModule/MovementModule.h"

struct NaoqiMoveToConfig;

/**
 * @class NaoqiMoveTo
 * @brief A class for defining movement of the robot using naoqi based
 *   moveTo() that moves robot to the desired relative position
 */
template <typename Scalar>
class NaoqiMoveTo : public MovementModule<Scalar>
{
public:
  /**
   * @brief NaoqiMoveTo Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  NaoqiMoveTo(
    MotionModule* motionModule,
    const boost::shared_ptr<NaoqiMoveToConfig>& config);

  /**
   * @brief ~NaoqiMoveTo Destructor
   */
  ~NaoqiMoveTo() final {}
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
  void loadExternalConfig() final;

private:
  /**
   * @brief getBehaviorCast Casts the config to NaoqiMoveToConfig
   * @return boost::shared_ptr<NaoqiMoveToConfig>
   */
  boost::shared_ptr<NaoqiMoveToConfig> getBehaviorCast();

  ///< Finite state machine for this behavior
  DECLARE_FSM(fsm, NaoqiMoveTo<Scalar>)

  ///< SetPosture: State for setting initial posture
  DECLARE_FSM_STATE(NaoqiMoveTo<Scalar>, SetPosture, setPosture, onStart, onRun,)

  ///< MoveTo: Move to state
  DECLARE_FSM_STATE(NaoqiMoveTo<Scalar>, MoveTo, moveTo, onStart, onRun,)

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  AL::ALValue moveConfig;
  const float frequency = 1.0; // Minimum from naoqi motion config
  const float minStepPeriod = 0.42; // Minimum from naoqi motion config
  const float maxStepPeriod = 0.6; // Minimum from naoqi motion config
  Matrix<Scalar, 4, 4> lastOdomTrans = Matrix<Scalar, 4, 4>::Zero();
  #endif
};

typedef boost::shared_ptr<NaoqiMoveTo<MType> > NaoqiMoveToPtr;
