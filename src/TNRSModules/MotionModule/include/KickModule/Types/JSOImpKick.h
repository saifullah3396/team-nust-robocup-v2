/**
 * @file MotionModule/include/KickModule/Types/JSOImpKick.h
 *
 * This file declares the class JSOImpKick
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Jul 2018
 */

#pragma once

#include "MotionModule/include/KickModule/Types/JointSpaceKick.h"
#include "BehaviorManager/include/StateMachineMacros.h"

class JointCmdsRecorder;
template <typename Scalar>
class MaxMomentumEEOpt;
struct JSOImpKickConfig;
struct ZmpControlConfig;

/**
 * @class JSOImpKick
 * @brief An omni-directional kicking behavior that uses joint-space
 *   optimization and impulse manipulation for sending the ball to a desired
 *   location
 */
template <typename Scalar>
class JSOImpKick : public JointSpaceKick<Scalar>
{
public:
  /**
   * @brief JSOImpKick Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  JSOImpKick(
    MotionModule* motionModule,
    const boost::shared_ptr<JSOImpKickConfig>& config);

  /**
   * @brief ~JSOImpKick Destructor
   */
  ~JSOImpKick() final {}

private:
  /**
   * @brief getBehaviorCast Casts the behavior configuration to JSOImpKickConfig
   * @return boost::shared_ptr<JSOImpKickConfig>
   */ 
  boost::shared_ptr<JSOImpKickConfig> getBehaviorCast();

  /**
   * @brief setupKickBase See JSKickConfig::setupKickBase
   */
  void setupKickBase() final;

  /**
   * @brief solveForImpact Finds best the end-effector point and
   *   orientation based on maximum mass*velocity product and
   *   x-coordinate on the foot surface
   */
  void solveForImpact() final;

  ///< Best end-effector solver
  boost::shared_ptr<MaxMomentumEEOpt<Scalar>> maxMomentumEEOpt;
};

typedef boost::shared_ptr<JSOImpKick<MType>> JSOImpKickPtr;
