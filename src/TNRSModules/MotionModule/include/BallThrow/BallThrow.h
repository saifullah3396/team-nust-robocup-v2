/**
 * @file MotionModule/include/BallThrow/BallThrow.h
 *
 * This file declares the class BallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017  
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MTypeHeader.h"

class JointCmdsRecorder;
struct MBBallThrowConfig;

/** 
 * @class BallThrow
 * @brief The base class for defining a ball throwing behavior
 */
template <typename Scalar>
class BallThrow : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief BallThrow Constructor
   *
   * @param motionModule Pointer to base motion module
   * @param config Configuration of the behavior
   * @param name Name of the behavior
   */
  BallThrow(
    MotionModule* motionModule,
    const boost::shared_ptr<MBBallThrowConfig>& config,
    const string& name = "BallThrow");

  /**
   * @brief ~BallThrow Destructor
   */
  virtual ~BallThrow() {}

  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<BallThrow<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig
   */
  virtual void loadExternalConfig() override;
protected:
  /**
   * @brief getBehaviorCast Returns the cast of config to MBBallThrowConfigPtr
	 */
  boost::shared_ptr<MBBallThrowConfig> getBehaviorCast();

  static Scalar ballRadius; ///< Ball radius
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
  
typedef boost::shared_ptr<BallThrow<MType> > BallThrowPtr;
