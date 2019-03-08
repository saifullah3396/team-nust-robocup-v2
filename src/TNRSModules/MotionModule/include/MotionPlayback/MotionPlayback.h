/**
 * @file MotionModule/include/MotionPlayback/MotionPlayback.h
 *
 * This file declares the class MotionPlayback
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017 
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/MotionBehavior.h"

struct MBMotionPlaybackConfig;

/** 
 * @class MotionPlayback
 * @brief The class for generating motion by playing back stored motion
 *   behaviors 
 */
template <typename Scalar>
class MotionPlayback : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief MotionPlayback Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  MotionPlayback(
    MotionModule* motionModule,
    const boost::shared_ptr<MBMotionPlaybackConfig>& config,
    const string& name = "MotionPlayback");

  /**
   * @brief ~MotionPlayback Destructor
   */
  virtual ~MotionPlayback() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<MotionPlayback<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

protected:
  //! The final posture state after dive
  PostureState endPosture;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<MotionPlayback<MType> > MotionPlaybackPtr;
