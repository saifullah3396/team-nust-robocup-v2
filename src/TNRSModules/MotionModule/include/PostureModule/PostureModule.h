/**
 * @file MotionModule/include/PostureModule/PostureModule.h
 *
 * This file declares the class PostureModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 13 May 2017 
 */

#pragma once

#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MTypeHeader.h"

enum class PostureState : unsigned;
struct MBPostureConfig;

/** 
 * @class PostureModule
 * @brief The class for sending the robot to predefined postures
 */
template <typename Scalar>
class PostureModule : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief PostureModule Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  PostureModule(
    MotionModule* motionModule,
    const boost::shared_ptr<MBPostureConfig>& config,
    const string& name = "PostureModule");
  

  /**
   * @brief ~PostureModule Destructor
   */
  virtual ~PostureModule() {}
  
  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<PostureModule<Scalar> >
   */ 
  static boost::shared_ptr<PostureModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

protected:
  /**
	 * Returns the cast of config to MBPostureConfigPtr
	 */ 
  boost::shared_ptr<MBPostureConfig> getBehaviorCast();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<PostureModule<MType> > PostureModulePtr;
