/**
 * @file MotionModule/include/MovementModule/MovementModule.h
 *
 * This file declares the class MovementModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/MotionBehavior.h"
//#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"

struct MBMovementConfig;

/** 
 * @class MovementModule
 * @brief The base class for defining robot navigation
 */
template <typename Scalar>
class MovementModule : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief MovementModule Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  MovementModule(
    MotionModule* motionModule,
    const boost::shared_ptr<MBMovementConfig>& config,
		const string& name = "MovementModule");

  /**
   * @brief ~MovementModule Destructor
   */
  virtual ~MovementModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return boost::shared_ptr<MovementModule<Scalar> >
   */
  static boost::shared_ptr<MovementModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * @brief loadExternalConfig See Behavior::loadExternalConfig()
   */
  void loadExternalConfig() override {}

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<MovementModule<MType> > MovementModulePtr;
