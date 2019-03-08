/**
 * @file MotionModule/include/BalanceModule/BalanceModule.h
 *
 * This file declares the class BalanceModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 April 2017
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/MotionBehavior.h"

/**
 * @class BalanceModule
 * @brief The class for controlling the stability of the robot.
 */
template <typename Scalar>
class BalanceModule : public MotionBehavior<Scalar>
{
public:
  /**
   * @brief BalanceModule Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   * @param name Behavior name
   */
  BalanceModule(
    MotionModule* motionModule,
    const boost::shared_ptr<MBBalanceConfig>& config,
    const string& name = "BalanceModule");


  /**
   * @brief ~BalanceModule Destructor
   */
  virtual ~BalanceModule() {}

  /**
   * @brief getType Returns its own child based on the given type
   *
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   *
   * @return boost::shared_ptr<BalanceModule<Scalar> >
   */
  static boost::shared_ptr<BalanceModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BalanceModule<MType> > BalanceModulePtr;
