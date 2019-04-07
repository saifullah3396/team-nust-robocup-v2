/**
 * @file MotionModule/include/BalanceModule/Types/PIDComControl.h
 *
 * This file declares the class PIDComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"

struct PIDComControlConfig;

/**
 * @class PIDComControl
 * @brief The class for controlling the center of mass movement through
 *   a PID controller
 */
template <typename Scalar>
class PIDComControl : public BalanceModule<Scalar>
{
public:
  /**
   * @brief PIDComControl Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  PIDComControl(
    MotionModule* motionModule,
    const boost::shared_ptr<PIDComControlConfig>& config);

  /**
   * @brief ~PIDComControl Destructor
   */
  ~PIDComControl() final {}

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
  void loadExternalConfig() final {}

private:
  /**
   * @brief getBehaviorCast Returns the cast of config to PIDComControlConfigPtr
   */
  boost::shared_ptr<PIDComControlConfig> getBehaviorCast();
};

typedef boost::shared_ptr<PIDComControl<MType> > PIDComControlPtr;
