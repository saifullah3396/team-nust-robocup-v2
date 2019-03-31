/**
 * @file MotionModule/BalanceModule/Types/MPComControl.h
 *
 * This file declares the class MPComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/TrajectoryPlanner/JointInterpolator.h"

struct MPComControlConfig;

/**
 * @class MPComControl
 * @brief The class for sending the robot to a predefined balanced posture
 */
template <typename Scalar>
class MPComControl : public BalanceModule<Scalar>, public JointInterpolator<Scalar>
{
public:
  /**
   * @brief MPComControl Constructor
   * @param motionModule Pointer to base motion module
   * @param config Configuration of this behavior
   */
  MPComControl(
    MotionModule* motionModule,
    const boost::shared_ptr<MPComControlConfig>& config);

  /**
   * @brief ~MPComControl Destructor
   */
  ~MPComControl() {}

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
   * @brief getBehaviorCast Returns the cast of config to MPComControlConfigPtr
   */
  boost::shared_ptr<MPComControlConfig> getBehaviorCast();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<MPComControl<MType> > MPComControlPtr;
