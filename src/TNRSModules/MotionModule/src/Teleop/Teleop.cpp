/**
 * @file MotionModule/src/Teleop/Teleop.cpp
 *
 * This file implements the class Teleop
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 13 May 2017
 */

#include "MotionModule/include/Teleop/Teleop.h"
#include "MotionModule/include/Teleop/Types/TeleopJoints.h"
#include "BehaviorConfigs/include/MBConfigs/MBTeleopConfig.h"

template <typename Scalar>
Teleop<Scalar>::Teleop(
  MotionModule* motionModule,
  const boost::shared_ptr<MBTeleopConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template <typename Scalar>
boost::shared_ptr<Teleop<Scalar> > Teleop<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg)
{
  Teleop<Scalar>* pm;
  switch (cfg->type) {
      case toUType(MBTeleopTypes::teleopJoints):
        pm = new TeleopJoints<Scalar>(motionModule, SPC(TeleopJointsConfig, cfg)); break;
  }
  return boost::shared_ptr<Teleop<Scalar> >(pm);
}

template class Teleop<MType>;
