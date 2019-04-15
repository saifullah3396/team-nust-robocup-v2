/**
 * @file MotionModule/src/MovementModule/MovementModule.cpp
 *
 * This file declares the class for the robot navigation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */


#include "MotionModule/include/MovementModule/MovementModule.h"
#include "MotionModule/include/MovementModule/Types/NaoqiFootsteps.h"
#include "MotionModule/include/MovementModule/Types/NaoqiMoveToward.h"
#include "MotionModule/include/MovementModule/Types/SpeedWalk.h"
#include "MotionModule/include/MovementModule/Types/KinResolutionWalk.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"

template <typename Scalar>
MovementModule<Scalar>::MovementModule(
  MotionModule* motionModule,
  const boost::shared_ptr<MBMovementConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template <typename Scalar>
boost::shared_ptr<MovementModule<Scalar> > MovementModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg)
{
  MovementModule<Scalar>* mm;
  switch (cfg->type) {
      case toUType(MBMovementTypes::naoqiFootsteps):
        mm = new NaoqiFootsteps<Scalar>(motionModule, SPC(NaoqiFootstepsConfig, cfg)); break;
      case toUType(MBMovementTypes::naoqiMoveToward):
        mm = new NaoqiMoveToward<Scalar>(motionModule, SPC(NaoqiMoveTowardConfig, cfg)); break;
      case toUType(MBMovementTypes::speedWalk):
        mm = new SpeedWalk<Scalar>(motionModule, SPC(SpeedWalkConfig, cfg)); break;
      case toUType(MBMovementTypes::kinResolutionWalk):
        mm = new KinResolutionWalk<Scalar>(motionModule, SPC(KinResolutionWalkConfig, cfg)); break;
  }
  return boost::shared_ptr<MovementModule<Scalar> >(mm);
}

template class MovementModule<MType>;
