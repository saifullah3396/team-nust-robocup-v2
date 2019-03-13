/**
 * @file MotionModule/src/BallThrow/BallThrow.cpp
 *
 * This file implements the BallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017
 */

#include "MotionModule/include/BallThrow/BallThrow.h"
#include "MotionModule/include/BallThrow/Types/WBBallThrow.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/ConfigMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"

template <typename Scalar>
Scalar BallThrow<Scalar>::ballRadius;

template <typename Scalar>
BallThrow<Scalar>::BallThrow(
  MotionModule* motionModule,
  const boost::shared_ptr<MBBallThrowConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template <typename Scalar>
boost::shared_ptr<BallThrow<Scalar> > BallThrow<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  BallThrow<Scalar>* bt;
  switch (cfg->type) {
      case toUType(MBBallThrowTypes::wbBallThrow):
        bt = new WBBallThrow<Scalar>(motionModule, SPC(WBBallThrowConfig, cfg)); break;
  }
  return boost::shared_ptr<BallThrow<Scalar> >(bt);
}

template <typename Scalar>
MBBallThrowConfigPtr BallThrow<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast<MBBallThrowConfig> (this->config);
}

template <typename Scalar>
void BallThrow<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG(
      "EnvProperties",
      (Scalar, ballRadius, ballRadius),
    )
    loaded = true;
  }
}

template class BallThrow<MType>;
