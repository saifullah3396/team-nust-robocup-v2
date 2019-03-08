/**
 * @file MotionModule/src/MotionPlayback/MotionPlayback.cpp
 *
 * This file declares the class MotionPlayback
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#include "MotionModule/include/MotionBehaviorIds.h"
#include "MotionModule/include/MotionPlayback/MotionPlayback.h"
#include "MotionModule/include/MotionPlayback/Types/ReplayStoredMB.h"
#include "Utils/include/Behaviors/MBConfigs/MBMotionPlaybackConfig.h"

template <typename Scalar>
MotionPlayback<Scalar>::MotionPlayback(
  MotionModule* motionModule,
  const boost::shared_ptr<MBMotionPlaybackConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template <typename Scalar>
boost::shared_ptr<MotionPlayback<Scalar> > MotionPlayback<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  MotionPlayback<Scalar>* dm;
  switch (cfg->type) {
      case toUType(MBMotionPlaybackTypes::replayStoredMB):
        dm = new ReplayStoredMB<Scalar>(motionModule, SPC(ReplayStoredMBConfig, cfg)); break;
  }
  return boost::shared_ptr<MotionPlayback<Scalar> >(dm);
}

template class MotionPlayback<MType>;
