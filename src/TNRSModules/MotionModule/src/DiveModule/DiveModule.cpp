/**
 * @file MotionModule/src/DiveModule/DiveModule.cpp
 *
 * This file declares the class DiveModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "MotionModule/include/DiveModule/DiveModule.h"
#include "MotionModule/include/DiveModule/Types/HandSaveDive.h"
#include "MotionModule/include/DiveModule/Types/KeyFrameMotionDive.h"

template <typename Scalar>
DiveModule<Scalar>::DiveModule(
  MotionModule* motionModule,
  const boost::shared_ptr<MBDiveConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template <typename Scalar>
boost::shared_ptr<DiveModule<Scalar> > DiveModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg)
{
  DiveModule<Scalar>* dm;
  switch (cfg->type) {
      case toUType(MBDiveTypes::kfmDive):
        dm = new KeyFrameMotionDive<Scalar>(motionModule, SPC(KFMDiveConfig, cfg)); break;
      case toUType(MBDiveTypes::handSaveDive):
        dm = new HandSaveDive<Scalar>(motionModule, SPC(HandSaveDiveConfig, cfg)); break;
  }
  return boost::shared_ptr<DiveModule<Scalar> >(dm);
}

template class DiveModule<MType>;
