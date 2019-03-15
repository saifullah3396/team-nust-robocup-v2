/**
 * @file MotionModule/src/GetupModule/GetupModule.cpp
 *
 * This file implements the class GetupModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "MotionModule/include/GetupModule/GetupModule.h"
#include "MotionModule/include/GetupModule/Types/KeyFrameMotionGetup.h"

template <typename Scalar>
boost::shared_ptr<GetupModule<Scalar> > GetupModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  GetupModule<Scalar>* gm;
  switch (cfg->type) {
      case toUType(MBGetupTypes::kfmGetup):
        gm = new KeyFrameMotionGetup<Scalar>(motionModule, SPC(KFMGetupConfig, cfg)); break;
  }
  return boost::shared_ptr<GetupModule<Scalar> >(gm);
}

template class GetupModule<MType>;
