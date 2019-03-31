/**
 * @file MotionModule/src/BalanceModule/BalanceModule.cpp
 *
 * This file implements the class BalanceModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 April 2017
 */

#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/BalanceModule/Types/MPComControl.h"
#include "MotionModule/include/BalanceModule/Types/PIDComControl.h"
#include "MotionModule/include/BalanceModule/Types/ZmpControl.h"
#include "Utils/include/ConfigManager.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"

template <typename Scalar>
BalanceModule<Scalar>::BalanceModule(
  MotionModule* motionModule,
  const boost::shared_ptr<MBBalanceConfig>& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name)
{
}

template<typename Scalar>
boost::shared_ptr<BalanceModule<Scalar> > BalanceModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg)
{
  BalanceModule<Scalar>* bm;
  switch (cfg->type) {
      case toUType(MBBalanceTypes::mpComControl):
        bm = new MPComControl<Scalar>(motionModule, SPC(MPComControlConfig, cfg)); break;
      case toUType(MBBalanceTypes::pidComControl):
        bm = new PIDComControl<Scalar>(motionModule, SPC(PIDComControlConfig, cfg)); break;
      case toUType(MBBalanceTypes::zmpControl):
        bm = new ZmpControl<Scalar>(motionModule, SPC(ZmpControlConfig, cfg)); break;
  }
  return boost::shared_ptr<BalanceModule<Scalar> >(bm);
}

template class BalanceModule<MType>;
