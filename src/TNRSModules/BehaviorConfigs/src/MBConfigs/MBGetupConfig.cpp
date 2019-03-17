/**
 * @file MotionModule/src/MotionConfigs/MBGetupConfig.cpp
 *
 * This file implements the structs MBGetupConfig and KFMGetupConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  MBGetupConfig, MBConfig, MBGetupConfigPtr,
  (MBGetupTypes, kfmGetup, KFMGetupConfig),
)

void KFMGetupConfig::init() {}
void KFMGetupConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}



