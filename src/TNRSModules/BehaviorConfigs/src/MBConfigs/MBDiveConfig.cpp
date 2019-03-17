/**
 * @file MotionModule/src/MotionConfigs/MBDiveConfig.cpp
 *
 * This file implements the structs MBDiveConfig and KFMDiveConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/DataUtils.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"

DEFINE_BEHAVIOR_CONFIG(
  MBDiveConfig, MBConfig, MBDiveConfigPtr,
  (MBDiveTypes, kfmDive, KFMDiveConfig),
  (MBDiveTypes, handSaveDive, HandSaveDiveConfig),
)

void KFMDiveConfig::init() {}
void KFMDiveConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}

void HandSaveDiveConfig::init() {}
void HandSaveDiveConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}
