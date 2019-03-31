/**
 * @file MotionModule/src/MotionConfigs/MBHeadControlConfig.cpp
 *
 * This file implements the structs MBHeadControlConfig,
 * HeadTargetSearchConfig and HeadTargetTrackConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  MBHeadControlConfig, MBConfig, MBHeadControlConfigPtr,
  (MBHeadControlTypes, headScan, HeadScanConfig),
  (MBHeadControlTypes, headTargetTrack, HeadTargetTrackConfig),
)

void HeadScanConfig::init() {}
void HeadScanConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}

void HeadTargetTrackConfig::init() {}
void HeadTargetTrackConfig::validate()
{
  //! Throw a BConfigException is behavior configuration is invalid
}

