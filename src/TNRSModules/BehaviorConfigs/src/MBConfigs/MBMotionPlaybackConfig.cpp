/**
 * @file BehaviorConfigs/include/MBConfigs/MBMotionPlaybackConfig.h
 *
 * This file defines the struct MBMotionPlaybackConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBMotionPlaybackConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  MBMotionPlaybackConfig, MBConfig, MBMotionPlaybackConfigPtr,
  (MBMotionPlaybackTypes, replayStoredMB, ReplayStoredMBConfig),
)

void ReplayStoredMBConfig::init() {}
void ReplayStoredMBConfig::validate()
{
  //LOG_INFO("ReplayStoredMBConfig::validate() called...:")
}
