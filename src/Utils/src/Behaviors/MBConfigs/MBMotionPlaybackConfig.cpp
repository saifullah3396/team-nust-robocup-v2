/**
 * @file Utils/include/Behaviors/MBConfigs/MBMotionPlaybackConfig.h
 *
 * This file defines the struct MBMotionPlaybackConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/MBConfigs/MBMotionPlaybackConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  MBMotionPlaybackConfig, MBConfig, MBMotionPlaybackConfigPtr,
  (MBMotionPlaybackTypes, replayStoredMB, ReplayStoredMBConfig),
)

void ReplayStoredMBConfig::validate()
{
  //LOG_INFO("ReplayStoredMBConfig::validate() called...:")
}
