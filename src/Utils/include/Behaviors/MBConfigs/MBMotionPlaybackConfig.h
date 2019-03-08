/**
 * @file MotionModule/include/MBConfigs/MBMotionPlaybackConfig.h
 *
 * This file defines the struct MBMotionPlaybackConfig and its types
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"

DECLARE_BEHAVIOR_CONFIG(
  MBMotionPlaybackConfig,
  MBConfig,
  MBMotionPlaybackConfigPtr,
  MBIds::motionPlayback,
  9999.f,
  MBMotionPlaybackTypes
)

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  ReplayStoredMBConfig,
  MBMotionPlaybackConfig,
  MBMotionPlaybackTypes::replayStoredMB,
  ReplayStoredMBConfigPtr,
  (string, pathToMB, string("")),
  (vector<unsigned>, activeJoints, vector<unsigned>(24, 1)),
)
