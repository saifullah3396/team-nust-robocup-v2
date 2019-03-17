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

/**
 * @struct MBMotionPlaybackConfig
 * @brief Motion playback behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG(
  MBMotionPlaybackConfig,
  MBConfig,
  MBMotionPlaybackConfigPtr,
  MBIds::motionPlayback,
  9999.f,
  MBMotionPlaybackTypes
)

/**
 * @struct ReplayStoredMBConfig
 * @brief Replays stored motion commands based on logged
 *   motion behavior configurations
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  ReplayStoredMBConfig,
  MBMotionPlaybackConfig,
  MBMotionPlaybackTypes::replayStoredMB,
  ReplayStoredMBConfigPtr,
  (string, pathToMB, string("")),
  (vector<unsigned>, activeJoints, vector<unsigned>(24, 1)),
)
