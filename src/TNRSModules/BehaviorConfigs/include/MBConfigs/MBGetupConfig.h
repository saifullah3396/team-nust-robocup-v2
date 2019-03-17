/**
 * @file BehaviorConfigs/include/MBConfigs/MBGetupConfig.h
 *
 * This file defines the structs MBGetupConfig and KFMGetupConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MotionModule/include/GetupModule/KeyFrameGetupTypes.h"
#include "MBConfig.h"

/**
 * @struct MBGetupConfig
 * @brief Getup motion behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG(
  MBGetupConfig,
  MBConfig,
  MBGetupConfigPtr,
  MBIds::getup,
  20.0,
  MBGetupTypes
)

/**
 * @struct KFMGetupConfig
 * @brief Key-frame motion based getup behavior configuration
 * @param keyFrameGetupType Type of getup motion
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  KFMGetupConfig,
  MBGetupConfig,
  MBGetupTypes::kfmGetup,
  KFMGetupConfigPtr,
  (KeyFrameGetupTypes, keyFrameGetupType, KeyFrameGetupTypes::back),
)
