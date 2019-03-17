/**
 * @file BehaviorConfigs/include/MBConfigs/MBDiveConfig.h
 *
 * This file declares the structs MBDiveConfig and KFMDiveConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct MBDiveConfig
 * @brief Dive behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG(
  MBDiveConfig,
  MBConfig,
  MBDiveConfigPtr,
  MBIds::dive,
  10.0,
  MBDiveTypes
)

/**
 * @struct KFMDiveConfig
 * @brief Key-frame motion based dive behavior configuration
 * @param keyFrameDiveType Type of dive motion
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  KFMDiveConfig,
  MBDiveConfig,
  MBDiveTypes::kfmDive,
  KFMDiveConfigPtr,
  (KeyFrameDiveTypes, keyFrameDiveType, KeyFrameDiveTypes::inPlace),
)

/**
 * @struct HandSaveDiveConfig
 * @brief Whole body motion based dive with hand movement for saving
 * @param supportLeg Support reference frame for diving
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  HandSaveDiveConfig,
  MBDiveConfig,
  MBDiveTypes::handSaveDive,
  HandSaveDiveConfigPtr,
  (LinkChains, supportLeg, LinkChains::lLeg),
)
