/**
 * @file BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h
 *
 * This file defines the structs MBHeadControlConfig, 
 * HeadTargetSearchConfig and HeadTargetTrackConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/HeadControl/HeadTargetTypes.h"
#include "Utils/include/AngleDefinitions.h"

/**
 * @struct MBHeadControlConfig
 * @brief Head control behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG(
  MBHeadControlConfig,
  MBConfig,
  MBHeadControlConfigPtr,
  MBIds::headControl,
  15.0,
  MBHeadControlTypes
)

/**
 * @struct HeadScanConfig
 * @brief Head scan behavior configuration
 * @param scanLowerArea Whether to scan lower area
 * @param totalWaitTime Time to wait in one scan
 * @param scanMaxYaw Maximum head yaw range for scan
 * @param scanMaxPitch Maximum head pitch range for scan
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  HeadScanConfig,
  MBHeadControlConfig,
  MBHeadControlTypes::headScan,
  HeadScanConfigPtr,
  (bool, scanLowerArea, false),
  (float, totalWaitTime, 1.0),
  (float, scanMaxYaw, Angle::DEG_100),
  (float, scanMaxPitch, Angle::DEG_16),
)

/**
 * @struct HeadTargetTrackConfig
 * @brief Head target track behavior configuration
 * @param scanConfig HeadScanConfig
 * @param headTargetType Type of target
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  HeadTargetTrackConfig,
  MBHeadControlConfig,
  MBHeadControlTypes::headTargetTrack,
  HeadTargetTrackConfigPtr,
  (HeadScanConfigPtr, scanConfig, HeadScanConfigPtr()),
  (HeadTargetTypes, headTargetType, HeadTargetTypes::ball),
)
