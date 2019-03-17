/**
 * @file BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h
 *
 * This file declares the struct MBBallThrowConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"

/**
 * @struct MBBallThrowConfig
 * @brief Ball throw behavior base configuration
 * @param timeToThrow Total throw time
 * @param headTapToStart Whether to wait for head tap
 */
DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  MBBallThrowConfig,
  MBConfig,
  MBBallThrowConfigPtr,
  MBIds::ballThrow,
  20.0,
  MBBallThrowTypes,
  (float, timeToThrow, 2.f),
  (bool, headTapToStart, false),
)

/**
 * @struct WBBallThrowConfig
 * @brief Ball throwing behavior using whole body motion
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  WBBallThrowConfig,
  MBBallThrowConfig,
  MBBallThrowTypes::wbBallThrow,
  WBBallThrowConfigPtr,
  (float, throwVelocity, 0.5),
  (float, throwAngle, 0.0),
)
