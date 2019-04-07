/**
 * @file TNRSModules/BehaviorConfigs/include/MBConfigs/MBTeleopConfig.h
 *
 * This file defines the struct MBTeleopConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MotionModule/include/MotionBehaviorIds.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "BehaviorConfigs/include/BehaviorConfigMacros.h"

/**
 * @struct MBTeleopConfig
 * @brief Teleop behavior base configuration
 */
DECLARE_BEHAVIOR_CONFIG(
  MBTeleopConfig,
  MBConfig,
  MBTeleopConfigPtr,
  MBIds::teleop,
  9999.f,
  MBTeleopTypes
);

/**
 * @struct TeleopJointsConfig
 * @brief Used to send and update joint commands based on user input
 */
DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  TeleopJointsConfig,
  MBTeleopConfig,
  MBTeleopTypes::teleopJoints,
  TeleopJointsConfigPtr,
  (vector<float>, jointCommands, vector<float>()),
);
