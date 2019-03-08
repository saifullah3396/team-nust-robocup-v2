/**
 * @file MotionModule/include/MBConfigs/MBTeleopConfig.h
 *
 * This file defines the struct MBTeleopConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MotionModule/include/MotionBehaviorIds.h"
#include "Utils/include/Behaviors/MBConfigs/MBConfig.h"
#include "Utils/include/Behaviors/BehaviorConfigMacros.h"

DECLARE_BEHAVIOR_CONFIG(
  MBTeleopConfig,
  MBConfig,
  MBTeleopConfigPtr,
  MBIds::teleop,
  9999.f,
  MBTeleopTypes
)

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  TeleopJointsConfig,
  MBTeleopConfig,
  MBTeleopTypes::teleopJoints,
  TeleopJointsConfigPtr,
  (vector<float>, jointCommands, vector<float>()),
)
