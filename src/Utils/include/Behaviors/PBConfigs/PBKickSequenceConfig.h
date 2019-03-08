/**
 * @file PlanningModule/include/PBConfigs/PBKickSequenceConfig.h
 *
 * This file defines the struct PBKickSequenceConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "Utils/include/Behaviors/PBConfigs/PBConfig.h"

DECLARE_BEHAVIOR_CONFIG(
  PBKickSequenceConfig,
  PBConfig,
  PBKickSequenceConfigPtr,
  PBIds::kickSequence,
  9999.f,
  PBKickSequenceTypes
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  BallInterceptConfig,
  PBKickSequenceConfig,
  PBKickSequenceTypes::ballIntercept,
  BallInterceptConfigPtr
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  FindAndKickConfig,
  PBKickSequenceConfig,
  PBKickSequenceTypes::findAndKick,
  FindAndKickConfigPtr
)
