/**
 * @file PlanningModule/include/PBConfigs/PBStartupConfig.h
 *
 * This file defines the struct PBStartupConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"

DECLARE_BEHAVIOR_CONFIG(
  PBStartupConfig,
  PBConfig,
  PBStartupConfigPtr,
  PBIds::startup,
  10.0,
  PBStartupTypes
)

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  RequestBehaviorConfig,
  PBStartupConfig,
  PBStartupTypes::requestBehavior,
  RequestBehaviorConfigPtr,
  (string, requestedPosture, string("")),
  (string, requestedBehavior, string("")),
  (float, startWaitTime, 0.5f),
)
