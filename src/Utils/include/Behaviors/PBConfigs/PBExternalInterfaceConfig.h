/**
 * @file PlanningModule/include/PBConfigs/PBExternalInterfaceConfig.h
 *
 * This file defines the struct PBExternalInterfaceConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "Utils/include/Behaviors/PBConfigs/PBConfig.h"
#include "Utils/include/Behaviors/BehaviorConfigMacros.h"

DECLARE_BEHAVIOR_CONFIG(
  PBExternalInterfaceConfig,
  PBConfig,
  PBExternalInterfaceConfigPtr,
  PBIds::externalInterface,
  9999.f,
  PBExternalInterfaceTypes
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  NIHACognitionConfig,
  PBExternalInterfaceConfig,
  PBExternalInterfaceTypes::nihaCognition,
  NIHACognitionConfigPtr
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  UserRequestsHandlerConfig,
  PBExternalInterfaceConfig,
  PBExternalInterfaceTypes::userReqHandler,
  UserRequestsHandlerConfigPtr
)
