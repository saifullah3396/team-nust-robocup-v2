/**
 * @file Utils/include/Behaviors/PBConfigs/PBStartupConfig.h
 *
 * This file defines the struct PBStartupConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/PBConfigs/PBStartupConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBStartupConfig, PBConfig, PBStartupConfigPtr,
  (PBStartupTypes, requestBehavior, RequestBehaviorConfig),
)

void RequestBehaviorConfig::validate() {}
