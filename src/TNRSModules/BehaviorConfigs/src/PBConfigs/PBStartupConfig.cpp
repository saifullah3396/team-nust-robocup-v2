/**
 * @file TNRSModules/BehaviorConfigs/src/PBConfigs/PBStartupConfig.cpp
 *
 * This file defines the struct PBStartupConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/PBConfigs/PBStartupConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBStartupConfig, PBConfig, PBStartupConfigPtr,
  (PBStartupTypes, requestBehavior, RequestBehaviorConfig),
);

void RequestBehaviorConfig::init() {}
void RequestBehaviorConfig::validate() {}
