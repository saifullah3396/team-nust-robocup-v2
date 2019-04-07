/**
 * @file TNRSModules/BehaviorConfigs/src/PBConfigs/PBNavigationConfig.cpp
 *
 * This file defines the struct PBNavigationConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBNavigationConfig, PBConfig, PBNavigationConfigPtr,
  (PBNavigationTypes, goToTarget, GoToTargetConfig),
);

void GoToTargetConfig::init() {}
void GoToTargetConfig::validate() {}
