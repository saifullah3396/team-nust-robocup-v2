/**
 * @file Utils/include/Behaviors/PBConfigs/PBNavigationConfig.h
 *
 * This file defines the struct PBNavigationConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/PBConfigs/PBNavigationConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBNavigationConfig, PBConfig, PBNavigationConfigPtr,
  (PBNavigationTypes, goToTarget, GoToTargetConfig),
)

void GoToTargetConfig::validate() {}
