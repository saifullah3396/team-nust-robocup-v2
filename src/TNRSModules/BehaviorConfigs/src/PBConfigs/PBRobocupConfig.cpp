/**
 * @file TNRSModules/BehaviorConfigs/src/PBConfigs/PBRobocupConfig.cpp
 *
 * This file defines the struct PBRobocupConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/PBConfigs/PBRobocupConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBRobocupConfig, PBConfig, PBRobocupConfigPtr,
  (PBRobocupTypes, robocupSetup, RobocupSetupConfig),
  (PBRobocupTypes, goalKeeper, GoalKeeperConfig),
  (PBRobocupTypes, attacker, AttackerConfig),
  (PBRobocupTypes, defender, DefenderConfig),
  (PBRobocupTypes, penalties, PenaltiesConfig),
);

void RobocupSetupConfig::init() {}
void RobocupSetupConfig::validate() {}
void GoalKeeperConfig::init() {}
void GoalKeeperConfig::validate() {}
void AttackerConfig::init() {}
void AttackerConfig::validate() {}
void DefenderConfig::init() {}
void DefenderConfig::validate() {}
void PenaltiesConfig::init() {}
void PenaltiesConfig::validate() {}
