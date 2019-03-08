/**
 * @file Utils/include/Behaviors/PBConfigs/PBRobocupConfig.h
 *
 * This file defines the struct PBRobocupConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/PBConfigs/PBRobocupConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  PBRobocupConfig, PBConfig, PBRobocupConfigPtr,
  (PBRobocupTypes, robocupSetup, RobocupSetupConfig),
  (PBRobocupTypes, goalKeeper, GoalKeeperConfig),
  (PBRobocupTypes, attacker, AttackerConfig),
  (PBRobocupTypes, defender, DefenderConfig),
  (PBRobocupTypes, penalties, PenaltiesConfig),
)

void RobocupSetupConfig::validate() {}
void GoalKeeperConfig::validate() {}
void AttackerConfig::validate() {}
void DefenderConfig::validate() {}
void PenaltiesConfig::validate() {}
