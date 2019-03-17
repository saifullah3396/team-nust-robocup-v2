/**
 * @file PlanningModule/include/PBConfigs/PBRobocupConfig.h
 *
 * This file defines the struct PBRobocupConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  PBRobocupConfig,
  PBConfig,
  PBRobocupConfigPtr,
  PBIds::robocup,
  99999,
  PBRobocupTypes,
  (bool, startPoseKnown, false),
  (bool, onSideAtStart, false),
  (RobotPose2D<float>, startPose, RobotPose2D<float>(-4.25, 0.0, 0.0)),
  (MBHeadControlConfigPtr, hsConfig, HeadScanConfigPtr()),
  (MBHeadControlConfigPtr, httConfig, HeadTargetTrackConfigPtr()),
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  RobocupSetupConfig,
  PBRobocupConfig,
  PBRobocupTypes::robocupSetup,
  RobocupSetupConfigPtr
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  GoalKeeperConfig,
  PBRobocupConfig,
  PBRobocupTypes::goalKeeper,
  GoalKeeperConfigPtr
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  DefenderConfig,
  PBRobocupConfig,
  PBRobocupTypes::defender,
  DefenderConfigPtr
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  AttackerConfig,
  PBRobocupConfig,
  PBRobocupTypes::attacker,
  AttackerConfigPtr
)

DECLARE_BEHAVIOR_CONFIG_TYPE(
  PenaltiesConfig,
  PBRobocupConfig,
  PBRobocupTypes::penalties,
  PenaltiesConfigPtr
)
