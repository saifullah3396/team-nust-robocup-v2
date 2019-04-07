/**
 * @file TNRSModules/BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h
 *
 * This file defines the struct PBNavigationConfig and its childs
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

DECLARE_BEHAVIOR_CONFIG_WITH_VARS(
  PBNavigationConfig,
  PBConfig,
  PBNavigationConfigPtr,
  PBIds::navigation,
  99999,
  PBNavigationTypes,
  (RobotPose2D<float>, goal, RobotPose2D<float>(0.0, 0.0, 0.0)),
  (bool, reachClosest, true),
  (boost::shared_ptr<MBPostureConfig>, startPosture, InterpToPostureConfigPtr()),
  (boost::shared_ptr<MBPostureConfig>, endPosture, InterpToPostureConfigPtr()),
);

DECLARE_BEHAVIOR_CONFIG_TYPE(
  GoToTargetConfig,
  PBNavigationConfig,
  PBNavigationTypes::goToTarget,
  GoToTargetConfigPtr
);

