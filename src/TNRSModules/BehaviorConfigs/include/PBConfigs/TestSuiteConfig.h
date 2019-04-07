/**
 * @file TNRSModules/BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h
 *
 * This file defines the struct TestSuiteConfig and its childs
 *
 * * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "PlanningModule/include/PlanningBehaviorIds.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"

DECLARE_BEHAVIOR_CONFIG(
  TestSuiteConfig,
  PBConfig,
  TestSuiteConfigPtr,
  PBIds::testSuite,
  9999.f,
  TestSuiteTypes
);

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  VisionTestSuiteConfig,
  TestSuiteConfig,
  TestSuiteTypes::vision,
  VisionTestSuiteConfigPtr,
  (string, testType, ""),
  (RobotPose2D<float>, startPose, RobotPose2D<float>(0.0, 0.0, 0.0)),
);

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  LocalizationTestSuiteConfig,
  TestSuiteConfig,
  TestSuiteTypes::localization,
  LocalizationTestSuiteConfigPtr,
  (string, startState, string("")),
);

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  MotionTestSuiteConfig,
  TestSuiteConfig,
  TestSuiteTypes::motion,
  MotionTestSuiteConfigPtr,
  (string, requestedBehavior, string("")),
);

DECLARE_BEHAVIOR_CONFIG_TYPE_WITH_VARS(
  NavigationTestSuiteConfig,
  TestSuiteConfig,
  TestSuiteTypes::navigation,
  NavigationTestSuiteConfigPtr,
  (string, startState, string("")),
  (RobotPose2D<float>, startPose, RobotPose2D<float>(0.0, 0.0, 0.0)),
  (RobotPose2D<float>, goalPose, RobotPose2D<float>(1.0, 0.0, 0.0)),
);
