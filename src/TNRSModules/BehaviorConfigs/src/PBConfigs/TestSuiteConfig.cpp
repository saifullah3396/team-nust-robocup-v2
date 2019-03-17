/**
 * @file BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h
 *
 * This file defines the struct TestSuiteConfig and its childs
 * 
 * * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"

DEFINE_BEHAVIOR_CONFIG(
  TestSuiteConfig, PBConfig, TestSuiteConfigPtr,
  (TestSuiteTypes, vision, VisionTestSuiteConfig),
  (TestSuiteTypes, motion, MotionTestSuiteConfig),
  (TestSuiteTypes, localization, LocalizationTestSuiteConfig),
  (TestSuiteTypes, navigation, NavigationTestSuiteConfig),
)

void VisionTestSuiteConfig::init() {}
void VisionTestSuiteConfig::validate() {}
void MotionTestSuiteConfig::init() {}
void MotionTestSuiteConfig::validate() {}
void LocalizationTestSuiteConfig::init() {}
void LocalizationTestSuiteConfig::validate() {}
void NavigationTestSuiteConfig::init() {}
void NavigationTestSuiteConfig::validate() {}
