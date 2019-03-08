/**
 * @file PlanningModule/src/PlanningBehaviors/TestSuite/TestSuite.cpp
 *
 * This file implements the class TestSuite
 *
 * @author <A href="mailto:saifullah3396@rsail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/TestSuite/TestSuite.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/VisionTestSuite.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/LocalizationTestSuite.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/MotionTestSuite.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/NavigationTestSuite.h"
#include "Utils/include/Behaviors/PBConfigs/TestSuiteConfig.h"

TestSuite::TestSuite(
  PlanningModule* planningModule,
  const boost::shared_ptr<TestSuiteConfig>& config,
  const string& name) :
  PlanningBehavior(planningModule, config, name)
{
}

boost::shared_ptr<TestSuite> TestSuite::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg) 
{ 
  TestSuite* ts;
  switch (cfg->type) {
      case toUType(TestSuiteTypes::vision):
        ts =
            new VisionTestSuite(
              planningModule,
              boost::static_pointer_cast <VisionTestSuiteConfig> (cfg));
        break;
      case toUType(TestSuiteTypes::localization):
        ts = new LocalizationTestSuite(
              planningModule,
              boost::static_pointer_cast <LocalizationTestSuiteConfig> (cfg));
        break;
      case toUType(TestSuiteTypes::motion):
        ts = new MotionTestSuite(
              planningModule,
              boost::static_pointer_cast <MotionTestSuiteConfig> (cfg));
        break;
      case toUType(TestSuiteTypes::navigation):
        ts = new NavigationTestSuite(
              planningModule,
              boost::static_pointer_cast <NavigationTestSuiteConfig> (cfg));
        break;
  }
  return boost::shared_ptr<TestSuite>(ts);
}

TestSuiteConfigPtr TestSuite::getBehaviorCast()
{
  return boost::static_pointer_cast <TestSuiteConfig> (config);
}
