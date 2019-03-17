/**
 * @file PlanningBehaviors/TestSuite/Types/NavigationTestSuite.cpp
 *
 * This file implements the class NavigationTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/NavigationTestSuite.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "VisionModule/include/VisionRequest.h"

NavigationTestSuite::NavigationTestSuite(
  PlanningModule* planningModule,
  const boost::shared_ptr<NavigationTestSuiteConfig>& config) :
  TestSuite(planningModule, config, "NavigationTestSuite")
{
  DEFINE_FSM_STATE(NavigationTestSuite, GoToTarget, goToTarget);
  DEFINE_FSM_STATE(NavigationTestSuite, GoalChangedReplan, goalChangedReplan);
  DEFINE_FSM_STATE(NavigationTestSuite, InvalidPathReplan, invalidPathReplan);
  DEFINE_FSM(fsm, NavigationTestSuite, goToTarget);
}

bool NavigationTestSuite::initiate()
{
  try {
    LOG_INFO("NavigationTestSuite.initiate() called...");
    fsm->state = fsmStates[getBehaviorCast()->startState];
    if (!fsm->state) {
      throw
      BehaviorException(
        this,
        "Invalid start fsm state requested. See " +
        ConfigManager::getPBConfigsPath() +
        "TestSuite/LocalizationTestSuite.json.",
        true
      );
    }
    // Run Localization module
    BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::segmentation));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::field));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::robot));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::lines));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::goal));
    BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
    BaseModule::publishModuleRequest(
      boost::make_shared<InitiateLocalizer>(getBehaviorCast()->startPose)
    );
    ON_SIDE_LINE_OUT(PlanningModule) = false;
    return true;
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what())
    return false;
  }
}

void NavigationTestSuite::update()
{
  //LOG_INFO("NavigationTestSuite::update() called...")
  if (requestInProgress()) return;
  if (fsm->update())
    finish();
}

void NavigationTestSuite::finish()
{
  LOG_INFO("NavigationTestSuite.finish()")
  inBehavior = false;
}

NavigationTestSuiteConfigPtr NavigationTestSuite::getBehaviorCast()
{
  return boost::static_pointer_cast <NavigationTestSuiteConfig> (config);
}

void NavigationTestSuite::GoToTarget::onStart()
{
  bPtr->killGeneralBehavior();
  bPtr->killMotionBehavior(MOTION_1);
  auto planConfig = boost::make_shared<GoToTargetConfig>();
  planConfig->goal = bPtr->getBehaviorCast()->goalPose;
  //planConfig->reachClosest = true;
  /*planConfig->startPosture =
    boost::make_shared<InterpToPostureConfig>(
      PostureState::STAND_HANDS_BEHIND,
      1.0f);
  planConfig->endPosture =
    boost::make_shared<InterpToPostureConfig>(
      PostureState::STAND_HANDS_BEHIND,
      1.0f);*/
  bPtr->setupChildRequest(planConfig, true);
}

void NavigationTestSuite::GoToTarget::onRun()
{
  if (!bPtr->getChild()) {
    nextState = nullptr;
  } else {
    static int goalResetCount = 0;
    if (goalResetCount++ > 25) {
      auto planConfig = boost::make_shared<GoToTargetConfig>();
      planConfig->goal = bPtr->getBehaviorCast()->goalPose;
      bPtr->setupChildRequest(planConfig, true);
      goalResetCount=0;
    }
  }
}

void NavigationTestSuite::GoalChangedReplan::onStart()
{
  bPtr->killGeneralBehavior();
  bPtr->killMotionBehavior(MOTION_1);
  auto planConfig = boost::make_shared<GoToTargetConfig>();
  planConfig->goal = bPtr->getBehaviorCast()->goalPose;
  bPtr->setupChildRequest(planConfig, true);
}

void NavigationTestSuite::GoalChangedReplan::onRun()
{
  static int goalResetCount = 0;
  if (goalResetCount++ > 25) {
    bPtr->getBehaviorCast()->goalPose.x() += 0.1;
    bPtr->getBehaviorCast()->goalPose.y() += 0.1;
    auto planConfig = boost::make_shared<GoToTargetConfig>();
    planConfig->goal = bPtr->getBehaviorCast()->goalPose;
    bPtr->setupChildRequest(planConfig, true);
    goalResetCount=0;
  }
}

void NavigationTestSuite::InvalidPathReplan::onStart()
{
}

void NavigationTestSuite::InvalidPathReplan::onRun()
{
  //Do nothing
}
