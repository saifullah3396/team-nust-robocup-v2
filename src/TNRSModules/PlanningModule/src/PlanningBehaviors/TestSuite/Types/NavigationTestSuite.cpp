/**
 * @file PlanningBehaviors/TestSuite/Types/NavigationTestSuite.cpp
 *
 * This file implements the class NavigationTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "TNRSBase/include/DebugBase.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/NavigationTestSuite.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"

NavigationTestSuite::NavigationTestSuite(
  PlanningModule* planningModule,
  const boost::shared_ptr<NavigationTestSuiteConfig>& config) :
  TestSuite(planningModule, config, "NavigationTestSuite")
{
  DEFINE_FSM_STATE(NavigationTestSuite, GoToTarget, goToTarget);
  DEFINE_FSM_STATE(NavigationTestSuite, GoalChangedReplan, goalChangedReplan);
  DEFINE_FSM_STATE(NavigationTestSuite, InvalidPathReplan, invalidPathReplan);
  DEFINE_FSM_STATE(NavigationTestSuite, PlanTowards, planTowards);
  DEFINE_FSM_STATE(NavigationTestSuite, FollowBall, followBall);
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
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(true, FeatureExtractionIds::ball));
    BaseModule::publishModuleRequest(
      boost::make_shared<InitiateLocalizer>(getBehaviorCast()->startPose)
    );
    BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
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
  if (shutdownCallBack()) return;
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

void NavigationTestSuite::FollowBall::onStart()
{
  LOG_INFO("FollowBall.onStart()");
  //Json::Value value;
  #ifdef MODULE_IS_REMOTE
  //value["BallExtraction"]["drawPredictionState"] = 1;
  //value["BallExtraction"]["drawBallContour"] = 1;
  //value["BallExtraction"]["drawPredictionROI"] = 1;
  //value["BallExtraction"]["displayOutput"] = 1;
  #endif
 // DebugBase::processDebugMsg(value);
  bPtr->killGeneralBehavior();
  bPtr->killMotionBehavior(MOTION_1);
}

void NavigationTestSuite::FollowBall::onRun()
{
  if (bPtr->getChild())
    boost::static_pointer_cast<PlanningBehavior>(bPtr->getChild())->setMBIdOffset(MOTION_1+1);
  if (!bPtr->mbInProgress()) {
    auto httConfig =
      boost::make_shared<HeadTargetTrackConfig>();
    httConfig->headTargetType = HeadTargetTypes::ball;
    bPtr->setupMBRequest(MOTION_1, httConfig);
  }
  const auto& robotPose2D = ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr);
  const auto& bInfo = BALL_INFO_IN_REL(PlanningModule, bPtr);
  if (bInfo.found) {
    auto rToBallAngle = atan2(bInfo.posRel.y, bInfo.posRel.x);
    auto targetWorld =
      RobotPose2D<float>(bInfo.posRel.x - 0.15 * cos(rToBallAngle), bInfo.posRel.y - 0.15 * sin(rToBallAngle), rToBallAngle);
    targetWorld = robotPose2D.transform(targetWorld);
    auto planConfig = boost::make_shared<PlanTowardsConfig>();
    planConfig->goal = targetWorld;
    planConfig->tolerance = RobotPose2D<float>(0.05, 0.05, Angle::DEG_30);
    bPtr->setupChildRequest(planConfig, true);
  } else {
    auto planConfig = boost::make_shared<PlanTowardsConfig>();
    planConfig->goal = robotPose2D;
    bPtr->setupChildRequest(planConfig, true);
  }
}


void NavigationTestSuite::PlanTowards::onStart()
{
  LOG_INFO("PlanTowards.onStart()");
  bPtr->killGeneralBehavior();
  bPtr->killMotionBehavior(MOTION_1);
  auto planConfig = boost::make_shared<PlanTowardsConfig>();
  planConfig->goal = bPtr->getBehaviorCast()->goalPose;
  bPtr->setupChildRequest(planConfig, true);
}

void NavigationTestSuite::PlanTowards::onRun()
{
  if (!bPtr->getChild()) {
    nextState = nullptr;
    LOG_INFO("Plan towards finished.");
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
