/**
 * @file PlanningBehaviors/TestSuite/Types/LocalizationTestSuite.cpp
 *
 * This file implements the class LocalizationTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "BehaviorConfigs/include/PBConfigs/PBNavigationConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/LocalizationTestSuite.h"
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/AngleDefinitions.h"
#include "Utils/include/ConfigMacros.h"
#include "VisionModule/include/VisionRequest.h"

LocalizationTestSuite::LocalizationTestSuite(
  PlanningModule* planningModule,
  const boost::shared_ptr<LocalizationTestSuiteConfig>& config) :
  TestSuite(planningModule, config, "LocalizationTestSuite")
{
  DEFINE_FSM_STATE(LocalizationTestSuite, OccupancyMapTest, occupancyMapTest);
  DEFINE_FSM_STATE(LocalizationTestSuite, SideLineLocalization, sideLineLocalization);
  DEFINE_FSM_STATE(LocalizationTestSuite, LocalizationLostState, localizationLostState);
  DEFINE_FSM_STATE(LocalizationTestSuite, LocalizationPrediction, localizationPrediction);
  DEFINE_FSM_STATE(LocalizationTestSuite, LocalizationWithMovement, localizationWithMovement);
  DEFINE_FSM(fsm, LocalizationTestSuite, occupancyMapTest);
}

bool LocalizationTestSuite::initiate()
{
  try {
    LOG_INFO("LocalizationTestSuite.initiate() called...");
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
    BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
    /*Json::Value value;
    value["GoalExtraction"]["drawScannedLines"] = 1;
    value["GoalExtraction"]["drawScannedRegions"] = 1;
    value["GoalExtraction"]["drawGoalBaseWindows"] = 1;
    value["GoalExtraction"]["drawShiftedBorderLines"] = 1;
    value["GoalExtraction"]["drawGoalPostBases"] = 1;
    value["GoalExtraction"]["displayOutput"] = 1;
    DebugBase::processDebugMsg(value);
    */
    return true;
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }
}

void LocalizationTestSuite::update()
{
  if (requestInProgress()) return;
  if (shutdownCallBack()) return;
  if (fsm->update())
    finish();
}

void LocalizationTestSuite::finish()
{
  LOG_INFO("LocalizationTestSuite.finish()")
  inBehavior = false;
}


LocalizationTestSuiteConfigPtr LocalizationTestSuite::getBehaviorCast()
{
  return boost::static_pointer_cast <LocalizationTestSuiteConfig> (config);
}

void LocalizationTestSuite::OccupancyMapTest::onStart()
{
  ON_SIDE_LINE_OUT_REL(PlanningModule, bPtr) = false;
  BaseModule::publishModuleRequest(
    boost::make_shared<InitiateLocalizer>(RobotPose2D<float>(0.0, 0.0, 0.0)));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchParticleFilter>(false));
  #ifdef MODULE_IS_REMOTE
  Json::Value value;
  //value["RobotExtraction"]["displayInfo"] = 1;
  //value["GoalExtraction"]["displayInfo"] = 1;
  value["FieldMap"]["displayInfo"] = 1;
  value["FieldMap"]["displayOutput"] = 1;
  DebugBase::processDebugMsg(value);
  #endif

}

void LocalizationTestSuite::OccupancyMapTest::onRun()
{
  //! Do nothing
}

void LocalizationTestSuite::SideLineLocalization::onStart()
{
  ON_SIDE_LINE_OUT_REL(PlanningModule, bPtr) = true;
}

void LocalizationTestSuite::SideLineLocalization::onRun()
{
  auto localized = ROBOT_LOCALIZED_IN_REL(PlanningModule, bPtr);
  if (!localized) {
    if (!bPtr->mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadTargetTrackConfig> ();
      mConfig->headTargetType = HeadTargetTypes::goal;
      // Robot is on sidelines with other robots so keep scan range minimum.
      mConfig->scanConfig = boost::make_shared<HeadScanConfig>();
      mConfig->scanConfig->scanMaxYaw = Angle::DEG_45;
      bPtr->setupMBRequest(MOTION_1, mConfig);
    }
  } else {
    bPtr->killAllMotionBehaviors();
    cout << "Robot pose: " << ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr).get().transpose() << endl;
    //LOCALIZE_LAST_KNOWN_OUT_REL(PlanningModule, bPtr) = true;
    //nextState = nullptr;
  }
}

void LocalizationTestSuite::LocalizationLostState::onStart()
{
  ON_SIDE_LINE_OUT_REL(PlanningModule, bPtr) = false;
  LOCALIZE_LAST_KNOWN_OUT_REL(PlanningModule, bPtr) = false;
}

void LocalizationTestSuite::LocalizationLostState::onRun()
{
  auto localized = ROBOT_LOCALIZED_IN_REL(PlanningModule, bPtr);
  if (!localized) {
    if (!bPtr->mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadTargetTrackConfig> ();
      mConfig->headTargetType = HeadTargetTypes::goal;
      // Robot is on sidelines with other robots so keep scan range minimum.
      mConfig->scanConfig = boost::make_shared<HeadScanConfig>();
      mConfig->scanConfig->scanMaxYaw = Angle::DEG_90;
      bPtr->setupMBRequest(MOTION_1, mConfig);
    }
  } else {
    bPtr->killAllMotionBehaviors();
    //cout << "Localized..." << endl;
    cout << "Robot pose: " << ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr).get().transpose() << endl;
    nextState = nullptr;
  }
}

void LocalizationTestSuite::LocalizationPrediction::onStart()
{
  BaseModule::publishModuleRequest(
    boost::make_shared<InitiateLocalizer>(RobotPose2D<float>(0.0, 0.0, 0.0)));
}

void LocalizationTestSuite::LocalizationPrediction::onRun()
{
  if (bPtr->getChild())
    boost::static_pointer_cast<PlanningBehavior>(bPtr->getChild())->setMBIdOffset(MOTION_1 + 1);

  auto localized = ROBOT_LOCALIZED_IN_REL(PlanningModule, bPtr);
  if (localized) {
    static bool setupNav = false;
    if (!setupNav) {
      auto pose = ROBOT_POSE_2D_IN_REL(PlanningModule, bPtr);
      RobotPose2D<float> goalPose;
      RobotPose2D<float> relPose = RobotPose2D<float>(0.0, 1.0, 0.0);
      goalPose = pose.transform(relPose);
      bPtr->killGeneralBehavior();
      bPtr->killMotionBehavior(MOTION_1);
      //auto hcConfig =
      //  boost::make_shared <HeadTargetTrackConfig> (HeadTargetTypes::GOAL);
      //setupMBRequest(MOTION_1, hcConfig);
      auto planConfig =
        boost::make_shared<PlanTowardsConfig>();
      planConfig->goal = goalPose;
      planConfig->startPosture =
        boost::make_shared<InterpToPostureConfig>();
      planConfig->startPosture->targetPosture = PostureState::standHandsBehind;
      planConfig->startPosture->timeToReachP = 1.0;
      planConfig->endPosture =
        boost::make_shared<InterpToPostureConfig>();
      planConfig->endPosture->targetPosture = PostureState::standHandsBehind;
      planConfig->endPosture->timeToReachP = 1.0;
      bPtr->setupChildRequest(planConfig, true);
      setupNav = true;
      LOCALIZE_LAST_KNOWN_OUT_REL(PlanningModule, bPtr) = true;
    }
  }
}

void LocalizationTestSuite::LocalizationWithMovement::onStart()
{
  ON_SIDE_LINE_OUT_REL(PlanningModule, bPtr) = true;
}

void LocalizationTestSuite::LocalizationWithMovement::onRun()
{
  if (bPtr->getChild())
    boost::static_pointer_cast<PlanningBehavior>(bPtr->getChild())->setMBIdOffset(MOTION_1 + 1);

  auto localized = ROBOT_LOCALIZED_IN_REL(PlanningModule, bPtr);
  if (!localized) {
    if (!bPtr->mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadTargetTrackConfig> ();
      mConfig->headTargetType = HeadTargetTypes::goal;
      mConfig->scanConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanConfig->scanMaxYaw = Angle::DEG_90;
      bPtr->setupMBRequest(MOTION_1, mConfig);
    }
  } else {
    if (!bPtr->mbInProgress()) {
      auto hcConfig =
        boost::make_shared <HeadScanConfig> ();
      bPtr->setupMBRequest(MOTION_1, hcConfig);
    }
    RobotPose2D<float> goalPose = RobotPose2D<float>(-1.0, 0.0, 0.0);
    auto planConfig = boost::make_shared<PlanTowardsConfig>();
    planConfig->goal = goalPose;
    planConfig->tolerance = RobotPose2D<float>(0.05, 0.05, Angle::DEG_30);
    /*auto planConfig =
      boost::make_shared<GoToTargetConfig>();
    planConfig->goal = goalPose;
    planConfig->reachClosest = true;
    planConfig->startPosture =
      boost::make_shared<InterpToPostureConfig>();
    planConfig->startPosture->targetPosture = PostureState::standHandsBehind;
    planConfig->startPosture->timeToReachP = 1.0;
    planConfig->endPosture =
      boost::make_shared<InterpToPostureConfig>();
    planConfig->endPosture->targetPosture = PostureState::standHandsBehind;
    planConfig->endPosture->timeToReachP = 1.0;*/
    bPtr->setupChildRequest(planConfig, true);
    ON_SIDE_LINE_OUT_REL(PlanningModule, bPtr) = false;
    LOCALIZE_LAST_KNOWN_OUT_REL(PlanningModule, bPtr) = false;
  }
}
