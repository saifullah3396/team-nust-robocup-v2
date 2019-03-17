/**
 * @file PlanningBehaviors/TestSuite/Types/VisionTestSuite.cpp
 *
 * This file implements the class VisionTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/VisionTestSuite.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "VisionModule/include/VisionRequest.h"

VisionTestSuite::VisionTestSuite(
  PlanningModule* planningModule,
  const boost::shared_ptr<VisionTestSuiteConfig>& config) :
  TestSuite(planningModule, config, "VisionTestSuite")
{
}

VisionTestSuiteConfigPtr VisionTestSuite::getBehaviorCast()
{
  return boost::static_pointer_cast <VisionTestSuiteConfig> (config);
}

bool VisionTestSuite::initiate()
{
  LOG_INFO("VisionTestSuite.initiate() called...");
  //! Set robot on side line as false
  ON_SIDE_LINE_OUT(PlanningModule) = false;

  //! Switch on the vision and localization modules
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));

  //! Initiate the localizer with given initial robot pose
  BaseModule::publishModuleRequest(
    boost::make_shared<InitiateLocalizer>(
      getBehaviorCast()->startPose)
  );
  return true;
}

void VisionTestSuite::update()
{
  try {
    auto& testType = getBehaviorCast()->testType;
    if (testType == "Segmentation") {
      testSegmentation();
    } else if (testType == "FieldExtraction") {
      testFieldExtraction();
    } else if (testType == "GoalExtraction") {
      testGoalExtraction();
    } else if (testType == "BallExtraction") {
      testBallExtraction();
    } else if (testType == "RobotExtraction") {
      testRobotExtraction();
    } else if (testType == "LinesExtraction") {
      testLinesExtraction();
    } else if (testType == "All") {
      testAll();
    } else {
      throw
      BehaviorException(
        this,
        "Invalid test type requested. See " +
        ConfigManager::getPBConfigsPath() +
        "TestSuite/VisionTestSuite.json.",
        true
      );
    }
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    finish();
  }
}

void VisionTestSuite::finish()
{
  LOG_INFO("VisionTestSuite.finish()")
  inBehavior = false;
}

void VisionTestSuite::testSegmentation()
{
  //! Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  /*BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::ball));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::goal));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::lines));*/
  static float wait = -5.f;
  static bool stop = false;
  if (wait > 1.f)
    DebugBase::processDebugMsg("RegionSegmentation:drawHorizontalLines:1");
  if (wait > 2.f)
    DebugBase::processDebugMsg("RegionSegmentation:drawVerticalLines:1");
  if (wait > 3.f) {
    DebugBase::processDebugMsg("RegionSegmentation:drawPoints:1");
    stop = true;
  }

  if (!stop) {
    wait += this->cycleTime;
  } else {
    if (!mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanMaxYaw = 45 * M_PI / 180;
      setupMBRequest(0, mConfig);
    }
  }
  //DebugBase::processDebugMsg("RegionSegmentation:displayInfo:1");
  DebugBase::processDebugMsg("RegionSegmentation:displayOutput:1");
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
}

void VisionTestSuite::testFieldExtraction()
{
  //! Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  /*static float wait = 0;
  static bool stop = false;
  if (wait > 1.f)
    DebugBase::processDebugMsg("RegionSegmentation:drawPoints:1");
  if (wait > 2.f)
    DebugBase::processDebugMsg("FieldExtraction:drawFiltPoints:1");
  if (wait > 3.f)
    DebugBase::processDebugMsg("FieldExtraction:drawBorder:1");
  if (wait > 4.f) {
    DebugBase::processDebugMsg("FieldExtraction:drawBorderLines:1");
    stop = true;
  }

  if (!stop) {
    wait += this->cycleTime;
  } else {
    if (!mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanMaxYaw = 75 * M_PI / 180;
      setupMBRequest(0, mConfig);
    }
  }*/
  DebugBase::processDebugMsg("RegionSegmentation:drawPoints:1");
  DebugBase::processDebugMsg("FieldExtraction:drawFiltPoints:1");
  DebugBase::processDebugMsg("FieldExtraction:drawBorder:1");
  DebugBase::processDebugMsg("FieldExtraction:drawBorderLines:1");
  //DebugBase::processDebugMsg("FieldExtraction:drawBorder:1");
  //DebugBase::processDebugMsg("FieldExtraction:drawBorderLines:1");
  //DebugBase::processDebugMsg("FieldExtraction:displayInfo:1");
  DebugBase::processDebugMsg("FieldExtraction:displayOutput:1");
}

void VisionTestSuite::testGoalExtraction()
{
  //! Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  //BaseModule::publishModuleRequest(
//    boost::make_shared<SwitchFeatureExtModule>(
//          true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::goal));
  /*static float wait = -5;
  static bool stop = false;
  if (wait > 1.f)
    DebugBase::processDebugMsg("FieldExtraction:drawBorder:1");
  if (wait > 2.f)
    DebugBase::processDebugMsg("GoalExtraction:drawScannedLines:1");
  if (wait > 3.f)
    DebugBase::processDebugMsg("GoalExtraction:drawScannedRegions:1");
  if (wait > 4.f)
    DebugBase::processDebugMsg("GoalExtraction:drawGoalBaseWindows:1");
  if (wait > 5.f) {
    DebugBase::processDebugMsg("GoalExtraction:drawGoalPostBases:1");
    stop = true;
  }

  if (!stop) {
    wait += this->cycleTime;
  } else {
    if (!mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanMaxYaw = 45 * M_PI / 180;
      setupMBRequest(0, mConfig);
    }
  }*/
  //DebugBase::processDebugMsg("GoalExtraction:drawScannedLines:1");
  //DebugBase::processDebugMsg("GoalExtraction:drawScannedRegions:1");
  //DebugBase::processDebugMsg("GoalExtraction:drawGoalBaseWindows:1");
  //DebugBase::processDebugMsg("GoalExtraction:drawGoalPostBases:1");
  //DebugBase::processDebugMsg("GoalExtraction:displayInfo:1");
  DebugBase::processDebugMsg("FieldExtraction:drawBorder:1");
  DebugBase::processDebugMsg("GoalExtraction:drawScannedLines:1");
  DebugBase::processDebugMsg("GoalExtraction:drawScannedRegions:1");
  DebugBase::processDebugMsg("GoalExtraction:drawGoalBaseWindows:1");
  DebugBase::processDebugMsg("GoalExtraction:drawGoalPostBases:1");
  DebugBase::processDebugMsg("GoalExtraction:displayOutput:1");
}

void VisionTestSuite::testBallExtraction()
{
  if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadTargetTrackConfig> ();
    mConfig->headTargetType = HeadTargetTypes::ball;
    setupMBRequest(0, mConfig);
  }

  DebugBase::processDebugMsg("BallExtraction:drawPredictionState:1");
  DebugBase::processDebugMsg("BallExtraction:drawScannedRegions:1");
  DebugBase::processDebugMsg("BallExtraction:drawBallContour:1");
  //DebugBase::processDebugMsg("BallExtraction:displayInfo:1");
  DebugBase::processDebugMsg("BallExtraction:displayOutput:1");
}

void VisionTestSuite::testRobotExtraction()
{
  static float wait = -5.f;
  static bool stop = false;
  if (wait > 1.f)
    DebugBase::processDebugMsg("FieldExtraction:drawBorder:1");
  if (wait > 2.f)
    DebugBase::processDebugMsg("RegionSegmentation:drawVerticalLines:1");
  if (wait > 3.f)
    DebugBase::processDebugMsg("RegionSegmentation:drawHorizontalLines:1");
  if (wait > 4.f)
    DebugBase::processDebugMsg("RobotExtraction:drawJerseyRegions:1");
  if (wait > 5.f) {
    DebugBase::processDebugMsg("RobotExtraction:drawRobotRegions:1");
    stop = true;
  }

  if (!stop) {
    wait += this->cycleTime;
  } else {
    if (!mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanMaxYaw = 45 * M_PI / 180;
      setupMBRequest(0, mConfig);
    }
  }
  //DebugBase::processDebugMsg("RobotExtraction:drawScannedLines:1");
  //DebugBase::processDebugMsg("RobotExtraction:drawJerseyRegions:1");
  //DebugBase::processDebugMsg("RobotExtraction:drawRobotRegions:1");
  //DebugBase::processDebugMsg("RobotExtraction:drawStrayRegions:1");
  //DebugBase::processDebugMsg("RobotExtraction:displayInfo:1");
  DebugBase::processDebugMsg("RobotExtraction:displayOutput:1");
}

void VisionTestSuite::testLinesExtraction()
{
  //! Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  //BaseModule::publishModuleRequest(
//    boost::make_shared<SwitchFeatureExtModule>(
//          true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::lines));
  /*static float wait = 0;
  static bool stop = false;
  if (wait > 1.f)
    DebugBase::processDebugMsg("FieldExtraction:drawBorderLines:1");
  if (wait > 2.f)
    DebugBase::processDebugMsg("LinesExtraction:drawScannedEdges:1");
  if (wait > 3.f)
    DebugBase::processDebugMsg("LinesExtraction:drawBorderLines:1");
  if (wait > 4.f)
    DebugBase::processDebugMsg("LinesExtraction:drawFiltWorldLines:1");
  if (wait > 5.f) {
    DebugBase::processDebugMsg("LinesExtraction:drawCorners:1");
    DebugBase::processDebugMsg("LinesExtraction:drawCircle:1");
  }
  if (wait > 6.f) {
    DebugBase::processDebugMsg("LinesExtraction:drawUnknownLandmarks:1");
    stop = true;
  }
  //DebugBase::processDebugMsg("LinesExtraction:drawScannedEdges:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawBorderLines:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawWorldLines:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawFiltWorldLines:1");

  //DebugBase::processDebugMsg("LinesExtraction:drawCorners:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawUnknownLandmarks:1");
  //DebugBase::processDebugMsg("LinesExtraction:displayInfo:1");
  if (!stop) {
    wait += this->cycleTime;
  } else {
    if (!mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanMaxYaw = 90 * M_PI / 180;
      setupMBRequest(0, mConfig);
    }
  }*/
  DebugBase::processDebugMsg("FieldExtraction:drawBorderLines:1");
  DebugBase::processDebugMsg("LinesExtraction:drawScannedEdges:1");
  DebugBase::processDebugMsg("LinesExtraction:drawBorderLines:1");
  DebugBase::processDebugMsg("LinesExtraction:drawWorldLines:1");
  DebugBase::processDebugMsg("LinesExtraction:drawFiltWorldLines:1");
  DebugBase::processDebugMsg("LinesExtraction:drawCorners:1");
  DebugBase::processDebugMsg("LinesExtraction:drawCircle:1");
  DebugBase::processDebugMsg("LinesExtraction:drawUnknownLandmarks:1");
  //DebugBase::processDebugMsg("LinesExtraction:displayInfo:1");
  DebugBase::processDebugMsg("LinesExtraction:displayOutput:1");
  //BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
}

void VisionTestSuite::testAll()
{
  //DebugBase::processDebugMsg("LinesExtraction:drawScannedEdges:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawBorderLines:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawWorldLines:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawFiltWorldLines:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawCircle:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawCorners:1");
  //DebugBase::processDebugMsg("LinesExtraction:drawUnknownLandmarks:1");
  //DebugBase::processDebugMsg("LinesExtraction:displayInfo:1");
  //DebugBase::processDebugMsg("LinesExtraction:displayOutput:1");
}
