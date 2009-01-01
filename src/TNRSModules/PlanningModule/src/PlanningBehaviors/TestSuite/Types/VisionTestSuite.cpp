/**
 * @file PlanningBehaviors/TestSuite/Types/VisionTestSuite.cpp
 *
 * This file implements the class VisionTestSuite
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include <opencv2/highgui/highgui.hpp>
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/TestSuite/Types/VisionTestSuite.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/PBConfigs/TestSuiteConfig.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/AngleDefinitions.h"
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
  ///< Set robot on side line as false
  ON_SIDE_LINE_OUT(PlanningModule) = false;

  ///< Switch on the vision and localization modules
  BaseModule::publishModuleRequest(boost::make_shared<SwitchVision>(true));
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(true));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(false));

  ///< Initiate the localizer with given initial robot pose
  BaseModule::publishModuleRequest(
    boost::make_shared<InitiateLocalizer>(
      getBehaviorCast()->startPose)
  );

  #ifdef MODULE_IS_REMOTE
  if (getBehaviorCast()->useLoggedData) {
    auto uliRequest =
      boost::make_shared<SwitchUseLoggedImages>(true);
    BaseModule::publishModuleRequest(uliRequest);
  }
  #endif
  return true;
}

void VisionTestSuite::update()
{
  if (requestInProgress()) return;
  if (shutdownCallBack()) return;
  try {
    auto& testType = getBehaviorCast()->testType;
    if (testType == "LogImages") {
      logImages();
    } else if (testType == "LogImagesWhileMoving") {
      logImagesWhileMoving();
    } else if (testType == "Segmentation") {
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
    } else if (testType == "FieldProjection") {
      testFieldProjection();
    } else if (testType == "ColorClassification") {
      testColorClassification();
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

void VisionTestSuite::logImages()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));

  Json::Value value;
  value["VisionModule"]["debugImageIndex"] = getBehaviorCast()->loggingCam;
  DebugBase::processDebugMsg(value);
  if (getBehaviorCast()->loggingCam >= 2) {
    auto sliRequest1 =
      boost::make_shared<SwitchLogImages>(true, CameraId::headTop);
    auto sliRequest2 =
      boost::make_shared<SwitchLogImages>(true, CameraId::headBottom);
    BaseModule::publishModuleRequest(sliRequest1);
    BaseModule::publishModuleRequest(sliRequest2);
  } else {
    auto sliRequest =
      boost::make_shared<SwitchLogImages>(true, static_cast<CameraId>(getBehaviorCast()->loggingCam));
    BaseModule::publishModuleRequest(sliRequest);
  }
}

void VisionTestSuite::logImagesWhileMoving()
{
  ///< Switch on field extraction modules
  static bool loaded = false;
  static vector<RobotPose2D<float>> targets;
  static auto targetNum = 0;
  if (!loaded) {
    BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
      Json::Value value;
    value["VisionModule"]["debugImageIndex"] = getBehaviorCast()->loggingCam;
    value["VisionModule"]["logCameraTransform"] = 1;
    DebugBase::processDebugMsg(value);
    if (getBehaviorCast()->loggingCam >= 2) {
      auto sliRequest1 =
        boost::make_shared<SwitchLogImages>(true, CameraId::headTop);
      auto sliRequest2 =
        boost::make_shared<SwitchLogImages>(true, CameraId::headBottom);
      BaseModule::publishModuleRequest(sliRequest1);
      BaseModule::publishModuleRequest(sliRequest2);
    } else {
      auto sliRequest =
        boost::make_shared<SwitchLogImages>(true, static_cast<CameraId>(getBehaviorCast()->loggingCam));
      BaseModule::publishModuleRequest(sliRequest);
    }
    targets.push_back(RobotPose2D<float>(0.0, 0.0, Angle::DEG_90));
    targets.push_back(RobotPose2D<float>(2.0, 0.0, 0.0));
    targets.push_back(RobotPose2D<float>(0.0, 0.0, -Angle::DEG_90));
    targets.push_back(RobotPose2D<float>(3.5, 0.0, 0.0));
    targets.push_back(RobotPose2D<float>(0.0, 0.0, -Angle::DEG_90));
    targets.push_back(RobotPose2D<float>(3.0, 0.0, 0.0));
    targets.push_back(RobotPose2D<float>(0.0, 0.0, Angle::DEG_90));
    loaded = true;
  }

  static bool moving = false;
  static bool moveSet = false;
  static float wait = 0.f;
  cout << "wait..." << wait << endl;
  if (!moving) {
    if (!mbInProgress()) {
      auto mConfig =
        boost::make_shared <HeadScanConfig> ();
      mConfig->scanMaxYaw = 90 * M_PI / 180;
      setupMBRequest(0, mConfig);
    }
  }
  /*
    if (wait < 6.0) {
      wait += this->cycleTime;
    } else {
      wait = 0.f;
      this->killAllMotionBehaviors();
      if (targetNum > targets.size())
        finish();
      moving = true;
    }
  } else {
    if (moveSet) {
      if (!mbInProgress()) {
        moveSet = false;
        moving = false;
      }
    }

    if (!moveSet) {
      if (!mbInProgress()) {
        auto moveConfig =
          boost::make_shared<NaoqiMoveToConfig>();
        moveConfig->goal = targets[targetNum];
        setupMBRequest(0, moveConfig);
        targetNum++;
        cout << "setting move" << endl;
          moveSet = true;
      }
    }
  }*/
}

void VisionTestSuite::testSegmentation()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  Json::Value value;
  #ifdef MODULE_IS_REMOTE
  value["RegionSegmentation"]["drawHorizontalLines"] = 1;
  value["RegionSegmentation"]["drawVerticalLines"] = 1;
  value["RegionSegmentation"]["drawPoints"] = 1;
  value["RegionSegmentation"]["displayOutput"] = 1;
  #endif
  value["RegionSegmentation"]["displayInfo"] = 1;
  DebugBase::processDebugMsg(value);
  /*if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadScanConfig> ();
    mConfig->scanMaxYaw = 45 * M_PI / 180;
    setupMBRequest(0, mConfig);
  }*/
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
}

void VisionTestSuite::testFieldExtraction()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));

  /*if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadScanConfig> ();
    mConfig->scanMaxYaw = 75 * M_PI / 180;
    setupMBRequest(0, mConfig);
  }*/
  Json::Value value;
  //value["RegionSegmentation"]["drawPoints"] = 1;
  //value["FieldExtraction"]["drawFiltPoints"] = 1;
  value["FieldExtraction"]["drawBorder"] = 1;
  value["FieldExtraction"]["drawBorderLines"] = 1;
  #ifdef MODULE_IS_REMOTE
  value["FieldExtraction"]["displayOutput"] = 1;
  #endif
  value["FieldExtraction"]["displayInfo"] = 1;
  DebugBase::processDebugMsg(value);
}

void VisionTestSuite::testGoalExtraction()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::goal));
  /*if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadScanConfig> ();
    mConfig->scanMaxYaw = 45 * M_PI / 180;
    setupMBRequest(0, mConfig);
  }*/
  Json::Value value;
  value["GoalExtraction"]["drawScannedLines"] = 1;
  value["GoalExtraction"]["drawScannedRegions"] = 1;
  value["GoalExtraction"]["drawGoalBaseWindows"] = 1;
  value["GoalExtraction"]["drawShiftedBorderLines"] = 1;
  value["GoalExtraction"]["drawGoalPostBases"] = 1;
  #ifdef MODULE_IS_REMOTE
  value["GoalExtraction"]["displayOutput"] = 1;
  #endif
  value["GoalExtraction"]["displayInfo"] = 1;
  DebugBase::processDebugMsg(value);
}

void VisionTestSuite::testBallExtraction()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::ball));
  /*if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadTargetTrackConfig> ();
    mConfig->headTargetType = HeadTargetTypes::ball;
    setupMBRequest(0, mConfig);
  }*/
  Json::Value value;
  value["BallExtraction"]["drawPredictionState"] = 1;
  value["BallExtraction"]["drawPredictionState"] = 1;
  value["BallExtraction"]["drawScannedLines"] = 1;
  value["BallExtraction"]["drawUnlinkedScannedRegions"] = 1;
  value["BallExtraction"]["drawClassifiedPentagons"] = 1;
  value["BallExtraction"]["drawClassifiedTriangles"] = 1;
  value["BallExtraction"]["drawScannedRegions"] = 1;
  value["BallExtraction"]["drawBallContour"] = 1;
  value["BallExtraction"]["drawBallCircles"] = 1;
  value["BallExtraction"]["drawPredictionROI"] = 1;
  #ifdef MODULE_IS_REMOTE
  value["BallExtraction"]["displayOutput"] = 1;
  #endif
  value["BallExtraction"]["displayInfo"] = 1;
  DebugBase::processDebugMsg(value);
}

void VisionTestSuite::testRobotExtraction()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::robot));
  /*if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadScanConfig> ();
    mConfig->scanMaxYaw = 45 * M_PI / 180;
    setupMBRequest(0, mConfig);
  }*/
  Json::Value value;
  //value["RobotExtraction"]["drawScannedLines"] = 1;
  value["RobotExtraction"]["drawJerseyRegions"] = 1;
  value["RobotExtraction"]["drawRobotRegions"] = 1;
  value["RobotExtraction"]["drawStrayRegions"] = 1;
  #ifdef MODULE_IS_REMOTE
  value["RobotExtraction"]["displayOutput"] = 1;
  #endif
  value["RobotExtraction"]["displayInfo"] = 1;
  DebugBase::processDebugMsg(value);
}

void VisionTestSuite::testLinesExtraction()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::robot));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::lines));
  /*if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadScanConfig> ();
    mConfig->scanMaxYaw = 90 * M_PI / 180;
    setupMBRequest(0, mConfig);
  }*/
  Json::Value value;
  #ifdef MODULE_IS_REMOTE
  value["FieldExtraction"]["drawBorderLines"] = 1;
  value["LinesExtraction"]["drawScannedEdges"] = 1;
  value["LinesExtraction"]["drawBorderLines"] = 1;
  value["LinesExtraction"]["drawWorldLines"] = 1;
  value["LinesExtraction"]["drawFiltWorldLines"] = 1;
  value["LinesExtraction"]["drawCorners"] = 1;
  value["LinesExtraction"]["drawCircle"] = 1;
  value["LinesExtraction"]["drawUnknownLandmarks"] = 1;
  value["LinesExtraction"]["displayOutput"] = 1;
  #endif
  value["LinesExtraction"]["displayInfo"] = 1;
  DebugBase::processDebugMsg(value);
  BaseModule::publishModuleRequest(boost::make_shared<SwitchLocalization>(false));
}

void VisionTestSuite::testFieldProjection()
{
  ///< Switch on field extraction modules
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::segmentation));
  BaseModule::publishModuleRequest(
    boost::make_shared<SwitchFeatureExtModule>(
          true, FeatureExtractionIds::field));
  if (!mbInProgress()) {
    auto mConfig =
      boost::make_shared <HeadScanConfig> ();
    mConfig->scanMaxYaw = 90 * M_PI / 180;
    mConfig->totalWaitTime = 3.0;
    setupMBRequest(0, mConfig);
  }
  BaseModule::publishModuleRequest(boost::make_shared<SwitchFieldProjection>(true));
}

void VisionTestSuite::testColorClassification()
{
  if (getBehaviorCast()->colorIndex >= 0) {
    Json::Value value;
    value["VisionModule"]["sendBinaryImage"] = getBehaviorCast()->colorIndex;
    value["VisionModule"]["debugImageIndex"] = 0;
    DebugBase::processDebugMsg(value);
  } else {
    LOG_ERROR("Invalid color index: " << getBehaviorCast()->colorIndex);
  }
}

void VisionTestSuite::testAll()
{
  testSegmentation();
  testFieldExtraction();
  testGoalExtraction();
  testRobotExtraction();
  testBallExtraction();
  testLinesExtraction();
}
