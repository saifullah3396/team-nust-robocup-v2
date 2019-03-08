/**
 * @file VisionModule/VisionModule.cpp
 *
 * This file implements a class for vision planning.
 * All the functions and algorithms for image processing, object
 * detection and classification, and colorspaces will be defined
 * under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/exception/diagnostic_information.hpp> 
#include <Eigen/Dense>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/ZLIBCompression.h"
#include "VisionModule/include/CameraModule.h"
#include "VisionModule/include/FeatureExtraction/BallExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/LinesExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"
#include "LocalizationModule/include/LocalizationRequest.h"

DEFINE_INPUT_CONNECTOR(VisionModule,
  (int, visionThreadPeriod),
  (Matrix4f, upperCamInFeet),
  (Matrix4f, lowerCamInFeet),
  (unsigned, planningState),
  (RoboCupGameControlData, gameData),
  (bool, robotOnSideLine),
  (RobotPose2D<float>, robotPose2D),
)

DEFINE_OUTPUT_CONNECTOR(VisionModule,
  (BallInfo<float>, ballInfo),
  (GoalInfo<float>, goalInfo),
  (bool, landmarksFound),
  (ObsObstacles<float>, obstaclesObs),
)

#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
VisionModule::VisionModule(void* processingModule,
  const ALVideoDeviceProxyPtr& camProxy) :
  BaseModule(processingModule, toUType(TNSPLModules::vision), "VisionModule"),
  DebugBase("VisionModule", this), 
  camProxy(camProxy),
  logImages(vector<bool>(toUType(CameraId::count), false)),
  writeVideo(vector<bool>(toUType(CameraId::count), false)),
  featureExtToRun(vector<bool>(toUType(FeatureExtractionIds::count), false))
{
}
#else
VisionModule::VisionModule(void* processingModule) :
  BaseModule(processingModule, toUType(TNSPLModules::vision), "VisionModule"),
  DebugBase("VisionModule", this),
  logImages(vector<bool>(toUType(CameraId::count), false)),
  writeVideo(vector<bool>(toUType(CameraId::count), false)),
  featureExtToRun(vector<bool>(toUType(FeatureExtractionIds::count), false))
{
}
#endif

void VisionModule::setThreadPeriod()
{
  setPeriodMinMS(VISION_PERIOD_IN(VisionModule));
}

void VisionModule::initMemoryConn()
{
  inputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void VisionModule::init()
{
  LOG_INFO("Initializing vision debug base...")
  initDebugBase();
  LOG_INFO("Initializing CameraModule...")
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  cameraModule =
    boost::shared_ptr<CameraModule>(new CameraModule(this));
  #else
  cameraModule =
    boost::shared_ptr<CameraModule>(new CameraModule());
  #endif
  LOG_INFO("Initializing CameraTransform...")
  for (const auto& id : CameraId()) {
    cameraTransforms.push_back(
      boost::shared_ptr<CameraTransform>(new CameraTransform(this, id))
    );
  }
  LOG_INFO("Initializing ColorHandler...")
  colorHandler = boost::shared_ptr<ColorHandler>(new ColorHandler());
  LOG_INFO("Initializing FeatureExtraction classes...")
  setupFeatureExtraction();
  LOG_INFO("Initializing VisionModule debugging variables... See" << ConfigManager::getConfigDirPath() << "VisionConfig.ini")
  int tempDebug;
  int tempDebugImageIndex;
  int tempSendKnownLandmarks;
  int tempSendUnknownLandmarks;
  GET_CONFIG(
    "VisionConfig",
    (int, VisionModule.debug, tempDebug), 
    (int, VisionModule.debugImageIndex, tempDebugImageIndex),
    (int, VisionModule.sendKnownLandmarks, tempSendKnownLandmarks),
    (int, VisionModule.sendUnknownLandmarks, tempSendUnknownLandmarks),
  )
  SET_DVAR(int, debug, tempDebug);
  SET_DVAR(int, debugImageIndex, tempDebugImageIndex);
  LOG_INFO("Initializing VisionModule Output variables...")
  BALL_INFO_OUT(VisionModule) = BallInfo<float>();
  GOAL_INFO_OUT(VisionModule) = GoalInfo<float>();
  LANDMARKS_FOUND_OUT(VisionModule) = false;
  OBSTACLES_OBS_OUT(VisionModule) = ObsObstacles<float>();
}

void VisionModule::handleRequests()
{
  while (!inRequests.isEmpty()) {
    auto request = inRequests.queueFront();
    if (boost::static_pointer_cast <VisionRequest>(request)) {
      auto reqId = request->getId();
      if (reqId == toUType(VisionRequestIds::switchVision)) {
        auto sv = boost::static_pointer_cast <SwitchVision>(request);
        runVision = sv->state;
      } else if (reqId == toUType(VisionRequestIds::switchVideoWriter)) {
        auto svw = boost::static_pointer_cast <SwitchVideoWriter>(request);
        writeVideo[svw->camIndex] = svw->state;
      } else if (reqId == toUType(VisionRequestIds::switchFieldProjection)) {
        auto sfp = boost::static_pointer_cast <SwitchFieldProjection>(request);
        projectField = sfp->state;
      } else if (reqId == toUType(VisionRequestIds::switchLogImages)) {
        auto sli = boost::static_pointer_cast <SwitchLogImages>(request);
        logImages[sli->camIndex] = sli->state;
      } else if (reqId == toUType(VisionRequestIds::switchUseLoggedImages)) {
        auto uli = boost::static_pointer_cast <SwitchUseLoggedImages>(request);
        useLoggedImages = uli->state;
      } else if (reqId == toUType(VisionRequestIds::switchFeatureExtModule)) {
        auto uli = boost::static_pointer_cast <SwitchFeatureExtModule>(request);
        if (uli->id != FeatureExtractionIds::count) {
          featureExtToRun[toUType(uli->id)] = uli->state;
          featureExt[toUType(uli->id)]->setCurrentImage(uli->camIndex);
        } else {
          for (size_t i = 0; i < toUType(FeatureExtractionIds::count); ++i) {
            featureExtToRun[i] = uli->state;
            featureExt[i]->setCurrentImage(uli->camIndex);
          }
        }
      }
    }
    inRequests.popQueue();
  }
}

void VisionModule::mainRoutine()
{
  OBSTACLES_OBS_OUT(VisionModule).data.clear();
  // Execution of this module is decided by planning module.
  runVision = true;
  if (runVision) {
    // update cameras and get images
    cameraUpdate();
    // save video if required
    //handleVideoWriting();
    // process image for extracting new features
    if (featureExtractionUpdate())
      updateLandmarksInfo();
    if (projectField)
      setupFieldProjection();
    if (GET_DVAR(int, debug)) { // If debugging is allowed
      sendKnownLandmarksInfo();
      sendUnknownLandmarksInfo();
      sendImages();
    }
    // Update the id of newly observed obstacles
    OBSTACLES_OBS_OUT(VisionModule).resetId();
  }
}

void VisionModule::cameraUpdate()
{
  #ifdef MODULE_IS_REMOTE
  for (int i = 0; i < toUType(CameraId::count); ++i) {
    cameraModule->updateImage(i, logImages[i], useLoggedImages);
    cameraTransforms[i]->update();
  }
  #else
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    cameraModule->updateImage(i);
    // update camera transformation matrices
    cameraTransforms[i]->update();
  }
  #endif
}

void VisionModule::handleVideoWriting()
{
  /* Video recording process is too computationally expensive for the robot
   * int& writeVideo = IVAR(int, VisionModule::writeVideo);
  if (writeVideo == 0 || writeVideo == 1) {
    cameraModule->recordVideo(writeVideo);
    return;
  } else if (writeVideo == 2) {
    for (int i = 0; i < 2; ++i)
      cameraModule->recordVideo(i);
    return;
  } else if (writeVideo == -1) {
    cameraModule->stopRecording();
    return;
  }*/
}

bool VisionModule::featureExtractionUpdate()
{
  FeatureExtraction::setupImagesAndHists();
  FeatureExtraction::clearLandmarks();
  //colorHandler->update();
  /*Mat blue, yellow, green, yuv;
  cvtColor(FeatureExtraction::getBgrMat(0), yuv, COLOR_BGR2YUV);
  colorHandler->getBinary(yuv, blue, Colors::BLUE);
  colorHandler->getBinary(yuv, yellow, Colors::YELLOW);
  colorHandler->getBinary(yuv, green, Colors::GREEN);
  imshow("blue", blue);
  imshow("hellow", yellow);
  imshow("hreen", green);
  imshow("image", FeatureExtraction::getBgrMat(0));
  waitKey(1);*/
  if (colorHandler->fieldHistFormed()) {
    for (size_t i = 0; i < toUType(FeatureExtractionIds::count); ++i) {
      if (featureExtToRun[i]) {
        featureExt[i]->processImage();
      }
    }
    return true;
  }
  return false;
}

void VisionModule::updateLandmarksInfo()
{
  auto klu =
    boost::make_shared<KnownLandmarksUpdate>(
      FeatureExtraction::getKnownLandmarks());
  auto ulu =
    boost::make_shared<UnknownLandmarksUpdate>(
      FeatureExtraction::getUnknownLandmarks());
  BaseModule::publishModuleRequest(klu);
  BaseModule::publishModuleRequest(ulu);
  if (klu->landmarks.size() > 0)// || ulu->landmarks.size() > 10)
    LANDMARKS_FOUND_OUT(VisionModule) = true;
  else
    LANDMARKS_FOUND_OUT(VisionModule) = false;
}

void VisionModule::setupFieldProjection()
{
  static bool fieldPointsSaved = false;
  if (!fieldPointsSaved) {
    float x = 4.5f;
    float y = -3.f;
    for (int i = 0; i < 13; ++i) {
      fieldPoints.push_back(Point3f(x, y, 0.0));
      y += 0.5f;
    }
    x = 4.f;
    y = -3.f;
    for (int i = 0; i < 8; ++i) {
      fieldPoints.push_back(Point3f(x, y, 0.0));
      fieldPoints.push_back(Point3f(x, -y, 0.0));
      x -= 0.5f;
    }
    float xEllipse = 0;
    float ellipseRadius = 0.75;
    for (int i = 0; i < 25; ++i) {
      xEllipse += 0.75 / 25.0;
      //cout << "xEllipse: " << xEllipse << endl;
      float yEllipse =
      sqrt(ellipseRadius * ellipseRadius - xEllipse * xEllipse);
      fieldPoints.push_back(Point3f(xEllipse, yEllipse, 0.0));
      fieldPoints.push_back(Point3f(xEllipse, -yEllipse, 0.0));
    }
    fieldPointsSaved = true;
  }
  //cout << "size: " << fieldPoints.size() << endl;
  vector<Point2f> imagePs;
  cameraTransforms[0]->worldToImage(fieldPoints, imagePs);
  VisionUtils::drawPoints(imagePs, FeatureExtraction::getBgrMat(0));
  cameraTransforms[1]->worldToImage(fieldPoints, imagePs);
  VisionUtils::drawPoints(imagePs, FeatureExtraction::getBgrMat(1));
  VisionUtils::displayImage("Top", FeatureExtraction::getBgrMat(0));
  VisionUtils::displayImage("Bottom", FeatureExtraction::getBgrMat(1));
}

void VisionModule::setupFeatureExtraction()
{
  auto& ourTeam = GAME_DATA_IN(VisionModule).teams[0].teamColour;
  auto& oppTeam = GAME_DATA_IN(VisionModule).teams[1].teamColour;
  TNColors ourColor = TNColors::YELLOW, oppColor = TNColors::BLUE;
  if (ourTeam == TEAM_BLACK) ourColor = TNColors::BLACK;
  else if (ourTeam == TEAM_BLUE) ourColor = TNColors::BLUE;
  else if (ourTeam == TEAM_RED) ourColor = TNColors::RED;
  else if (ourTeam == TEAM_YELLOW) ourColor = TNColors::YELLOW;

  if (oppTeam == TEAM_BLACK) oppColor = TNColors::BLACK;
  else if (oppTeam == TEAM_BLUE) oppColor = TNColors::BLUE;
  else if (oppTeam == TEAM_RED) oppColor = TNColors::RED;
  else if (oppTeam == TEAM_YELLOW) oppColor = TNColors::YELLOW;
  auto blackJerseyExists =
    ourColor == TNColors::BLACK || oppColor == TNColors::BLACK;
  FeatureExtraction::setup(this);
  ourColor = TNColors::YELLOW;
  oppColor = TNColors::BLUE;
  FeatureExtraction::updateColorInfo(ourColor, oppColor, blackJerseyExists);

  featureExt.resize(toUType(FeatureExtractionIds::count));
  featureExt[toUType(FeatureExtractionIds::segmentation)] =
    boost::shared_ptr<RegionSegmentation> (new RegionSegmentation(this));
  featureExt[toUType(FeatureExtractionIds::field)] =
    boost::shared_ptr<FieldExtraction> (new FieldExtraction(this));
  featureExt[toUType(FeatureExtractionIds::robot)] =
    boost::shared_ptr<RobotExtraction> (new RobotExtraction(this));
  featureExt[toUType(FeatureExtractionIds::goal)] =
    boost::shared_ptr<GoalExtraction> (new GoalExtraction(this));
  featureExt[toUType(FeatureExtractionIds::ball)] =
    boost::shared_ptr<BallExtraction> (new BallExtraction(this));
  featureExt[toUType(FeatureExtractionIds::lines)] =
    boost::shared_ptr<LinesExtraction> (new LinesExtraction(this));
  try {
    boost::static_pointer_cast<RegionSegmentation>(
      featureExt[toUType(FeatureExtractionIds::segmentation)]
    )->setFieldExtraction(
        boost::static_pointer_cast<FieldExtraction>(
          featureExt[toUType(FeatureExtractionIds::field)]
        )
      );
  } catch (boost::exception &e) {
    LOG_EXCEPTION(boost::diagnostic_information(e));
  }
}

void VisionModule::sendKnownLandmarksInfo()
{
  if (GET_DVAR(int, sendKnownLandmarks)) {
    vector<KnownLandmarkPtr> kl = FeatureExtraction::getKnownLandmarks();
    //sample data
    //kl.push_back(boost::make_shared<KnownLandmark>(FL_TYPE_GOAL_POST, Point2f(4.5, -0.9)));
    //kl.push_back(boost::make_shared<KnownLandmark>(FL_TYPE_PENALTY_MARK, Point2f(3.7, 0)));
    //kl.push_back(boost::make_shared<KnownLandmark>(FL_TYPE_CIRCLE, Point2f(0.0, 0)));
    //kl.push_back(boost::make_shared<KnownLandmark>(FL_TYPE_T_CORNER, Point2f(0.0, 3.0)));
    //kl.push_back(boost::make_shared<KnownLandmark>(FL_TYPE_L_CORNER, Point2f(4.4, 3.0)));
    if (kl.size() > 0) {
      Mat_<float> klData(kl.size(), 3);
      for (size_t i = 0; i < kl.size(); ++i) {
        klData(i, 0) = kl[i]->type;
        klData(i, 1) = kl[i]->pos.x;
        klData(i, 2) = kl[i]->pos.y;
      }
      unsigned char* ptr = klData.data;
      int count = kl.size() * 12;
      // Change now to Comm message request
      CommMessage msg = CommMessage(DataUtils::bytesToHexString(ptr, count), CommMsgTypes::knownLandmarks);
      UserCommRequestPtr request = boost::make_shared<SendMsgRequest>(msg);
      BaseModule::publishModuleRequest(request);
    }
  }
}

void VisionModule::sendUnknownLandmarksInfo()
{
  if (GET_DVAR(int, sendUnknownLandmarks)) {
    vector<UnknownLandmarkPtr> ukl = FeatureExtraction::getUnknownLandmarks();
    //sample data
    //ukl.push_back(boost::make_shared<UnknownLandmark>(0, Point2f(1.0, 0)));
    //ukl.push_back(boost::make_shared<UnknownLandmark>(0, Point2f(2.0, 0)));
    //ukl.push_back(boost::make_shared<UnknownLandmark>(0, Point2f(3.0, 0)));
    //ukl.push_back(boost::make_shared<UnknownLandmark>(0, Point2f(4.0, 0)));
    if (ukl.size() > 0) {
      Mat_<float> uklData(ukl.size(), 2);
      for (size_t i = 0; i < ukl.size(); ++i) {
        uklData(i, 0) = ukl[i]->pos.x;
        uklData(i, 1) = ukl[i]->pos.y;
      }
      unsigned char* ptr = uklData.data;
      int count = ukl.size() * 8;
      // Change now to Comm message request
      CommMessage msg = CommMessage(DataUtils::bytesToHexString(ptr, count), CommMsgTypes::unknownLandmarks);
      UserCommRequestPtr request = boost::make_shared<SendMsgRequest>(msg);
      BaseModule::publishModuleRequest(request);
    }
  }
}

void VisionModule::sendImages()
{
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  if (GET_DVAR(int, debugImageIndex) == 0) {
    UserCommRequestPtr request =
      boost::make_shared<SendImageRequest>(FeatureExtraction::getBgrMat(0));
    BaseModule::publishModuleRequest(request);
  } else if (GET_DVAR(int, debugImageIndex) == 1) {
    /*UserCommRequestPtr request = boost::make_shared<SendImageRequest>(FeatureExtraction::getBgrMat(1));
    BaseModule::publishModuleRequest(request);*/
  }
#else
  if (GET_DVAR(int, debugImageIndex) == 0) {
  CommMessage imageMsg(
      VisionUtils::cvMatToString(FeatureExtraction::getBgrMat(toUType(CameraId::headTop))),
    CommMsgTypes::topImage
  );
  UserCommRequestPtr request =
    boost::make_shared<SendMsgRequest>(imageMsg);
  BaseModule::publishModuleRequest(request);
  } else if (GET_DVAR(int, debugImageIndex) == 1) {
  CommMessage imageMsg(
    VisionUtils::cvMatToString(FeatureExtraction::getBgrMat(toUType(CameraId::headBottom))),
    CommMsgTypes::bottomImage
  );
  UserCommRequestPtr request = boost::make_shared<SendMsgRequest>(imageMsg);
  BaseModule::publishModuleRequest(request);
  }
#endif
}
