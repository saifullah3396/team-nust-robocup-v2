/**
 * @file VisionModule/src/VisionModule.cpp
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
#include "UserCommModule/include/UserCommRequest.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/DataHolders/CommMsgTypes.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/VisionUtils.h"
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
);

DEFINE_OUTPUT_CONNECTOR(VisionModule,
  (int, visionThreadTimeTaken),
  (BallInfo<float>, ballInfo),
  (GoalInfo<float>, goalInfo),
  (bool, landmarksFound),
  (ObsObstacles<float>, obstaclesObs),
);

#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  #ifndef V6_CROSS_BUILD
    VisionModule::VisionModule(void* teamNUSTSPL,
      const ALVideoDeviceProxyPtr& camProxy) :
      BaseModule(teamNUSTSPL, TNSPLModules::vision, "VisionModule"),
      DebugBase("VisionModule", this),
      camProxy(camProxy),
      logImages(vector<bool>(toUType(CameraId::count), false)),
      writeVideo(vector<bool>(toUType(CameraId::count), false))
    {
    }
  #else
    VisionModule::VisionModule(void* teamNUSTSPL,
      const qi::AnyObject& camProxy) :
      BaseModule(teamNUSTSPL, TNSPLModules::vision, "VisionModule"),
      DebugBase("VisionModule", this),
      camProxy(camProxy),
      logImages(vector<bool>(toUType(CameraId::count), false)),
      writeVideo(vector<bool>(toUType(CameraId::count), false))
    {
    }
  #endif
#else
  VisionModule::VisionModule(void* teamNUSTSPL) :
    BaseModule(teamNUSTSPL, TNSPLModules::vision, "VisionModule"),
    DebugBase("VisionModule", this),
    logImages(vector<bool>(toUType(CameraId::count), false)),
    writeVideo(vector<bool>(toUType(CameraId::count), false))
  {
  }
#endif

void VisionModule::setThreadPeriod()
{
  setPeriodMinMS(VISION_PERIOD_IN(VisionModule));
}

void VisionModule::setThreadTimeTaken()
{
  VISION_TIME_TAKEN_OUT(VisionModule) = lastIterationTimeMS;
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
  GET_DEBUG_CONFIG(
    VisionConfig,
    VisionModule,
    (int, debug),
    (int, debugImageIndex),
    (int, logCameraTransform),
    (int, sendBinaryImage),
    (int, sendKnownLandmarks),
    (int, sendUnknownLandmarks),
    (int, displayInfo),
  );
  LOG_INFO("Initializing CameraModule...")
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  cameraModule =
    boost::shared_ptr<CameraModule>(new CameraModule(this));
  #else
  activeCamera = CameraId::headTop;
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
      auto reqId = request->getRequestId();
      if (reqId == toUType(VisionRequestIds::switchVision)) {
        auto sv = boost::static_pointer_cast <SwitchVision>(request);
        runVision = sv->state;
      } else if (reqId == toUType(VisionRequestIds::switchVideoWriter)) {
        auto svw = boost::static_pointer_cast <SwitchVideoWriter>(request);
        writeVideo[toUType(svw->camIndex)] = svw->state;
      } else if (reqId == toUType(VisionRequestIds::switchFieldProjection)) {
        auto sfp = boost::static_pointer_cast <SwitchFieldProjection>(request);
        projectField = sfp->state;
      } else if (reqId == toUType(VisionRequestIds::switchLogImages)) {
        auto sli = boost::static_pointer_cast <SwitchLogImages>(request);
        logImages[toUType(sli->camIndex)] = sli->state;
      } else if (reqId == toUType(VisionRequestIds::switchUseLoggedImages)) {
        auto uli = boost::static_pointer_cast <SwitchUseLoggedImages>(request);
        useLoggedImages = uli->state;
      } else if (reqId == toUType(VisionRequestIds::switchFeatureExtModule)) {
        auto uli = boost::static_pointer_cast <SwitchFeatureExtModule>(request);
        if (uli->id != FeatureExtractionIds::count) {
          featureExt[toUType(uli->id)]->setEnabled(uli->state);
        } else {
          for (const auto& fe : featureExt) fe->setEnabled(uli->state);
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
  if (runVision) {
    // update cameras and get images
    cameraUpdate();
    // save video if required
    //handleVideoWriting();
    // process image for extracting new features
    /*Mat image = FeatureExtraction::makeYuvMat(static_cast<CameraId>(0));
    for (int i = 0; i < toUType(TNColors::count); ++i) {
      Mat out;
      colorHandler->getBinary(image, out, static_cast<TNColors>(i));
      VisionUtils::displayImage(colorNames[i], out);
    }
    waitKey(0);*/
    FeatureExtraction::updateImageMatrices(activeCamera, bool(GET_DVAR(int, debug)));
    //Not using histograms
    //if (activeCamera == CameraId::headBottom)
    //  FeatureExtraction::createYuvHist(activeCamera, false);
    FeatureExtraction::clearLandmarks();
    if (GET_DVAR(int, debug))
      colorHandler->update();
    if (featureExtractionUpdate())
      updateLandmarksInfo();
    if (projectField)
      setupFieldProjection();
    if (GET_DVAR(int, debug)) { // If debugging is allowed
      sendKnownLandmarksInfo();
      sendUnknownLandmarksInfo();
      sendImages();
      #ifndef MODULE_IS_REMOTE
      saveImages();
      #endif
    }
    if (GET_DVAR(int, displayInfo)) {
      LOG_INFO("Active camerae:" << toUType(activeCamera));
      LOG_INFO("RegionSegmentation Time: " << GET_FEATURE_EXT_CLASS_(RegionSegmentation, FeatureExtractionIds::segmentation)->getProcessTime());
      LOG_INFO("Field border found: " << GET_FEATURE_EXT_CLASS_(FieldExtraction, FeatureExtractionIds::field)->isFound());
      LOG_INFO("FieldExtraction Time: " << GET_FEATURE_EXT_CLASS_(FieldExtraction, FeatureExtractionIds::field)->getProcessTime());
      LOG_INFO("Robots reagions: " << GET_FEATURE_EXT_CLASS_(RobotExtraction, FeatureExtractionIds::robot)->getRobotRegions().size());
      LOG_INFO("RobotExtraction Time: " << GET_FEATURE_EXT_CLASS_(RobotExtraction, FeatureExtractionIds::robot)->getProcessTime());
      LOG_INFO("Ball found: " << BALL_INFO_OUT(VisionModule).found);
      LOG_INFO("Ball position: " << BALL_INFO_OUT(VisionModule).posRel);
      LOG_INFO("BallExtraction Time: " << GET_FEATURE_EXT_CLASS_(BallExtraction, FeatureExtractionIds::ball)->getProcessTime());
      LOG_INFO("Goal posts found: " << GET_FEATURE_EXT_CLASS_(GoalExtraction, FeatureExtractionIds::goal)->getGoalPosts().size());
      LOG_INFO("Goal found: " << GOAL_INFO_OUT(VisionModule).found);
      LOG_INFO("GoalExtraction Time: " << GET_FEATURE_EXT_CLASS_(GoalExtraction, FeatureExtractionIds::goal)->getProcessTime());
      LOG_INFO("Unknown landmarks found: " << FeatureExtraction::getUnknownLandmarks().size());
      LOG_INFO("Known landmarks found: " << FeatureExtraction::getKnownLandmarks().size());
      for (const auto& l : FeatureExtraction::getKnownLandmarks()) {
          LOG_INFO("Type:" << l->type);
      }
      LOG_INFO("LinesExtraction Time: " << GET_FEATURE_EXT_CLASS_(LinesExtraction, FeatureExtractionIds::lines)->getProcessTime());
      LOG_INFO("Last iteration time in MS: " << this->lastIterationTimeMS);
    }
    //VisionUtils::displayImage("Top", FeatureExtraction::getBgrMat(0));
    //VisionUtils::displayImage("Bottom", FeatureExtraction::getBgrMat(1));
    //waitKey(0);

    // Update the id of newly observed obstacles
    OBSTACLES_OBS_OUT(VisionModule).resetId();
  }
}

void VisionModule::cameraUpdate()
{
  activeCamera = activeCamera == CameraId::headTop ? CameraId::headBottom : CameraId::headTop;
  //cout << "logImages[toUType(activeCamera)]:" <<logImages[toUType(activeCamera)] << endl;
  #ifdef MODULE_IS_REMOTE
  cameraModule->updateImage(activeCamera, logImages[toUType(activeCamera)], useLoggedImages);
  if (useLoggedImages) {
    Json::Value root;
    string basePath = ConfigManager::getLogsDirPath() + "Transformations/";
    if (activeCamera == CameraId::headTop) {
      root = JsonUtils::readJson(basePath + "Top/Transformation-" + CameraModule::getCurrentImagePath().path().stem().string() + ".json");
      auto upperCamTrans = IVAR_PTR(Matrix4f, VisionModule::Input::upperCamInFeet);
      JsonUtils::jsonToType(*upperCamTrans, root, Matrix4f::Identity());
    } else {
      root = JsonUtils::readJson(basePath + "Bottom/Transformation-" + CameraModule::getCurrentImagePath().path().stem().string() + ".json");
      auto lowerCamTrans = IVAR_PTR(Matrix4f, VisionModule::Input::lowerCamInFeet);
      JsonUtils::jsonToType(*lowerCamTrans, root, Matrix4f::Identity());
    }
  }
  cameraTransforms[toUType(activeCamera)]->update();
  #else
  cameraModule->updateImage(activeCamera);
  cameraTransforms[toUType(activeCamera)]->update();
  #endif
  if (GET_DVAR(int, logCameraTransform)) {
    Json::Value root;
    string filePath;
    if (activeCamera == CameraId::headTop) {
      root = JsonUtils::getJson(UPPER_CAM_TRANS_IN(VisionModule));
      filePath = ConfigManager::getLogsDirPath() +
            "Images/Top/Transformation-" + CameraModule::getLogDatTime() + ".json";
    } else {
      root = JsonUtils::getJson(LOWER_CAM_TRANS_IN(VisionModule));
      filePath = ConfigManager::getLogsDirPath() +
            "Images/Bottom/Transformation-" + CameraModule::getLogDatTime() + ".json";
    }
    std::ofstream file;
    file.open(filePath);
    Json::FastWriter fastWriter;
    file << fastWriter.write(root);
    file.close();
  }
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
  //if (colorHandler->fieldHistFormed()) {
  for (const auto& fe : featureExt) {
    if (fe->isEnabled()) {
      fe->setActiveCamera(activeCamera);
      try {
        fe->processImage();
      } catch (exception& e) {
        LOG_EXCEPTION("Exception raised in " << fe->getName() << "::processImage():\n\t" << e.what());
      }
    }
  }
  return true;
  //}
  //return false;
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
  //LOG_INFO("N landmarks seen1: " << ulu->landmarks.size());
  //LOG_INFO("N landmarks seen2: " << klu->landmarks.size());
  //LOG_INFO("LastIterationTimeMS: " << this->lastIterationTimeMS);
  /*for (auto l : klu->landmarks) {
    if (l->type == 0) {
      cout << "l->type:" << l->type << endl;
      cout << "l->pos: "<< l->pos << endl;
      cout << "l->pose:" << l->poseFromLandmark.get().transpose() << endl;
    }
  }*/
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
    float ellipseRadius = 0.75;
    for (int i = 0; i < 360; ++i) {
      fieldPoints.push_back(Point3f(ellipseRadius * cos(i*MathsUtils::DEG_TO_RAD), ellipseRadius * sin(i*MathsUtils::DEG_TO_RAD), 0.0));
    }
    fieldPointsSaved = true;
  }
  //cout << "size: " << fieldPoints.size() << endl;
  vector<Point2f> imagePs;
  cameraTransforms[0]->worldToImage(fieldPoints, imagePs);
  VisionUtils::drawPoints(imagePs, FeatureExtraction::getBgrMat(0));
  cameraTransforms[1]->worldToImage(fieldPoints, imagePs);
  VisionUtils::drawPoints(imagePs, FeatureExtraction::getBgrMat(1));
  #ifdef MODULE_IS_REMOTE
  VisionUtils::displayImage("Top", FeatureExtraction::getBgrMat(0));
  VisionUtils::displayImage("Bottom", FeatureExtraction::getBgrMat(1));
  #endif
}

void VisionModule::setupFeatureExtraction()
{
  auto& ourTeam = GAME_DATA_IN(VisionModule).teams[0].teamColour;
  auto& oppTeam = GAME_DATA_IN(VisionModule).teams[1].teamColour;
  TNColors ourColor = TNColors::yellow, oppColor = TNColors::red;
  if (ourTeam == TEAM_BLACK) ourColor = TNColors::black;
  else if (ourTeam == TEAM_BLUE) ourColor = TNColors::blue;
  else if (ourTeam == TEAM_RED) ourColor = TNColors::red;
  else if (ourTeam == TEAM_YELLOW) ourColor = TNColors::yellow;

  if (oppTeam == TEAM_BLACK) oppColor = TNColors::black;
  else if (oppTeam == TEAM_BLUE) oppColor = TNColors::blue;
  else if (oppTeam == TEAM_RED) oppColor = TNColors::red;
  else if (oppTeam == TEAM_YELLOW) oppColor = TNColors::yellow;
  auto blackJerseyExists =
    ourColor == TNColors::black || oppColor == TNColors::black;
  FeatureExtraction::setup(this);
  ourColor = TNColors::yellow;
  oppColor = TNColors::red;
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
    vector<boost::shared_ptr<KnownLandmark<float>>> kl = FeatureExtraction::getKnownLandmarks();
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
    vector<boost::shared_ptr<UnknownLandmark<float>>> ukl = FeatureExtraction::getUnknownLandmarks();
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
  auto cameraIndex = GET_DVAR(int, debugImageIndex);
  if (cameraIndex < toUType(CameraId::count)) {
    if (GET_DVAR(int, sendBinaryImage) >= 0) {
      ASSERT(GET_DVAR(int, sendBinaryImage) < toUType(TNColors::count));
      Mat image = FeatureExtraction::makeYuvMat(static_cast<CameraId>(GET_DVAR(int, debugImageIndex)));
      Mat out;
      colorHandler->getBinary(image, out, static_cast<TNColors>(GET_DVAR(int, sendBinaryImage)));
      #ifndef MODULE_IS_REMOTE
      UserCommRequestPtr request =
        boost::make_shared<SendImageRequest>(out);
      BaseModule::publishModuleRequest(request);
      #else
      VisionUtils::displayImage("binary", out);
      waitKey(0);
      #endif
    } else {
      Mat image = FeatureExtraction::getBgrMat(cameraIndex);
      ///cv::resize(image, image, Size(160, 120));
      UserCommRequestPtr request =
        boost::make_shared<SendImageRequest>(image);
      BaseModule::publishModuleRequest(request);
    }
  }
}

#ifndef MODULE_IS_REMOTE
void VisionModule::saveImages()
{
  auto cameraIndex = GET_DVAR(int, debugImageIndex);
  if (logImages[cameraIndex] &&
      toUType(activeCamera) == cameraIndex)
  {
    static string imagesDir = ConfigManager::getLogsDirPath() + "Images";
    static string topDir = ConfigManager::getLogsDirPath() + "Images/Top";
    static string bottomDir = ConfigManager::getLogsDirPath() + "Images/Bottom";
    if (!boost::filesystem::exists(imagesDir)) {
      boost::filesystem::create_directory(imagesDir);
    }
    if (!boost::filesystem::exists(topDir)) {
      boost::filesystem::create_directory(topDir);
    }
    if (!boost::filesystem::exists(bottomDir)) {
      boost::filesystem::create_directory(bottomDir);
    }
    string imageStr;
    if (activeCamera == CameraId::headTop) {
      imageStr = topDir + "/" + DataUtils::getCurrentDateTime() + ".jpg";
    } else {
      imageStr = bottomDir + "/" + DataUtils::getCurrentDateTime() + ".jpg";
    }
    LOG_INFO("Writing image: "  << imageStr);
    Mat image = FeatureExtraction::getBgrMat(GET_DVAR(int, debugImageIndex));
    imwrite(imageStr, image);
  }
}
#endif
