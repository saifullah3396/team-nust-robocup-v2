/**
 * @file LocalizationModule/src/LocalizationModule.cpp
 *
 * This file declares a class for the complete robot localization.
 * All the functions and algorithms for robot state estimation and
 * pose determination will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 11 Feb 2017
 */

#include <boost/make_shared.hpp>
#include "LocalizationModule/include/LocalizationModule.h"
#include "LocalizationModule/include/LocalizerStates.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "LocalizationModule/include/ParticleFilter.h"
#include "LocalizationModule/include/FieldMap.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/BallInfo.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/PlanningState.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/DataHolders/RoboCupGameControlData.h"
#include "Utils/include/ConfigMacros.h"

DEFINE_INPUT_CONNECTOR(LocalizationModule,
  (int, localizationThreadPeriod),
  (RoboCupGameControlData, gameData),
  (Matrix4f, upperCamInFeet),
  (Matrix4f, lowerCamInFeet),
  (PlanningState, planningState),
  (ObsObstacles<float>, obstaclesObs),
  (GoalInfo<float>, goalInfo),
  (BallInfo<float>, ballInfo),
  (WorldBallInfo<float>, worldBallInfo),
  (bool, robotOnSideLine),
  (bool, localizeWithLastKnown),
  (bool, robotInMotion),
);
DEFINE_OUTPUT_CONNECTOR(LocalizationModule,
  (RobotPose2D<float>, robotPose2D),
  (RobotPose2D<float>, lastKnownPose2D),
  (OccupancyMap<float>, occupancyMap),
  (bool, robotLocalized),
  (int, positionConfidence),
  (int, sideConfidence),
  (float, fieldWidth),
  (float, fieldHeight),
);

LocalizationModule::LocalizationModule(void* processingModule) :
  BaseModule(
    processingModule,
    TNSPLModules::localization,
    "LocalizationModule")
{
}

void LocalizationModule::setThreadPeriod()
{
  setPeriodMinMS(LOCALIZATION_PERIOD_IN(LocalizationModule));
}

void LocalizationModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void LocalizationModule::init()
{
  LOG_INFO("Initializing LocalizationModule Output variables...")
  OccupancyMap<float> defMap;
  float fW, fH;
  GET_CONFIG(
    "EnvProperties",
    (float, Map.cellSize, defMap.resolution),
    (float, Map.fieldWidth, fW),
    (float, Map.fieldHeight, fH),
  );
  defMap.data = Mat(
    Size(fW / defMap.resolution, fH / defMap.resolution),
    CV_8UC1,
    Scalar(0));
  defMap.originPose = cv::Point3_<float>(
    fW / 2.0 / defMap.resolution,
    fH / 2.0 / defMap.resolution,
    0.0);
  FIELD_WIDTH_OUT(LocalizationModule) = fW;
  FIELD_HEIGHT_OUT(LocalizationModule) = fH;
  OCCUPANCY_MAP_OUT(LocalizationModule) = defMap;
  ROBOT_POSE_2D_OUT(LocalizationModule) = RobotPose2D<float>(0.0, 0.0, 0.0);
  LAST_POSE_2D_OUT(LocalizationModule) = RobotPose2D<float>(0.0, 0.0, 0.0);
  ROBOT_LOCALIZED_OUT(LocalizationModule) = false;
  POSITION_CONFIDENCE_OUT(LocalizationModule) = 0;
  SIDE_CONFIDENCE_OUT(LocalizationModule) = 0;

  LOG_INFO("Initializing ParticleFilter...")
  particleFilter =
    boost::shared_ptr<ParticleFilter>(new ParticleFilter(this));
  LOG_INFO("Initializing FieldMap...")
  fieldMap = boost::make_shared<FieldMap>(this);
  //particleFilter->init(RobotPose2D<float>(0.f, 0.f, 0.f));
}

void LocalizationModule::handleRequests()
{
  while (!inRequests.isEmpty()) {
    auto request = inRequests.queueFront();
    if (boost::static_pointer_cast <LocalizationRequest>(request)) {
      auto reqId = request->getRequestId();
      if (reqId == (unsigned)LocalizationRequestIds::switchLocalization) {
        auto sl = boost::static_pointer_cast <SwitchLocalization>(request);
        runLocalization = sl->state;
      } else if (reqId == (unsigned)LocalizationRequestIds::switchParticleFilter) {
        auto spf = boost::static_pointer_cast <SwitchParticleFilter>(request);
        updateFilter = spf->state;
      } else if (reqId == (unsigned)LocalizationRequestIds::switchFieldMap) {
        auto spf = boost::static_pointer_cast <SwitchFieldMap>(request);
        updateMap = spf->state;
      } else if (reqId == (unsigned)LocalizationRequestIds::switchBallObstacle) {
        auto sbo = boost::static_pointer_cast <SwitchBallObstacle>(request);
        fieldMap->setBallObstacle(sbo->state);
      } else if (reqId == (unsigned)LocalizationRequestIds::initiateLocalizer) {
        auto il = boost::static_pointer_cast <InitiateLocalizer>(request);
        particleFilter->init(il->pose2D);
        fieldMap->updateRobotPose2D();
      } else if (reqId == (unsigned)LocalizationRequestIds::resetLocalizer) {
        particleFilter->reset();
      } else if (reqId == (unsigned)LocalizationRequestIds::positionUpdate) {
        auto pu = boost::static_pointer_cast <PositionUpdate>(request);
        particleFilter->addPositionInput(pu->input);
      } else if (reqId == (unsigned)LocalizationRequestIds::knownLandmarksUpdate) {
        auto klu = boost::static_pointer_cast <KnownLandmarksUpdate>(request);
        particleFilter->setKnownLandmarks(klu->landmarks);
      } else if (reqId == (unsigned)LocalizationRequestIds::unknownLandmarksUpdate) {
        auto ulu = boost::static_pointer_cast <UnknownLandmarksUpdate>(request);
        particleFilter->setUnknownLandmarks(ulu->landmarks);
      }
    }
    inRequests.popQueue();
  }
}

void LocalizationModule::mainRoutine()
{
  // Execution of this module is decided by planning module.
  if (runLocalization) {
    if (updateFilter)
      particleFilter->update();
    if (updateMap)
      fieldMap->update();
    ROBOT_LOCALIZED_OUT(LocalizationModule) =
      particleFilter->isLocalized();
  }
}
