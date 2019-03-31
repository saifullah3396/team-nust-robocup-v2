/**
 * @file LocalizationModule/src/FieldMap.cpp
 *
 * The class for generating the map for robot localization.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 28 Sep 2017
 */

#include <boost/make_shared.hpp>
#include "LocalizationModule/include/FieldMap.h"
#include "LocalizationModule/include/LocalizationModule.h"
#include "LocalizationModule/include/ParticleFilter.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/ConfigMacros.h"

FieldMap::FieldMap(LocalizationModule* localizationModule) :
  MemoryBase(localizationModule), DebugBase("FieldMap", this),
  localizer(localizationModule->getParticleFilter())
{
  initDebugBase();
  int tempDebug;
  int tempSendParticles;
  int tempSendObstacleMap;
  int tempDrawParticles;
  int tempDisplayOutput;
  GET_CONFIG(
    "LocalizationConfig",
    (int, FieldMap.debug, tempDebug),
    (int, FieldMap.sendParticles, tempSendParticles),
    (int, FieldMap.sendObstacleMap, tempSendObstacleMap),
    (int, FieldMap.drawParticles, tempDrawParticles),
    (int, FieldMap.displayOutput, tempDisplayOutput),
  )
  SET_DVAR(int, debug, tempDebug);
  SET_DVAR(int, sendParticles, tempSendParticles);
  SET_DVAR(int, sendObstacleMap, tempSendObstacleMap);
  SET_DVAR(int, drawParticles, tempDrawParticles);
  SET_DVAR(int, displayOutput, tempDisplayOutput);
  GET_CONFIG("EnvProperties", (float, ballRadius, ballRadius),);

  auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
  const auto& fieldWidth = FIELD_WIDTH_OUT(LocalizationModule);
  const auto& fieldHeight = FIELD_HEIGHT_OUT(LocalizationModule);
  auto mapWidth = fieldWidth / occupancyMap.resolution;
  auto mapHeight = fieldHeight / occupancyMap.resolution;
  //worldImage = Mat(Size(mapWidth, mapHeight), CV_8UC3);
  //mapDrawing = Mat(Size(mapWidth, mapHeight), CV_8UC3);
  //drawField();

  occupancyMap.data = Scalar(0);
  //! Create the boundary
  rectangle(
    occupancyMap.data,
    cv::Point(
      0.25 / occupancyMap.resolution,
      0.25 / occupancyMap.resolution),
    cv::Point(
      (fieldWidth - 0.25) / occupancyMap.resolution,
      (fieldHeight - 0.25) / occupancyMap.resolution),
    cv::Scalar(255),
    -1);
  occupancyMapBase = occupancyMap.data;
}

void FieldMap::update()
{
  updateRobotPose2D();
  updateOccupancyMap();
  //drawRobot();
  //updateWorldImage();
}

void FieldMap::updateRobotPose2D()
{
  auto& robotPose2D = ROBOT_POSE_2D_OUT(LocalizationModule);
  robotPose2D = localizer->getFilteredState();
  robotPose2D.ct = cos(robotPose2D.getTheta());
  robotPose2D.st = sin(robotPose2D.getTheta());
}

void FieldMap::updateOccupancyMap()
{
  //! Check if robot is localized
  //if (!localizer->isLocalized()) return;
  auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
  occupancyMap.data = occupancyMapBase.clone();
  const auto& obstaclesObs = OBSTACLES_OBS_IN(LocalizationModule);
  if (
    obstaclesObs.id != 0 &&
    obstaclesObs.id != prevObsId &&
    !obstaclesObs.data.empty())
  {
    for (auto& obs : obstaclesObs.data) {
      Point obstaclePoly[4];
      obstaclePoly[0] = worldToMap(obs.frontT.p1);
      obstaclePoly[1] = worldToMap(obs.backT.p1);
      obstaclePoly[2] = worldToMap(obs.backT.p2);
      obstaclePoly[3] = worldToMap(obs.frontT.p2);
      //! Draw the obstacles on the occupancy map
      fillConvexPoly(occupancyMap.data, obstaclePoly, 4, 0, 8, 0);
    }
  }
  prevObsId = obstaclesObs.id;

  if (addBallObstacle) {
    //! Draw ball as an obstacle if required
    auto wbInfo = WORLD_BALL_INFO_IN(LocalizationModule);
    if (wbInfo.found) {
      circle(
        occupancyMap.data,
        worldToMap(wbInfo.posWorld),
        ballRadius / occupancyMap.resolution,
        Scalar(0, 0, 0),
        -1);
    }
  }
}

template<typename T>
cv::Point_<int> FieldMap::worldToMap(const cv::Point_<T>& p)
{
  auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
  return
    cv::Point_<int> (
      p.x / occupancyMap.resolution + occupancyMap.originPose.x, p.y /
      occupancyMap.resolution + occupancyMap.originPose.y);
}
/*
void FieldMap::updateWorldImage()
{
  if (GET_DVAR(int, debug)) {
    if (GET_DVAR(int, sendParticles)) {
      vector<Particle> particles = localizer->getParticles();
      if (particles.size() > 0) {
        Mat_<float> particleData(particles.size(), 2);
        for (size_t i = 0; i < particles.size(); ++i) {
          particleData(i, 0) = particles[i].state.getX();
          particleData(i, 1) = particles[i].state.getY();
        }
        const unsigned char* ptr = particleData.data;
        int count = particles.size() * 8;
        // Change now to Comm message request
        CommMessage msg = CommMessage(DataUtils::bytesToHexString(ptr, count), CommMsgTypes::pfStates);
        UserCommRequestPtr request = boost::make_shared<SendMsgRequest>(msg);
        BaseModule::publishModuleRequest(request);
      }
    }
  }
  if (GET_DVAR(int, drawParticles)) {
    worldImage = mapDrawing.clone();
    vector<Particle> particles = localizer->getParticles();

    for (unsigned i = 0; i < particles.size(); ++i) {
      //cout << "particle[" << i << "]: " << "[" << particles[i].state.x << "," << particles[i].state.y << "," <<  particles[i].state.theta << endl;
      circle(
        worldImage,
        Point(
          mapWidth / 2 + particles[i].state.x() * 100,
          mapHeight / 2 - particles[i].state.y() * 100),
        10,
        Scalar(0, 0, 0));
      line(
        worldImage,
        Point(
          mapWidth / 2 + particles[i].state.x() * 100,
          mapHeight / 2 - particles[i].state.y() * 100),
        Point(
          mapWidth / 2 + particles[i].state.x() * 100 + 15 * cos(
            particles[i].state.theta()),
          mapHeight / 2 - particles[i].state.y() * 100 + -15 * sin(
            particles[i].state.theta())),
        Scalar(255, 255, 255));
    }
    if (WORLD_BALL_INFO_IN(LocalizationModule).found) {
      Point2f ballWorld = WORLD_BALL_INFO_IN(LocalizationModule).posWorld;
      circle(
        worldImage,
        Point(
          mapWidth / 2 + ballWorld.x * 100,
          mapHeight / 2 - ballWorld.y * 100),
        5,
        Scalar(0, 0, 0));
    }

    auto& lastknown = LAST_POSE_2D_OUT(LocalizationModule);
    if (lastknown.x() != 1e3) {
      circle(
        worldImage,
        Point(
          mapWidth / 2 + lastknown.x() * 100,
          mapHeight / 2 - lastknown.y() * 100),
        10,
        Scalar(0, 0, 255));
      line(
        worldImage,
        Point(
          mapWidth / 2 + lastknown.x() * 100,
          mapHeight / 2 - lastknown.y() * 100),
        Point(
          mapWidth / 2 + lastknown.x() * 100 + 15 * cos(lastknown.theta()),
          mapHeight / 2 - lastknown.y() * 100 + -15 * sin(lastknown.theta())),
        Scalar(0, 0, 255));
    }
  }

  if (GET_DVAR(int, displayOutput))
    VisionUtils::displayImage("worldImage", worldImage);
}

void
FieldMap::drawField()
{
  int width = mapWidth;
  int height = mapHeight;
  mapDrawing = Scalar(0, 100, 0); //BGR
  int borderDiff = 50;
  rectangle(
    mapDrawing,
    Point(borderDiff, borderDiff),
    Point(width - borderDiff, height - borderDiff),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(borderDiff, height / 2 - 220 / 2),
    Point(borderDiff + 60, height / 2 + 220 / 2),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(width - borderDiff, height / 2 - 220 / 2),
    Point(width - borderDiff - 60, height / 2 + 220 / 2),
    Scalar(255, 255, 255),
    5);
  rectangle(
    mapDrawing,
    Point(borderDiff + 130 - 6, height / 2 + 3),
    Point(borderDiff + 130 + 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(borderDiff + 130 - 3, height / 2 + 6),
    Point(borderDiff + 130 + 3, height / 2 - 6),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width - borderDiff - 130 + 6, height / 2 + 3),
    Point(width - borderDiff - 130 - 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width - borderDiff - 130 + 3, height / 2 + 6),
    Point(width - borderDiff - 130 - 3, height / 2 - 6),
    Scalar(255, 255, 255),
    -1);
  rectangle(
    mapDrawing,
    Point(width / 2 - 6, height / 2 + 3),
    Point(width / 2 + 6, height / 2 - 3),
    Scalar(255, 255, 255),
    -1);
  line(
    mapDrawing,
    Point(width / 2, 5),
    Point(width / 2, height - 5),
    Scalar(255, 255, 255),
    5);
  circle(
    mapDrawing,
    Point(width / 2, height / 2),
    75,
    Scalar(255, 255, 255),
    5);
}

void
FieldMap::drawRobot()
{
  RobotPose2D<float> rState = localizer->getFilteredState();
  circle(mapDrawing, Point(rState.x(), rState.y()), 10, Scalar(255, 0, 0));
  line(
    mapDrawing,
    Point(rState.x(), rState.y()),
    Point(rState.x() + 15 * cos(rState.theta()), rState.y() + 15 * sin(rState.theta())),
    Scalar(0, 0, 0));
}
*/
