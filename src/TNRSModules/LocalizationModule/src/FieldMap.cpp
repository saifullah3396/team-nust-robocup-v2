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
#include "LocalizationModule/include/ObstacleTracker.h"
#include "LocalizationModule/include/ParticleFilter.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/DataHolders/Obstacle.h"
#include "Utils/include/DataHolders/OccupancyMap.h"
#include "Utils/include/DataHolders/WorldBallInfo.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/MathsUtils.h"

FieldMap::FieldMap(LocalizationModule* localizationModule) :
  MemoryBase(localizationModule), DebugBase("FieldMap", this),
  localizer(localizationModule->getParticleFilter()),
  unitVecX(toUType(CameraId::count)),
  unitVecY(toUType(CameraId::count)),
  localizationModule(localizationModule),
  cycleTime(localizationModule->getPeriodMinMS() / ((float) 1000))
{
  GET_CONFIG(
    "EnvProperties",
    (float, ballRadius, ballRadius),
  );

  initDebugBase();
  GET_DEBUG_CONFIG(
    LocalizationConfig, FieldMap,
    (int, debug),
    (int, sendParticles),
    (int, sendObstacleMap),
    (int, drawParticles),
    (int, displayOutput),
    (int, displayInfo),
  );
  auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
  const auto& fieldWidth = FIELD_WIDTH_OUT(LocalizationModule);
  const auto& fieldHeight = FIELD_HEIGHT_OUT(LocalizationModule);

  occupancyMap.data = Scalar(0);
  ///< Create the boundary
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
  setupViewVectors();
}

void FieldMap::update()
{
  cout << "updating robot pose..." << endl;
  updateRobotPose2D();
  cout << "updating obstacle trackers..." << endl;
  updateObstacleTrackers();

  cout << "updating occupancy map..." << endl;
  updateOccupancyMap();
  if (GET_DVAR(int, displayInfo)) {
    LOG_INFO("Current robot pose: " << ROBOT_POSE_2D_OUT(LocalizationModule).get().transpose());
    LOG_INFO("Current tracked obstacles: " << trackedObstacles.size());
  }
  if (GET_DVAR(int, displayOutput)) {
    auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
    VisionUtils::displayImage("Occupancy Map", occupancyMap.data, 0.5);
    waitKey(0);
  }
}

void FieldMap::updateRobotPose2D()
{
  auto& robotPose2D = ROBOT_POSE_2D_OUT(LocalizationModule);
  robotPose2D.x() = 0.0;
  robotPose2D.y() = 0.0;
  robotPose2D.theta() = 0.0;
  robotPose2D = localizer->getFilteredState();
  robotPose2D.ct = cos(robotPose2D.getTheta());
  robotPose2D.st = sin(robotPose2D.getTheta());
}

void FieldMap::updateObstacleTrackers()
{
  trackedObstacles.resize(maxObstacleTrackers);
  float currentTime = localizationModule->getModuleTime();
  auto iter = trackedObstacles.begin();
  while (iter != trackedObstacles.end()) {
    if (*iter) {
      if (currentTime - (*iter)->getLastUpdateTime() < refreshTime) {
        ++iter;
      } else {
        cout << "erased dye to time" << endl;
        iter = trackedObstacles.erase(iter);
      }
    } else {
      iter = trackedObstacles.erase(iter);
    }
  }

  const auto& obstacles = OBSTACLES_OBS_IN(LocalizationModule);
  if (
    obstacles.id != 0 &&
    obstacles.id != prevObsId &&
    !obstacles.data.empty())
  {
    cout << "obstacles observed: " << obstacles.data.size() << endl;
    auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
    for (const auto& obs : obstacles.data) {
      if (obs.centerT.x != obs.centerT.x)
        continue;
      // Update new regions and check if a obstacle already exists in the vicinity
      auto closest = 1000;
      boost::shared_ptr<ObstacleTracker> bestMatch;
      cout << "obs: " << obs.centerT << endl;
      cout << "obs.depth: "<< obs.depth << endl;
      for (auto& to : trackedObstacles) {
        const auto& state = to->getState();
        auto dx = obs.centerT.x - state[0];
        auto dy = obs.centerT.y - state[1];
        cout << "toX: " << state[0] << endl;
        cout << "toY: " << state[1] << endl;
        auto dist = sqrt(dx * dx + dy * dy);
        auto distRatio = 1 - (obs.depth - dist) / obs.depth;
        cout << "dist: " << dist << endl;
        cout << "distRatio: " << distRatio << endl;
        if (distRatio > 0 && distRatio < obstacleMatchMaxDistanceRatio && dist < closest) {
          closest = dist;
          bestMatch = to;
        }
      }

      cout << "best match not found." << endl;
      if (bestMatch &&
          (obs.type == bestMatch->getObstacleType() ||
          obs.type == ObstacleType::unknown))
      {
        Matrix<float, 2, 1> meas;
        meas[0] = obs.centerT.x;
        meas[1] = obs.centerT.y;
        auto currentTime = localizationModule->getModuleTime();
        bestMatch->update(meas, currentTime);
      } else {
        cout << "Adding new obstacle tracker..."  << endl;
        bool addNewTracker = false;
        if (trackedObstacles.size() < maxObstacleTrackers) {
          addNewTracker = true;
        } else if (norm(obs.centerT) < trackedObstacles.back()->getDist()) {
          addNewTracker = true;
        }
        if (addNewTracker) {
          Matrix<float, 4, 1> state;
          state[0] = obs.centerT.x;
          state[1] = obs.centerT.y;
          state[2] = 0.0;
          state[3] = 0.0;
          auto newTracked = boost::shared_ptr<ObstacleTracker>(new ObstacleTracker(obs.type, obs.depth));
          newTracked->init(state, this->cycleTime, localizationModule->getModuleTime());
          trackedObstacles.push_back(newTracked);
        }
      }
      circle(occupancyMap.data, worldToMap(Point2f(obs.centerT)), obs.depth / occupancyMap.resolution, Scalar(0, 0, 0), -1);
      VisionUtils::displayImage("Occupancy Map", occupancyMap.data, 0.5);
      waitKey(0);
    }
    sort(trackedObstacles.begin(), trackedObstacles.end(), [](
      const boost::shared_ptr<ObstacleTracker>& tr1,
      const boost::shared_ptr<ObstacleTracker>& tr2)
    { return tr1->getDist() < tr2->getDist();});
  }
  prevObsId = obstacles.id;

  iter = trackedObstacles.begin();
  while (iter != trackedObstacles.end()) {
    if (*iter && obstacleInFOV(*iter)) {
      if (currentTime - (*iter)->getLastUpdateTime() < refreshTimeForObsInFOV) {
        ++iter;
      } else {
        iter = trackedObstacles.erase(iter);
      }
    } else {
      ++iter;
    }
  }
}

bool FieldMap::obstacleInFOV(const boost::shared_ptr<ObstacleTracker>& obsTracker)
{
  return false;
}

void FieldMap::updateOccupancyMap()
{
  //CameraId activeCamera = CameraId::headTop;
  //Matrix<float, 4, 4> T;
  //if (activeCamera == CameraId::headTop)
  //  T = UPPER_CAM_TRANS_IN(LocalizationModule);
  //else
  //  T = LOWER_CAM_TRANS_IN(LocalizationModule);
  //Matrix<float, 3, 1> unitX = T.block(0, 0, 3, 3) * unitVecX[toUType(activeCamera)];
  //Matrix<float, 3, 1> unitY = T.block(0, 0, 3, 3) * unitVecY[toUType(activeCamera)];
  /*minDistGround =
    slopes[static_cast<int>(CameraId::headBottom)] *
      (0.0 - T[static_cast<int>(CameraId::headBottom)](2, 3)) +
      T[static_cast<int>(CameraId::headBottom)](0, 3);
  */

  ///< Check if robot is localized
  //if (!localizer->isLocalized()) return;
  auto& occupancyMap = OCCUPANCY_MAP_OUT(LocalizationModule);
  occupancyMap.data = occupancyMapBase.clone();
  for (auto& to : trackedObstacles) {
    const auto& state = to->getState();
    cout << "to->getRadius() / occupancyMap.resolution: "<< to->getRadius() / occupancyMap.resolution << endl;
    circle(occupancyMap.data, worldToMap(Point2f(state[0], state[1])), to->getRadius() / occupancyMap.resolution, Scalar(0, 0, 0), -1);
  }

  if (addBallObstacle) {
    ///< Draw ball as an obstacle if required
    const auto& wbInfo = WORLD_BALL_INFO_IN(LocalizationModule);
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

void FieldMap::setupViewVectors()
{
  Vector2f fovX(toUType(CameraId::count));
  Vector2f fovY(toUType(CameraId::count));
  #ifndef NAOQI_VIDEO_PROXY_AVAILABLE
  auto settings =
    JsonUtils::readJson(ConfigManager::getConfigDirPath() + "CameraSettings.json");
  #else
  auto settings =
    JsonUtils::readJson(ConfigManager::getConfigDirPath() + "NaoqiCameraSettings.json");
  #endif
  JsonUtils::jsonToType(fovX[toUType(CameraId::headTop)], settings["visionTop"]["intrinsic"]["fovX"], 0.0);
  JsonUtils::jsonToType(fovY[toUType(CameraId::headTop)], settings["visionTop"]["intrinsic"]["fovY"], 0.0);
  JsonUtils::jsonToType(fovX[toUType(CameraId::headBottom)], settings["visionTop"]["intrinsic"]["fovX"], 0.0);
  JsonUtils::jsonToType(fovY[toUType(CameraId::headBottom)], settings["visionTop"]["intrinsic"]["fovY"], 0.0);
  fovX *= MathsUtils::DEG_TO_RAD;
  fovY *= MathsUtils::DEG_TO_RAD;
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    unitVecX[i][1] = 0;
    unitVecY[i][1] = 0;
    if (i == static_cast<int>(CameraId::headTop)) {
      unitVecY[i][1] = -sin(fovY[i] / 2);
      unitVecX[i][0] = -cos(fovX[i] / 2);
    } else {
      unitVecY[i][1] = sin(fovY[i] / 2);
      unitVecX[i][0] = cos(fovX[i] / 2);
    }
  }
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
