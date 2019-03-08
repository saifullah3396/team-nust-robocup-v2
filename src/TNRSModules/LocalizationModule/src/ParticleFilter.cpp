/**
 * @file LocalizationModule/src/ParticleFilter.cpp
 *
 * The file implements the class ParticleFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 17 Sep 2017
 */

#include <boost/make_shared.hpp>
#include "LocalizationModule/include/LocalizationModule.h"
#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "LocalizationModule/include/FieldLandmarkIds.h"
#include "LocalizationModule/include/ParticleFilter.h"
#include "LocalizationModule/include/Particle.h"
#include "RandomLib/include/RandomSelect.hpp"
#include "RandomLib/include/NormalDistribution.hpp"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/DataHolders/GoalInfo.h"
#include "Utils/include/DataHolders/Landmark.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/TeamPositions.h"
#include "Utils/include/VisionUtils.h"

#define HALF_FIELD_HEIGHT 4.5
#define HALF_FIELD_WIDTH 3.0
#define MAX_LOST_COUNT 10
#define ALPHA_SLOW .01 //0.05
#define ALPHA_FAST .1

ParticleFilter::ParticleFilter(LocalizationModule* lModule) :
  MemoryBase(lModule),
  DebugBase("ParticleFilter", this),
  lModule(lModule),
  cycleTime(lModule->getPeriodMinMS() / (1000.f)),
  nParticles(20),
  lostCount(0),
  //recheckCount(0),
  //inRecheck(false),
  initiated(false),
  localized(false),
  lastKnownHalf(0),
  unitVecY(toUType(CameraId::count)),
  prevGoalInfoId(0),
  avgFilteredState(RobotPose2D<float>(NAN, NAN, NAN)),
  landmarkTypeCount(NUM_LANDMARK_TYPES),
  landmarkTypeStarts(NUM_LANDMARK_TYPES)
{
  initDebugBase();
  int tempDisplayLandmarksMap;
  int tempDisplayVoronoiMap;
  GET_CONFIG(
    "LocalizationConfig",
    (int, ParticleFilter.displayLandmarksMap, tempDisplayLandmarksMap),
    (int, ParticleFilter.displayVoronoiMap, tempDisplayVoronoiMap),
  )
  SET_DVAR(int, displayLandmarksMap, tempDisplayLandmarksMap);
  SET_DVAR(int, displayVoronoiMap, tempDisplayVoronoiMap);


  GET_CONFIG(
    "LocalizationConfig",
    (double, ParticleFilter.genStdX, genStd[0]),
    (double, ParticleFilter.genStdY, genStd[1]),
    (double, ParticleFilter.genStdTheta, genStd[2]),
    (double, ParticleFilter.motionStdX, motionStd[0]),
    (double, ParticleFilter.motionStdY, motionStd[1]),
    (double, ParticleFilter.motionStdTheta, motionStd[2]),
    (double, ParticleFilter.predStdX, predictionStd[0]),
    (double, ParticleFilter.predStdY, predictionStd[1]),
    (double, ParticleFilter.predStdTheta, predictionStd[2]),
    (double, ParticleFilter.measStdX, measurementStd[0]),
    (double, ParticleFilter.measStdY, measurementStd[1]),
    (double, ParticleFilter.measStdTheta, measurementStd[2]),
    (double, ParticleFilter.lGridResolution, lGridResolution),
  );

  gaussianConst = 0.5 / (M_PI * measurementStd[0] * measurementStd[1]);
  expConstX = -0.5 / (measurementStd[0] * measurementStd[0]);
  expConstY = -0.5 / (measurementStd[1] * measurementStd[1]);
  estimatedStates.set_capacity(20);
  setupLandmarks();
  setupVoronoiMap();
  setupViewVectors();
  reset();
}

void ParticleFilter::init(const RobotPose2D<float>& state)
{
  NormalDistribution<double> dist;
  nParticles = 20;
  particles.resize(nParticles);
  float w = 1.0 / nParticles;
  for (size_t i = 0; i < nParticles; ++i) {
    for (size_t j = 0; j < state.get().size(); ++j) {
      particles[i].state.get()[j] =
        dist(random, state.get()[j], genStd[j]);
    }
    particles[i].weight = w;
  }
  avgFilteredState = state;
  wSlow = 0.0, wFast = 0.0;
  SIDE_CONFIDENCE_OUT(LocalizationModule) = 100;
  POSITION_CONFIDENCE_OUT(LocalizationModule) = 100;
  initiated = true;
}

void ParticleFilter::init(const boost::circular_buffer<RobotPose2D<float> >& states)
{
  NormalDistribution<double> dist;
  nParticles = 20;
  particles.resize(nParticles);
  float w = 1.0 / nParticles;
  avgFilteredState = RobotPose2D<float>(0.f, 0.f, 0.f);
  int divider = nParticles / states.size();
  for (size_t i = 0; i < nParticles; ++i) {
    for (size_t j = 0; j < particles[i].state.get().size(); ++j) {
      particles[i].state.get()[j] =
        dist(random, states[i / divider].get()[j], genStd[j]);
    }
    particles[i].weight = w;
    avgFilteredState += particles[i].state;
  }
  avgFilteredState *= (1 / nParticles);
  wSlow = 0.0, wFast = 0.0;
  SIDE_CONFIDENCE_OUT(LocalizationModule) = 100;
  POSITION_CONFIDENCE_OUT(LocalizationModule) = 100;
  initiated = true;
}

void ParticleFilter::update()
{
  if (ON_SIDE_LINE_IN(LocalizationModule)) {
    if (estimatedStates.capacity() != 20) estimatedStates.set_capacity(20);
  } else {
    if (estimatedStates.capacity() != 10) estimatedStates.set_capacity(10);
  }
  if (!initiated) {
    //! If the particle filter is not initiated and we do not have any info
    //! on the robot position, try to get an estimate using observed landmarks info
    updateRobotStateEstimate();
  } else {
    //! Keep updating the estimate for correction
    //updateRobotStateEstimate();
    //! Set the particles to 20 if the robot is localized
    if (localized)
      nParticles = 20;
    vector<LandmarkPtr> obsLandmarks;
    obsLandmarks.insert(obsLandmarks.begin(), knownLandmarks.begin(), knownLandmarks.end());
    obsLandmarks.insert(obsLandmarks.begin(), unknownLandmarks.begin(), unknownLandmarks.end());
    //cout << "observed landmarks..." << obsLandmarks.size() << endl;
    prediction();
    if (obsLandmarks.size() > 5) {// && !IVAR(bool, LocalizationModule::robotInMotion)) {
      updateWeights(obsLandmarks);
      normalizeWeights(false);
      addPredictionNoise();
    } else {
      updateWeights (vector<LandmarkPtr>());
      normalizeWeights(true);
    }
    updateSideConfidence();
    updatePositionConfidence();
    resample();
  }
  //cout << "lastKnownHalf: " << lastKnownHalf << endl;
  knownLandmarks.clear();
  unknownLandmarks.clear();
}

void ParticleFilter::reset()
{
  particles.clear();
  estimatedStates.clear();
  initiated = false;
  localized = false;
  lostCount = 0;
  POSITION_CONFIDENCE_OUT(LocalizationModule) = 0;
  SIDE_CONFIDENCE_OUT(LocalizationModule) = 0;
}

void ParticleFilter::setupLandmarks()
{
  landmarkTypeCount[FL_TYPE_GOAL_POST] = FL_GOAL_POSTS;
  landmarkTypeCount[FL_TYPE_T_CORNER] = FL_T_CORNERS;
  landmarkTypeCount[FL_TYPE_LINES] = 0;
  landmarkTypeCount[FL_TYPE_L_CORNER] = FL_L_CORNERS;
  landmarkTypeCount[FL_TYPE_CIRCLE] = FL_CIRCLES;
  landmarkTypeCount[FL_TYPE_PENALTY_MARK] = FL_PENALTY_MARKS;
  landmarkTypeStarts[FL_TYPE_GOAL_POST] = FL_LT_GOALPOST;
  landmarkTypeStarts[FL_TYPE_T_CORNER] = FL_GOAL_POSTS;
  landmarkTypeStarts[FL_TYPE_LINES] = 0;
  landmarkTypeStarts[FL_TYPE_L_CORNER] =
    FL_GOAL_POSTS + FL_T_CORNERS;
  landmarkTypeStarts[FL_TYPE_CIRCLE] =
    FL_GOAL_POSTS + FL_T_CORNERS + FL_L_CORNERS;
  landmarkTypeStarts[FL_TYPE_PENALTY_MARK] =
    FL_GOAL_POSTS + FL_T_CORNERS + FL_L_CORNERS + FL_CIRCLES;

  for (size_t i = 0; i < FL_GOAL_POSTS; ++i) {
    fieldLandmarks.push_back(
      boost::make_shared<Landmark<float> >(
        static_cast<unsigned>(FL_TYPE_GOAL_POST),
        cv::Point_<float>(landmarksOnField[i].getX(), landmarksOnField[i].getY()))
      );
    fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
  }
  vector<Point2f> fieldCornerPs;
  fieldCornerPs.push_back(Point2f(HALF_FIELD_HEIGHT, HALF_FIELD_WIDTH));
  fieldCornerPs.push_back(Point2f(HALF_FIELD_HEIGHT, -HALF_FIELD_WIDTH));
  fieldCornerPs.push_back(Point2f(-HALF_FIELD_HEIGHT, -HALF_FIELD_WIDTH));
  fieldCornerPs.push_back(Point2f(-HALF_FIELD_HEIGHT, HALF_FIELD_WIDTH));
  vector<Point2f> midLinePs;
  midLinePs.push_back(Point2f(0.0, HALF_FIELD_WIDTH));
  midLinePs.push_back(Point2f(0.0, -HALF_FIELD_WIDTH));
  vector<Point2f> goalBox1Ps;
  goalBox1Ps.push_back(Point2f(HALF_FIELD_HEIGHT, 1.1));
  goalBox1Ps.push_back(Point2f(3.9, 1.1));
  goalBox1Ps.push_back(Point2f(3.9, -1.1));
  goalBox1Ps.push_back(Point2f(HALF_FIELD_HEIGHT, -1.1));
  vector<Point2f> goalBox2Ps;
  goalBox2Ps.push_back(Point2f(-HALF_FIELD_HEIGHT, 1.1));
  goalBox2Ps.push_back(Point2f(-3.9, 1.1));
  goalBox2Ps.push_back(Point2f(-3.9, -1.1));
  goalBox2Ps.push_back(Point2f(-HALF_FIELD_HEIGHT, -1.1));

  for (size_t i = 0; i < fieldCornerPs.size(); ++i) {
    int k = i + 1;
    k = k == fieldCornerPs.size() ? 0 : k;
    float ratio = ceil(
      norm(fieldCornerPs[k] - fieldCornerPs[i]) / lGridResolution);
    for (size_t j = 0; j < ratio; ++j) {
      fieldLandmarks.push_back(
        boost::make_shared<Landmark<float> >(
          FL_TYPE_LINES,
          Point2f(
            fieldCornerPs[i].x + (fieldCornerPs[k].x - fieldCornerPs[i].x) * j / ratio,
            fieldCornerPs[i].y + (fieldCornerPs[k].y - fieldCornerPs[i].y) * j / ratio
          )
        )
      );
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
  for (size_t i = 0; i < goalBox1Ps.size(); ++i) {
    int k = i + 1;
    if (k == goalBox1Ps.size()) {
      break;
    }
    float ratio = ceil(norm(goalBox1Ps[k] - goalBox1Ps[i]) / lGridResolution);
    for (size_t j = 0; j < ratio; ++j) {
      fieldLandmarks.push_back(
        boost::make_shared<Landmark<float> >(
          FL_TYPE_LINES,
          Point2f(
            goalBox1Ps[i].x + (goalBox1Ps[k].x - goalBox1Ps[i].x) * j / ratio,
            goalBox1Ps[i].y + (goalBox1Ps[k].y - goalBox1Ps[i].y) * j / ratio
          )
        )
      );
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
  for (size_t i = 0; i < goalBox2Ps.size(); ++i) {
    int k = i + 1;
    if (k == goalBox2Ps.size()) {
      break;
    }
    float ratio = ceil(norm(goalBox2Ps[k] - goalBox2Ps[i]) / lGridResolution);
    for (size_t j = 0; j < ratio; ++j) {
      fieldLandmarks.push_back(
        boost::make_shared<Landmark<float> >(
          FL_TYPE_LINES,
          Point2f(
            goalBox2Ps[i].x + (goalBox2Ps[k].x - goalBox2Ps[i].x) * j / ratio,
            goalBox2Ps[i].y + (goalBox2Ps[k].y - goalBox2Ps[i].y) * j / ratio
          )
        )
      );
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
  float ratio = ceil(norm(midLinePs[1] - midLinePs[0]) / lGridResolution);
  for (size_t j = 1; j < ratio; ++j) {
    fieldLandmarks.push_back(
      boost::make_shared<Landmark<float> >(
        FL_TYPE_LINES,
        Point2f(
          midLinePs[0].x + (midLinePs[1].x - midLinePs[0].x) * j / ratio,
          midLinePs[0].y + (midLinePs[1].y - midLinePs[0].y) * j / ratio
        )
      )
    );
    fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
  }

  float eRadius = 0.75;
  ratio = 12;
  for (size_t i = 0; i <= ratio; ++i) {
    float eX = eRadius * cos(M_PI * i / ratio);
    float eY = eRadius * sin(M_PI * i / ratio);
    fieldLandmarks.push_back(
      boost::make_shared<Landmark<float> >(FL_TYPE_LINES, Point2f(eX, eY)));
    fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    if (i != 0 && i != ratio) {
      fieldLandmarks.push_back(
        boost::make_shared<Landmark<float> >(FL_TYPE_LINES, Point2f(eX, -eY)));
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
}

void ParticleFilter::setupVoronoiMap()
{
  vector<Point2f> voronoiMapPs;
  voronoiMapPs.push_back(Point2f(HALF_FIELD_HEIGHT, 0.0));
  voronoiMapPs.push_back(Point2f(3.9, 0.0));
  voronoiMapPs.push_back(Point2f(HALF_FIELD_HEIGHT, 2.05));
  voronoiMapPs.push_back(Point2f(HALF_FIELD_HEIGHT, -2.05));
  voronoiMapPs.push_back(Point2f(HALF_FIELD_HEIGHT / 2, 0.0));
  voronoiMapPs.push_back(Point2f(HALF_FIELD_HEIGHT / 2, HALF_FIELD_WIDTH));
  voronoiMapPs.push_back(Point2f(HALF_FIELD_HEIGHT / 2, -HALF_FIELD_WIDTH));
  voronoiMapPs.push_back(Point2f(-HALF_FIELD_HEIGHT, 0.0));
  voronoiMapPs.push_back(Point2f(-3.9, 0.0));
  voronoiMapPs.push_back(Point2f(-HALF_FIELD_HEIGHT, 2.05));
  voronoiMapPs.push_back(Point2f(-HALF_FIELD_HEIGHT, -2.05));
  voronoiMapPs.push_back(Point2f(-HALF_FIELD_HEIGHT / 2, 0.0));
  voronoiMapPs.push_back(Point2f(-HALF_FIELD_HEIGHT / 2, HALF_FIELD_WIDTH));
  voronoiMapPs.push_back(Point2f(-HALF_FIELD_HEIGHT / 2, -HALF_FIELD_WIDTH));
  voronoiMapPs.push_back(Point2f(0.0, 0.0));
  voronoiMapPs.push_back(Point2f(0.0, 1.875));
  voronoiMapPs.push_back(Point2f(0.0, -1.875));
  //voronoiMapPs.push_back(Point2f(0.75, 0.0));
  //voronoiMapPs.push_back(Point2f(-0.75, 0.0));
  float fieldWidth, fieldHeight;
  GET_CONFIG(
    "PathPlanner",
    (float, Map.cellSize, vMapResolution),
    (float, Map.fieldWidth, fieldWidth),
    (float, Map.fieldHeight, fieldHeight),
  );
  vMapSize = Size(fieldWidth / vMapResolution, fieldHeight / vMapResolution);
  Mat distMap, voronoiColored;
  Mat voronoiPMap = Mat(vMapSize, CV_8UC1, Scalar(255));
  for (size_t j = 0; j < voronoiMapPs.size(); ++j) {
    auto p = worldToVoronoiMap(voronoiMapPs[j]);
    voronoiPMap.at<uchar>(p.y, p.x) = 0;
  }
  distanceTransform(voronoiPMap, distMap, voronoiLabels, CV_DIST_L1, 5);
  int numLabels = 0;
  for (size_t i = 0; i < voronoiLabels.rows; i++) {
    const int* ll = (const int*) voronoiLabels.ptr(i);
    for (size_t j = 0; j < voronoiLabels.cols; j++) {
      numLabels = ll[j] > numLabels ? ll[j] : numLabels;
    }
  }
  labelledLandmarks.resize(numLabels);
  for (size_t i = 0; i < fieldLandmarks.size(); ++i) {
    auto fl = fieldLandmarks[i];
    Point pos = worldToVoronoiMap(fl->pos);
    int landmarkLabel = voronoiLabels.at<uint>(pos.y, pos.x);
    labelledLandmarks[landmarkLabel - 1].push_back(fl);
  }
  if (GET_DVAR(int, displayVoronoiMap) || GET_DVAR(int, displayLandmarksMap)) {
    voronoiColored.create(voronoiLabels.size(), CV_8UC3);
    Scalar colors[] =
    {
      Scalar(0, 0, 0),
      Scalar(255, 0, 0),
      Scalar(255, 128, 0),
      Scalar(255, 255, 0),
      Scalar(0, 255, 0),
      Scalar(0, 128, 255),
      Scalar(0, 255, 255),
      Scalar(0, 0, 255),
      Scalar(255, 0, 255),
      Scalar(255, 255, 255),
      Scalar(255, 255, 128)
    };
    for (size_t i = 0; i < voronoiLabels.rows; i++) {
      const int* ll = (const int*) voronoiLabels.ptr(i);
      uchar* d = (uchar*) voronoiColored.ptr(i);
      for (size_t j = 0; j < voronoiLabels.cols; j++) {
        int idx = ll[j] == 0 ? 0 : (ll[j] - 1) % 10 + 1;
        int b = cvRound(colors[idx][0]);
        int g = cvRound(colors[idx][1]);
        int r = cvRound(colors[idx][2]);
        d[j * 3] = (uchar) b;
        d[j * 3 + 1] = (uchar) g;
        d[j * 3 + 2] = (uchar) r;
      }
    }
    VisionUtils::displayImage("VoronoiMap", voronoiColored);
    if (GET_DVAR(int, displayLandmarksMap)) {
      Mat flImage = Mat(Size(1000, 700), CV_8UC3, Scalar(0));
      for (size_t j = 0; j < fieldLandmarks.size(); ++j) {
       Point pos = Point(fieldLandmarks[j]->pos.x * 100 + 500, 350 - 100 * fieldLandmarks[j]->pos.y);
       Vec3b color = voronoiColored.at<Vec3b>(pos.y, pos.x);
       VisionUtils::drawPoint(pos, flImage, Scalar(color[0],color[1],color[2]));
      }
      VisionUtils::displayImage("flImage", flImage);
    }
    waitKey(0);
  }
}

Point ParticleFilter::worldToVoronoiMap(const Point& p)
{
  return
    Point(
      p.x / vMapResolution + vMapSize.width / 2,
      vMapSize.height  / 2 - p.y / vMapResolution);
}

Point2f ParticleFilter::worldToVoronoiMap(const Point2f& p) {
  return
    Point2f(
      p.x / vMapResolution + vMapSize.width / 2,
      vMapSize.height  / 2 - p.y / vMapResolution);
}

void ParticleFilter::setupViewVectors()
{
  vector<float> fovX(toUType(CameraId::count));
  vector<float> fovY(toUType(CameraId::count));
  GET_CONFIG(
    "UpperCamera",
    (float, Intrinsic.fovX, fovX[static_cast<int>(CameraId::headTop)]),
    (float, Intrinsic.fovY, fovY[static_cast<int>(CameraId::headTop)]),
  );
  GET_CONFIG(
    "LowerCamera",
    (float, Intrinsic.fovX, fovX[static_cast<int>(CameraId::headBottom)]),
    (float, Intrinsic.fovY, fovY[static_cast<int>(CameraId::headBottom)]),
  );
  fovX[0] *= M_PI / 180;
  fovY[0] *= M_PI / 180;
  fovX[1] *= M_PI / 180;
  fovY[1] *= M_PI / 180;
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    unitVecY[i][0] = 0;
    if (i == static_cast<int>(CameraId::headTop))
      unitVecY[i][1] = -sin(fovY[i] / 2);
    else
      unitVecY[i][1] = sin(fovY[i] / 2);
    unitVecY[i][2] = cos(fovY[i] / 2);
  }
  maxDistGround = 10.0;
}

void ParticleFilter::updateRobotStateEstimate()
{
  //! If we know the robot is on side lines
  if (ON_SIDE_LINE_IN(LocalizationModule)) {
    //! If we have over 20 state estimates, initiate the particle filter
    if (estimatedStates.size() >= 20) {
      RobotPose2D<float> avgEstimatedState(0.f, 0.f, 0.f);
      for (size_t i = 0; i < estimatedStates.size(); ++i) {
        avgEstimatedState += estimatedStates[i];
      }
      avgEstimatedState /= 20.0;
      avgEstimatedState.y() =
        abs(avgEstimatedState.y() + HALF_FIELD_WIDTH) < abs(avgEstimatedState.y() - 0.3) ?
        -HALF_FIELD_WIDTH : HALF_FIELD_WIDTH;
      avgEstimatedState.theta() =
        abs(avgEstimatedState.theta() + M_PI_2) < abs(avgEstimatedState.theta() - M_PI_2) ?
        -M_PI_2 : M_PI_2;
      estimatedStates.clear();
      //! 10 while in game - 20 while in starting pose determination
      estimatedStates.set_capacity(10);
      //! We know the robot is in our half
      lastKnownHalf = 0;
      init(avgEstimatedState);
    } else {
      //! Estimate the robot position based on known observed landmarks such as
      //! Goal, Corners, and Circle keeping mind the robot is on the side lines
      estimateForSideLines();
    }
  } else {
    if (estimatedStates.size() >= 10) {
      init(estimatedStates);
    } else {
      //! Estimate the robot position based on known observed landmarks such as
      //! Goal, Corners, and Circle
      //cout << "estimating from landmarks..." << endl;
      estimateFromLandmarks();
    }
  }
}

void ParticleFilter::estimateForSideLines()
{
  //! Determine the state of the robot and initiate the particle filter
  auto goalInfo = GOAL_INFO_IN(LocalizationModule);
  //! Get an estimate using goal information
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found) {
      if (goalInfo.type == GoalPostType::ourTeam) {
        if (goalInfo.leftPost.y == goalInfo.leftPost.y) {
          if (goalInfo.leftPost.y < 0) {
            auto state =
              RobotPose2D<float>(-HALF_FIELD_HEIGHT - goalInfo.leftPost.y, HALF_FIELD_WIDTH, -M_PI_2);
            estimatedStates.push_back(state);
          } else {
            auto state =
              RobotPose2D<float>(-HALF_FIELD_HEIGHT + goalInfo.leftPost.y, -HALF_FIELD_WIDTH, M_PI_2);
            estimatedStates.push_back(state);
          }
        }

        if (goalInfo.rightPost.y == goalInfo.rightPost.y) {
          if (goalInfo.rightPost.y < 0) {
            auto state =
              RobotPose2D<float>(-HALF_FIELD_HEIGHT - goalInfo.rightPost.y, HALF_FIELD_WIDTH, -M_PI_2);
            estimatedStates.push_back(state);
          } else {
            auto state =
              RobotPose2D<float>(-HALF_FIELD_HEIGHT + goalInfo.rightPost.y, -HALF_FIELD_WIDTH, M_PI_2);
            estimatedStates.push_back(state);
          }
        }
      } else if (goalInfo.type == GoalPostType::opponentTeam) {
        if (goalInfo.leftPost.y == goalInfo.leftPost.y) {
          if (goalInfo.leftPost.y < 0) {
            auto state =
              RobotPose2D<float>(goalInfo.leftPost.y + HALF_FIELD_HEIGHT, -HALF_FIELD_WIDTH, M_PI_2);
            estimatedStates.push_back(state);
          } else {
            auto state = RobotPose2D<float>(goalInfo.leftPost.y - HALF_FIELD_HEIGHT, HALF_FIELD_WIDTH, -M_PI_2);
            estimatedStates.push_back(state);
          }
        }

        if (goalInfo.rightPost.y == goalInfo.rightPost.y) {
          if (goalInfo.rightPost.y < 0) {
            auto state = RobotPose2D<float>(goalInfo.leftPost.y + HALF_FIELD_HEIGHT, -HALF_FIELD_WIDTH, M_PI_2);
            estimatedStates.push_back(state);
          } else {
            auto state = RobotPose2D<float>(goalInfo.rightPost.y - HALF_FIELD_HEIGHT, HALF_FIELD_WIDTH, -M_PI_2);
            estimatedStates.push_back(state);
          }
        }
      }
    }
  }
  prevGoalInfoId = goalInfo.id;

  //! Get an estimate using line landmarks information
  auto landmarks = knownLandmarks;
  if (!landmarks.empty()) {
    /*Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0,0,0));*/
    for (size_t i = 0; i < landmarks.size(); ++i) {
      KnownLandmarkPtr l = landmarks[i];
      auto rPose = l->poseFromLandmark;
      unsigned lCount = landmarkTypeCount[l->type];
      unsigned lStart = landmarkTypeStarts[l->type];
      //cout << "Type: " << l->type << endl;
      //cout << "poseX: " << rPose.x << endl;
      //cout << "poseY: " << rPose.y << endl;
      //cout << "poseTheta: " << rPose.theta * 180 / M_PI << endl;
      for (size_t j = 0; j < lCount; ++j) {
        auto lPose = landmarksOnField[j + lStart];
        //cout << "lX: " << lPose.x << endl;
        //cout << "lY: " << lPose.y << endl;
        //cout << "lTheta: " << lPose.theta * 180 / M_PI << endl;
        RobotPose2D<float> wPose;
        wPose.x() = lPose.getX() + rPose.getX() * cos(lPose.getTheta()) - rPose.getY() * sin(lPose.getTheta());
        wPose.y() = lPose.getY() + rPose.getX() * sin(lPose.getTheta()) + rPose.getY() * cos(lPose.getTheta());
        wPose.theta() = MathsUtils::rangeToPi(rPose.getTheta() + lPose.getTheta());
        // Should be in our own half which is in -ve x and not too close to the goal line.
        // Should be either closer to upper line or lower line which is 3 
        // meters from center
        //float rangeT = MathsUtils::rangeToPi(wPose.theta);
        bool positive = false;
        if (wPose.x() > -4.25 && wPose.x() < 0.f) {
          if (wPose.y() > 2.5f) {
            if (wPose.theta() > -1.7444 && wPose.theta() < -1.39556) {
              positive = true;
            }
          } else if (wPose.y() < -2.5f) {
            if (wPose.theta() > 1.39556 && wPose.theta() < 1.7444) {
              positive = true;
            }
          }
        }
        //cout << "j; " << j << endl;
        //cout << "wX: " << wPose.x << endl;
        //cout << "wY: " << wPose.y << endl;
        //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
        if (positive) {
          estimatedStates.push_back(wPose);
        } else {
          continue;
        }
        //drawParticle(wPose, worldImage);
        //VisionUtils::displayImage("worldImage", worldImage);
        //waitKey(0);
      }
    }
  }
}

void ParticleFilter::estimateFromLandmarks()
{
  //Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0, 0, 0));
  auto& lastPose = LAST_POSE_2D_OUT(LocalizationModule);
  auto useLastEstimate = LOCALIZE_LAST_KNOWN_IN(LocalizationModule);
  if (!knownLandmarks.empty()) {
    float worldLimitX = HALF_FIELD_HEIGHT + 0.25;
    float worldLimitY = HALF_FIELD_WIDTH + 0.25;
    for (const auto& l : knownLandmarks) {
      auto rPose = l->poseFromLandmark;
      unsigned lCount = landmarkTypeCount[l->type];
      unsigned lStart = landmarkTypeStarts[l->type];
      //cout << "Type: " << l->type << endl;
      //cout << "poseX: " << rPose.x << endl;
      //cout << "poseY: " << rPose.y << endl;
      //cout << "poseTheta: " << rPose.theta * 180 / M_PI << endl;
      for (size_t j = 0; j < lCount; ++j) {
        auto lPose = landmarksOnField[j + lStart];
        //cout << "lX: " << lPose.x << endl;
        //cout << "lY: " << lPose.y << endl;
        //cout << "lTheta: " << lPose.theta * 180 / M_PI << endl;
        RobotPose2D<float> wPose = landmarkPoseToWorldPose(lPose, rPose);
        if (useLastEstimate && lastPose.getX() == lastPose.getX()) {
          //! We know the last known pose of the robot so we find estimates
          //! that are close to it
          if (
              wPose.x() > -worldLimitX &&
              wPose.x() < worldLimitX &&
              wPose.y() > -worldLimitY &&
              wPose.y() < worldLimitY)
          {
            float d = (wPose.get() - lastPose.get()).segment(0, 2).norm();
            if (d < 1.f) { // Within 25 cm radius
              estimatedStates.push_back(wPose);
              //drawParticle(wPose, worldImage);
            }
          }
        } else {
          //! We add all the poses that lie within last known half
          if (lastKnownHalf == 0) {
            if (
                wPose.x() > -worldLimitX &&
                wPose.x() < 0.f &&
                wPose.y() > -worldLimitY &&
                wPose.y() < worldLimitY)
            {
              estimatedStates.push_back(wPose);
              //drawParticle(wPose, worldImage);
              //cout << "wX: " << wPose.x << endl;
              //cout << "wY: " << wPose.y << endl;
              //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
            }
          } else {
            if (
                wPose.x() > 0.f &&
                wPose.x() < worldLimitX &&
                wPose.y() > -worldLimitY &&
                wPose.y() < worldLimitY)
            {
              estimatedStates.push_back(wPose);
              //cout << "wX: " << wPose.x << endl;
              //cout << "wY: " << wPose.y << endl;
              //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
            }
          }
        }
      }
    }
  }
  auto goalInfo = GOAL_INFO_IN(LocalizationModule);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found) {
      auto rPose = goalInfo.poseFromGoal;
      RobotPose2D<float> wPose;
      if (goalInfo.type == GoalPostType::ourTeam) {
        RobotPose2D<float> lPose = landmarksOnField[FL_LB_GOALPOST];
        lPose.y() = 0.f;
        //wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
        //  lPose.theta);
        //wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
        //  lPose.theta);
        //wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);
        wPose = landmarkPoseToWorldPose(lPose, rPose);
      } else if (goalInfo.type == GoalPostType::opponentTeam) {
        RobotPose2D<float> lPose = landmarksOnField[FL_LT_GOALPOST];
        lPose.y() = 0.f;
        wPose = landmarkPoseToWorldPose(lPose, rPose);
        /*
        wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
          lPose.theta);
        wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
          lPose.theta);
        wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);*/
      } else {
        RobotPose2D<float> lPose1 = landmarksOnField[FL_LB_GOALPOST];
        lPose1.y() = 0.f;
        RobotPose2D<float> lPose2 = landmarksOnField[FL_LT_GOALPOST];
        lPose2.y() = 0.f;
        RobotPose2D<float> wPose1, wPose2;
        wPose1 = landmarkPoseToWorldPose(lPose1, rPose);
        wPose2 = landmarkPoseToWorldPose(lPose2, rPose);
        /*wPose1.x = lPose1.x + rPose.x * cos(lPose1.theta) - rPose.y * sin(
          lPose1.theta);
        wPose1.y = lPose1.y + rPose.x * sin(lPose1.theta) + rPose.y * cos(
          lPose1.theta);
        wPose1.theta = rPose.theta + lPose1.theta;
        wPose2.x = lPose2.x + rPose.x * cos(lPose2.theta) - rPose.y * sin(
          lPose2.theta);
        wPose2.y = lPose2.y + rPose.x * sin(lPose2.theta) + rPose.y * cos(
          lPose2.theta);
        wPose2.theta = MathsUtils::rangeToPi(rPose.theta + lPose2.theta);*/
        if (lastKnownHalf == 0) {
          if (wPose1.x() < 0.f) wPose = wPose1;
          else if (wPose2.x() < 0.f) wPose = wPose2;
        } else {
          if (wPose1.x() > 0.f) wPose = wPose1;
          else if (wPose2.x() > 0.f) wPose = wPose2;
        }
      }
      if (wPose.x() > -4.75f && wPose.x() < 4.75f && wPose.y() > -3.25 && wPose.y() < 3.25) {
        if (useLastEstimate && lastPose.getX() == lastPose.getX()) {
          float d = (wPose.get() - lastPose.get()).segment(0, 2).norm();
          if (d < 1.f) { // Within 25 cm radius
            estimatedStates.push_back(wPose);
          }
        } else {
          estimatedStates.push_back(wPose);
        }
      }
      //drawParticle(wPose, worldImage);
    }
  }
  prevGoalInfoId = goalInfo.id;
}

void ParticleFilter::minMaxGroundDist()
{
  vector<Matrix4f> T(toUType(CameraId::count));
  vector<Vector3f> unitVecYT(toUType(CameraId::count));
  vector<float> slopes(toUType(CameraId::count));
  T[static_cast<int>(CameraId::headTop)] = UPPER_CAM_TRANS_IN(LocalizationModule);
  T[static_cast<int>(CameraId::headBottom)] = LOWER_CAM_TRANS_IN(LocalizationModule);
  for (size_t i = 0; i < toUType(CameraId::count); ++i) {
    unitVecYT[i] = T[i].block(0, 0, 3, 3) * unitVecY[i];
    slopes[i] = unitVecYT[i][0] / unitVecYT[i][2];
  }
  minDistGround =
    slopes[static_cast<int>(CameraId::headBottom)] *
      (0.0 - T[static_cast<int>(CameraId::headBottom)](2, 3)) +
      T[static_cast<int>(CameraId::headBottom)](0, 3);
}

void ParticleFilter::prediction()
{
  if(!positionInputs.empty()) {
    while (!positionInputs.empty()) {
      auto& input = positionInputs.front();
      NormalDistribution<double> dist;
      for (size_t i = 0; i < nParticles; ++i) {
        auto ct = cos(particles[i].state.getTheta());
        auto st = sin(particles[i].state.getTheta());
        particles[i].state.x() = particles[i].state.getX() + input.getX() * ct - input.getY() * st;
        particles[i].state.y() = particles[i].state.getY() + input.getX() * st + input.getY() * ct;
        particles[i].state.theta() = particles[i].state.getTheta() + input.getTheta();
        for (size_t j = 0; j < particles[i].state.get().size(); ++j) {
          particles[i].state.get()[j] =
            dist(random, particles[i].state.get()[j], motionStd[j]);
        }
      }
      positionInputs.pop();
    }
  }// else {
   // addPredictionNoise();
  //}
}

void ParticleFilter::prediction(const VelocityInput<double>& vI)
{
  NormalDistribution<double> dist;
  for (size_t i = 0; i < nParticles; ++i) {
    Particle p = particles[i];
    double thetaF = p.state.getTheta() + cycleTime * vI.getTheta();
    double velMag = vI.get().segment(0, 2).norm();
    double velAngle = atan2(vI.getY(), vI.getX());
    double deltaX;
    double deltaY;
    if (vI.getTheta() != 0.0) {
      double arcRadius = velMag / vI.getTheta();
      deltaX =
        arcRadius * (sin(thetaF + velAngle) - sin(p.state.getTheta() + velAngle));
      deltaY =
        arcRadius * (-cos(thetaF + velAngle) + cos(p.state.getTheta() + velAngle));
    } else {
      deltaX = cycleTime * vI.getX();
      deltaY = cycleTime * vI.getY();
    }
    p.state.x() += deltaX;
    p.state.y() += deltaY;
    p.state.theta() = thetaF;
    for (size_t j = 0; j < particles[i].state.get().size(); ++j) {
      particles[i].state.get()[j] =
        dist(random, p.state.get()[j], predictionStd[j]);
    }
  }
}

void ParticleFilter::addPredictionNoise()
{
  NormalDistribution<double> dist;
  for (size_t i = 0; i < nParticles; ++i) {
    for (size_t j = 0; j < particles[i].state.get().size(); ++j) {
      particles[i].state.get()[j] =
        dist(random, particles[i].state.get()[j], predictionStd[j]);
    }
  }
}

void ParticleFilter::updateWeights(const vector<LandmarkPtr>& obsLandmarks)
{
  avgFilteredState = RobotPose2D<float>(0.f, 0.f, 0.f);
  sumWeights = 0.0;
  maxWeight = 0.0;
  Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0,0,0));
  if (!obsLandmarks.empty()) {
    for (size_t i = 0; i < nParticles; ++i) {
      Particle& p = particles[i];
      double& w = particles[i].weight;
      vector<bool> lMarked(fieldLandmarks.size(), false);
      for (size_t j = 0; j < fieldLandmarks.size(); ++j) {
       auto ll = fieldLandmarks[j];
       VisionUtils::drawPoint(Point(ll->pos.x * 100 + 500, 350 - 100 * ll->pos.y), worldImage, Scalar(0,0,255));
      }
      for (size_t j = 0; j < obsLandmarks.size(); ++j) {
        bool badObs = false;
        auto obs = obsLandmarks[j];
        auto ct = cos(p.state.theta());
        auto st = sin(p.state.theta());
        Point2f tPos;
        tPos.x = p.state.x() + obs->pos.x * ct - obs->pos.y * st;
        tPos.y = p.state.y() + obs->pos.x * st + obs->pos.y * ct;
        if (abs(tPos.x) > 4.75 || abs(tPos.y) > 3.25) {
          badObs = true;
          //++badObs;
          //continue;
        }
        VisionUtils::drawPoint(Point(tPos.x * 100 + 500, 350 - 100 * tPos.y), worldImage, Scalar(255,0,0));
        if (!badObs) {
          //cout << "T Pos: " << Point(tPos.x * 100 + 500, 350 - 100 * tPos.y) << endl;
          //imshow("olImage", olImage);
          //waitKey(0);
          int index = -1;

          auto tPosWorld = worldToVoronoiMap(tPos);
          unsigned label =
            voronoiLabels.at<uint>(tPosWorld.y, tPosWorld.x);
          double minDist = 1000;
          //if (!labelledLandmarks[label-1].empty()) {
          //  index = labelledLandmarks[label-1][0]->id;
          //  minDist = norm(labelledLandmarks[label-1][0]->pos - tPos);
          //}
          //cout << "label: " << label << endl;
          //cout << "labelledLandmarks[label-1].size(): " << labelledLandmarks[label-1].size() << endl;
          //cout << "fieldLandmarks.size(): " << fieldLandmarks.size() << endl;
          for (unsigned k = 0; k < labelledLandmarks[label - 1].size(); ++k) {
            auto ll = labelledLandmarks[label - 1][k];
            //cout << "ll->id:"<< ll->id << endl;
            if (!lMarked[ll->id]) {
              if (obs->type == ll->type) {
                double d = norm(ll->pos - tPos);
                if (d < minDist) {
                  minDist = d;
                  index = ll->id;
                }
              }
            }
          }
          if (minDist < 0.25) {
            auto ll = fieldLandmarks[index];
            double dx = (tPos.x - ll->pos.x);
            dx *= dx;
            double dy = (tPos.y - ll->pos.y);
            dy *= dy;
            w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
            //cout << "w: " << gaussianConst * exp(dx*expConstX + dy*expConstY) << endl;
            VisionUtils::drawPoint(Point(ll->pos.x * 100 + 500, 350 - 100 * ll->pos.y), worldImage, Scalar(0,255,0));
            lMarked[index] = true;
          } else {
            double dx = 0.25 / sqrt(2);
            dx *= dx;
            double dy = 0.25 / sqrt(2);
            dy *= dy;
            w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
          }
        } else {
          double dx = 0.25 / sqrt(2);
          dx *= dx;
          double dy = 0.25 / sqrt(2);
          dy *= dy;
          w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
        }
      }
      particles[i].weight = w;
      sumWeights += w;
      maxWeight = w > maxWeight ? w : maxWeight;
      avgFilteredState = avgFilteredState + particles[i].state;
      //cout << "i; " << i << endl;
      //cout << "maxWeight: " << maxWeight << endl;
      //cout << "w: " << w << endl;
      //VisionUtils::displayImage("worldImage", worldImage, 0.5);
      //imshow("olImage", olImage);
      //waitKey(0);
    }
  } else {
    for (size_t i = 0; i < nParticles; ++i)
      avgFilteredState = avgFilteredState + particles[i].state;
  }
  avgFilteredState /= nParticles;
  double avgWeights = sumWeights / nParticles;
  wSlow += ALPHA_SLOW * (avgWeights - wSlow);
  wFast += ALPHA_FAST * (avgWeights - wFast);
}

void ParticleFilter::normalizeWeights(const bool& noData)
{
  //! If no data or the highest weight is close to zero
  if (noData) {
    for (size_t i = 0; i < nParticles; ++i)
      particles[i].weight = 1.0 / nParticles;
  } else {
    if (maxWeight < 1e-9) {
      lostCount++;
      if (lostCount > MAX_LOST_COUNT) {
        reset();
        lostCount = 0;
      } else {
        for (size_t i = 0; i < nParticles; ++i)
          particles[i].weight = 1.0 / nParticles;
      }
    } else {
      for (size_t i = 0; i < nParticles; ++i)
        particles[i].weight = particles[i].weight / sumWeights;
      lostCount = 0;
    }
  }
}

void ParticleFilter::resample()
{
  if (!localized)
    return;
  vector<Particle> resampled;
  double rNumber = random.FloatU() * (1.0 / nParticles);
  double w = particles[0].weight;
  int j = 0;
  double vl = 0;
  double threshold = 1.0 - (wFast / wSlow);
  if (threshold > vl) vl = threshold;
  for (size_t i = 0; i < nParticles; ++i) {
    /*if (random.FloatU() < vl) {
     NormalDistribution<double> distX;
     NormalDistribution<double> distY;
     NormalDistribution<double> distTheta;
     Particle p;
     p.state.x = distX(random, avgFilteredState.x, 0.25 / 3);
     p.state.y = distY(random, avgFilteredState.y, 0.25 / 3);
     p.state.theta = distTheta(random, avgFilteredState.theta, 10 * M_PI / 180 / 3);
     p.weight = 1.0 / nParticles;
     resampled.push_back(p);
     } else {*/
    double u = rNumber + i / (double) nParticles;
    while (u > w) {
      j++;
      w += particles[j].weight;
    }
    resampled.push_back(particles[j]);
    //}
  }
  /*RandomSelect<double> distr(weights.begin(), weights.end());
   for (size_t i = 0; i < nParticles; ++i) {
   if(particles[i].weight > 0.3) {
   resampled[i] = particles[i];
   } else {
   int index = distr(random);
   resampled[i] = particles[index];
   }
   resampled[i].weight = 1.0 / nParticles;
   }*/
  for (size_t i = 0; i < nParticles; ++i) {
    resampled[i].weight = 1.0 / nParticles;
  }
  particles = resampled;
}

void ParticleFilter::updatePositionConfidence()
{
  LAST_POSE_2D_OUT(LocalizationModule) = avgFilteredState;
    /*float highestNormWeight = 0.f;
    cout << "wieghts: " << endl;
    for (size_t i = 0; i < nParticles; ++i) {
      cout << particles[i].weight << endl;
      highestNormWeight = particles[i].weight > highestNormWeight ? particles[i].weight : highestNormWeight;
    }
    highestNormWeight /= nParticles;
    cout << "highestNormWeight: " << highestNormWeight << endl;
    POSITION_CONFIDENCE_OUT(LocalizationModule) = highestNormWeight * 100;
    cout << "position confidence: " << POSITION_CONFIDENCE_OUT(LocalizationModule) << endl;
    if (POSITION_CONFIDENCE_OUT(LocalizationModule) < 25) {
      lostCount++;
      if (lostCount > MAX_LOST_COUNT) {
        reset();
      } else {
        for (size_t i = 0; i < nParticles; ++i) {
          particles[i].weight = 1.0 / nParticles;
        }
      }
    } else {
      lostCount = 0;
    }*/
  double std = 0.0;
  //cout << "avgFilteredState: " <<avgFilteredState.get().transpose() << endl;
  for (size_t i = 0; i < nParticles; ++i) {
    std += (particles[i].state.get() - avgFilteredState.get()).segment(0, 2).norm();
    //cout << "i: " << particles[i].state.get().transpose() << endl;
  }
  std = sqrt(std / nParticles);
  auto& positionConfidence = POSITION_CONFIDENCE_OUT(LocalizationModule);
  //cout << "std: " << std << endl;
  //cout << "positionConfidence: " << positionConfidence << endl;
  if (std < 0.5) {
    positionConfidence /= 0.9;
    positionConfidence = positionConfidence >= 100 ? 100 : positionConfidence;
  } else {
    positionConfidence *= 0.9;
  }

  if (POSITION_CONFIDENCE_OUT(LocalizationModule) < 25) {
    localized = false;
    reset();
  } else {
    localized = true;
  }
}

void ParticleFilter::updateSideConfidence()
{
  if (!localized)
    return;
  auto goalInfo = GOAL_INFO_IN(LocalizationModule);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found && goalInfo.type != GoalPostType::unknown) {
      if (goalInfo.mid == goalInfo.mid) { // NAN
        Point2f goalInMap;
        auto ct = cos(avgFilteredState.theta());
        auto st = sin(avgFilteredState.theta());
        goalInMap.x =
          avgFilteredState.x() + goalInfo.mid.x * ct - goalInfo.mid.y * st;
        goalInMap.y =
          avgFilteredState.y() + goalInfo.mid.x * st + goalInfo.mid.y * ct;
        float distOurs = norm(goalInMap - Point2f(-HALF_FIELD_HEIGHT, 0));
        float distOpp = norm(goalInMap - Point2f(HALF_FIELD_HEIGHT, 0));
        auto& sideConfidence = SIDE_CONFIDENCE_OUT(LocalizationModule);
        //cout << "goalInMap: " << goalInMap << endl;
        //cout << "distOurs: " << distOurs << endl;
        //cout << "distOpp: " << distOpp << endl;
        if (distOurs < distOpp) {
          if (goalInfo.type == GoalPostType::opponentTeam) {
            // If seen goal post is closer to our goal post
            // but classified as opponents goalpost
            // Decrease the side confidence
            sideConfidence *= 0.9;
          } else if (goalInfo.type == GoalPostType::ourTeam) {
            sideConfidence /= 0.9;
            sideConfidence = sideConfidence >= 100 ? 100 : sideConfidence;
          }
        } else {
          if (goalInfo.type == GoalPostType::ourTeam) {
            // If seen goal post is closer to our opponent's post
            // but classified as our goalpost
            // Decrease the side confidence
            sideConfidence *= 0.9;
          } else if (goalInfo.type == GoalPostType::opponentTeam) {
            sideConfidence /= 0.9;
            sideConfidence = sideConfidence >= 100 ? 100 : sideConfidence;
          }
        }
        SIDE_CONFIDENCE_OUT(LocalizationModule) = sideConfidence;
      }
    }
  }
  prevGoalInfoId = goalInfo.id;

  if (SIDE_CONFIDENCE_OUT(LocalizationModule) <= 30) {
    // Mirror all particles
    for (size_t i = 0; i < nParticles; ++i) {
      particles[i].state.x() = -particles[i].state.x();
      particles[i].state.y() = -particles[i].state.y();
      particles[i].state.theta() = particles[i].state.theta() + M_PI;
    }
    SIDE_CONFIDENCE_OUT(LocalizationModule) = 70;
  } else {
    lastKnownHalf = avgFilteredState.getX() > 0 ? 1 : 0;
  }
}

void ParticleFilter::addPositionInput(const PositionInput<float>& input)
{
  this->positionInputs.push(input);
}

void ParticleFilter::setKnownLandmarks(const vector<KnownLandmarkPtr>& landmarks)
{
  this->knownLandmarks = landmarks;
}

void ParticleFilter::setUnknownLandmarks(const vector<UnknownLandmarkPtr>& landmarks)
{
  this->unknownLandmarks = landmarks;
}

RobotPose2D<float> ParticleFilter::landmarkPoseToWorldPose(
  const RobotPose2D<float> lPose, const RobotPose2D<float> rPose)
{
  RobotPose2D<float> wPose;
  wPose.x() = lPose.getX() + rPose.getX() * cos(lPose.getTheta()) - rPose.getY() * sin(lPose.getTheta());
  wPose.y() = lPose.getY() + rPose.getX() * sin(lPose.getTheta()) + rPose.getY() * cos(lPose.getTheta());
  wPose.theta() = MathsUtils::rangeToPi(rPose.getTheta() + lPose.getTheta());
  return wPose;
}

void ParticleFilter::drawParticle(const Particle& p, Mat& image)
{
  drawParticle(p.state, image);
}

void ParticleFilter::drawParticle(const RobotPose2D<float>& p, Mat& image)
{
  if (!image.empty()) {
    circle(image, worldToVoronoiMap(Point(p.getX(), p.getY())), 10, Scalar(255, 0, 0));
    line(
      image,
      worldToVoronoiMap(Point(p.getX(), p.getY())),
      worldToVoronoiMap(Point(p.getX() + 0.15 * cos(p.getTheta()), - 0.15 * sin(p.getTheta()))),
      Scalar(255, 0, 0)
    );
  }
}
