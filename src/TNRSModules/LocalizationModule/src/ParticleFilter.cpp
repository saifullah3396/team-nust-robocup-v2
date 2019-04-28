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
#define MAX_LOST_COUNT 100
#define ALPHA_SLOW .01 //0.05
#define ALPHA_FAST .1

ParticleFilter::ParticleFilter(LocalizationModule* lModule) :
  MemoryBase(lModule),
  DebugBase("ParticleFilter", this),
  lModule(lModule),
  cycleTime(lModule->getPeriodMinMS() / (1000.f)),
  landmarkTypeCount(NUM_LANDMARK_TYPES),
  landmarkTypeStarts(NUM_LANDMARK_TYPES)
{
  initDebugBase();
  GET_DEBUG_CONFIG(
    LocalizationConfig, ParticleFilter,
    (int, displayLandmarksMap),
    (int, displayVoronoiMap),
  );

  GET_CLASS_CONFIG(
    LocalizationConfig, ParticleFilter,
    nParticles,
    genStd,
    motionStd,
    predictionStd,
    measurementStd,
    lGridResolution,
  );

  gaussianConst = 0.5 / (M_PI * measurementStd[0] * measurementStd[1]);
  expConstX = -0.5 / (measurementStd[0] * measurementStd[0]);
  expConstY = -0.5 / (measurementStd[1] * measurementStd[1]);
  estimatedStates.set_capacity(nParticles / 2);
  particles.resize(nParticles);
  setupLandmarks();
  setupVoronoiMap();
  reset();
}

void ParticleFilter::init(const RobotPose2D<float>& state)
{
  NormalDistribution<double> dist;
  auto w = 1.0 / (float) nParticles;
  for (size_t i = 0; i < nParticles; ++i) {
    auto& p = particles[i];
    p.x() = dist(random, state.getX(), genStd[0]);
    p.y() = dist(random, state.getY(), genStd[1]);
    p.theta() = dist(random, state.getTheta(), genStd[2]);
    p.ct = cos(p.theta());
    p.st = sin(p.theta());
    p.weight = w;
  }
  avgState = state;
  avgState.ct = cos(avgState.theta());
  avgState.st = sin(avgState.theta());
  bestParticle = nullptr;
  wSlow = 0.0, wFast = 0.0;
  SIDE_CONFIDENCE_OUT(LocalizationModule) = 100;
  POSITION_CONFIDENCE_OUT(LocalizationModule) = 100;
  initiated = true;
}

void ParticleFilter::init(const boost::circular_buffer<RobotPose2D<float> >& states)
{
  //Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0,0,0));
  avgState.x() = 0.f;
  avgState.y() = 0.f;
  avgState.theta() = 0.f;
  auto w = 1.0 / (float) nParticles;
  int divider = nParticles / states.size();
  for (size_t i = 0; i < nParticles; ++i) {
    auto& p = particles[i];
    p = states[i / divider];
    p.weight = w;
    avgState.x() += p.getX();
    avgState.y() += p.getY();
    avgState.theta() += (p.getTheta() < 0 ? MathsUtils::M_TWICE_PI + p.getTheta() : p.getTheta());
    //drawParticle(p, worldImage);
  }
  //VisionUtils::displayImage("Particles init", worldImage, 0.25);
  //waitKey(0);
  avgState /= (float)nParticles;
  avgState.theta() = MathsUtils::rangeToPi(avgState.getTheta());
  avgState.ct = cos(avgState.theta());
  avgState.st = sin(avgState.theta());
  bestParticle = nullptr;

  wSlow = 0.0, wFast = 0.0;
  SIDE_CONFIDENCE_OUT(LocalizationModule) = 100;
  POSITION_CONFIDENCE_OUT(LocalizationModule) = 100;
  initiated = true;
}

void ParticleFilter::update()
{
  //auto worldImage = Mat(vMapSize, CV_8UC3, Scalar(0,0,0));
  //cout << "initiated: " << initiated << endl;
  //cout << "localized: " << localized << endl;
  if (!initiated) {
    ///< If the particle filter is not initiated and we do not have any info
    ///< on the robot position, try to get an estimate using observed landmarks info
    updateRobotStateEstimate();
  } else {
    ///< Keep updating the estimate for correction
    /*if (ON_SIDE_LINE_IN(LocalizationModule)) {
      estimateForSideLines();
    } else {
      estimateFromLandmarks();
    }*/
    vector<LandmarkPtr> obsLandmarks;
    obsLandmarks.insert(obsLandmarks.begin(), knownLandmarks.begin(), knownLandmarks.end());
    obsLandmarks.insert(obsLandmarks.begin(), unknownLandmarks.begin(), unknownLandmarks.end());
    prediction();
    cout << "obsLandmarks.size() : " << obsLandmarks.size() << endl;
    cout << "knownLandmarks.size() : " << knownLandmarks.size() << endl;
    cout << "unknownLandmarks.size() : " << unknownLandmarks.size() << endl;
    static auto minObsLandmarksRequired = 10;
    if (obsLandmarks.size() > minObsLandmarksRequired) {// && !ROBOT_IN_MOTION_IN(LocalizationModule)) { // Movement results in a lot of bad candidates
      updateWeights(obsLandmarks);
      normalizeWeights(false);
      if (!ROBOT_IN_MOTION_IN(LocalizationModule)) {
        addPredictionNoise();
      }
    } else {
      updateWeights (vector<LandmarkPtr>());
      normalizeWeights(true);
    }
    if (initiated) {
      resample();
      updateSideConfidence();
      updatePositionConfidence();
    }
  }
  /*for (auto& p : estimatedStates) {
    drawParticle(p, worldImage, Scalar(0,255,0));
  }
  for (auto& p : particles) {
    drawParticle(p, worldImage, Scalar(0,0,255));
  }
  if (bestParticle)
    drawParticle(*bestParticle, worldImage, Scalar(255,0,0));*/
  //VisionUtils::displayImage("particles", worldImage, 0.25);
  //waitKey(0);
  knownLandmarks.clear();
  unknownLandmarks.clear();
}

void ParticleFilter::reset()
{
  cout << "Resetting filter..." << endl;
  particles.clear();
  estimatedStates.clear();
  initiated = false;
  localized = false;
  lostCount = 0;
  bestParticle = nullptr;
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

void ParticleFilter::updateRobotStateEstimate()
{
  ///< If we know the robot is on side lines
  if (ON_SIDE_LINE_IN(LocalizationModule)) {
    ///< If we have over nParticles state estimates, initiate the particle filter
    if (estimatedStates.size() >= nParticles / 2.0) {
      static auto sideLineTolerance = 0.3; //! 30 cm
      for (auto& state : estimatedStates) {
        state.y() =
          abs(state.y() + HALF_FIELD_WIDTH) < abs(state.y() - sideLineTolerance) ?
          -HALF_FIELD_WIDTH : HALF_FIELD_WIDTH;
      }
      ///< We know the robot is in our half
      lastKnownHalf = FieldHalf::teamHalf;
      cout << "estimatedStates: " << estimatedStates.size() << endl;
      init(estimatedStates);
      estimatedStates.clear();
    } else {
      ///< Estimate the robot position based on known observed landmarks such as
      ///< Goal, Corners, and Circle keeping mind the robot is on the side lines
      estimateForSideLines();
    }
  } else {
    cout << "estimatedStates: " << estimatedStates.size() << endl;
    if (estimatedStates.size() >= nParticles / 2.0) {
      init(estimatedStates);
    } else {
      ///< Estimate the robot position based on known observed landmarks such as
      ///< Goal, Corners, and Circle
      estimateFromLandmarks();
    }
  }
}

void ParticleFilter::estimateForSideLines()
{
  ///< Determine the state of the robot and initiate the particle filter
  const auto& goalInfo = GOAL_INFO_IN(LocalizationModule);
  ///< Get an estimate using goal information
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

  ///< Get an estimate using line landmarks information
  auto landmarks = knownLandmarks;
  if (!landmarks.empty()) {
    //Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0,0,0));
    for (const auto& l : landmarks) {
      unsigned lCount = landmarkTypeCount[l->type];
      unsigned lStart = landmarkTypeStarts[l->type];
      //cout << "Type: " << l->type << endl;
      //cout << "poseX: " << rPose.x << endl;
      //cout << "poseY: " << rPose.y << endl;
      //cout << "poseTheta: " << rPose.theta * 180 / M_PI << endl;
      for (size_t j = 0; j < lCount; ++j) {
        auto& landmarkPose = landmarksOnField[j + lStart];
        //cout << "lX: " << lPose.x << endl;
        //cout << "lY: " << lPose.y << endl;
        //cout << "lTheta: " << lPose.theta * 180 / M_PI << endl;
        auto wPose = landmarkPose.transform(l->poseFromLandmark);
        // Should be in our own half which is in -ve x and not too close to the goal line.
        // Should be either closer to upper line or lower line which is 3
        // meters from center
        //float rangeT = MathsUtils::rangeToPi(wPose.theta);
        bool positive = false;
        if (wPose.getX() > -4.25 && wPose.getX() < 0.f) {
          if (wPose.getY() > 2.5f) {
            if (wPose.getTheta() > -1.7444 && wPose.getTheta() < -1.39556) {
              positive = true;
            }
          } else if (wPose.getY() < -2.5f) {
            if (wPose.getTheta() > 1.39556 && wPose.getTheta() < 1.7444) {
              positive = true;
            }
          }
        }
        //cout << "j; " << j << endl;
        //cout << "wX: " << wPose.x << endl;
        //cout << "wY: " << wPose.y << endl;
        //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
        //drawParticle(wPose, worldImage);
        //VisionUtils::displayImage("worldImage", worldImage);
        //waitKey(0);
        if (positive) {
          estimatedStates.push_back(wPose);
        } else {
          continue;
        }
      }
    }
  }
}

void ParticleFilter::estimateFromLandmarks()
{
  //Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0, 0, 0));
  auto& lastPose = LAST_POSE_2D_OUT(LocalizationModule);
  const auto& useLastEstimate = LOCALIZE_LAST_KNOWN_IN(LocalizationModule);
  static auto lastKnownEstimateTolerance = 1.0; //! 1m
  if (!knownLandmarks.empty()) {
    auto worldLimitX = HALF_FIELD_HEIGHT + 0.25;
    auto worldLimitY = HALF_FIELD_WIDTH + 0.25;
    for (const auto& l : knownLandmarks) {
      auto rPose = l->poseFromLandmark;
      auto lCount = landmarkTypeCount[l->type];
      auto lStart = landmarkTypeStarts[l->type];
      //cout << "Type: " << l->type << endl;
      //cout << "poseX: " << rPose.x << endl;
      //cout << "poseY: " << rPose.y << endl;
      //cout << "poseTheta: " << rPose.theta * 180 / M_PI << endl;
      for (size_t j = 0; j < lCount; ++j) {
        auto lPose = landmarksOnField[j + lStart];
        //cout << "lX: " << lPose.x << endl;
        //cout << "lY: " << lPose.y << endl;
        //cout << "lTheta: " << lPose.theta * 180 / M_PI << endl;
        RobotPose2D<float> wPose = lPose.transform(rPose);
        if (useLastEstimate && lastPose.getX() == lastPose.getX()) {
          ///< We know the last known pose of the robot so we find estimates
          ///< that are close to it
          if (
              wPose.x() > -worldLimitX &&
              wPose.x() < worldLimitX &&
              wPose.y() > -worldLimitY &&
              wPose.y() < worldLimitY)
          {
            auto d = (wPose.get() - lastPose.get()).segment(0, 2).norm();
            if (d < lastKnownEstimateTolerance) { // Within 1.0 radius
              estimatedStates.push_back(wPose);
              //drawParticle(wPose, worldImage);
            }
          }
        } else {
          ///< We add all the poses that lie within last known half
          if (lastKnownHalf == FieldHalf::teamHalf) {
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
  const auto& goalInfo = GOAL_INFO_IN(LocalizationModule);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found) {
      auto rPose = goalInfo.poseFromGoal;
      RobotPose2D<float> wPose;
      if (goalInfo.type == GoalPostType::ourTeam) {
        auto lPose = landmarksOnField[FL_LB_GOALPOST];
        lPose.y() = 0.f;
        //wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
        //  lPose.theta);
        //wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
        //  lPose.theta);
        //wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);
        wPose = lPose.transform(rPose);
      } else if (goalInfo.type == GoalPostType::opponentTeam) {
        auto lPose = landmarksOnField[FL_LT_GOALPOST];
        lPose.y() = 0.f;
        wPose = lPose.transform(rPose);
        /*
        wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
          lPose.theta);
        wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
          lPose.theta);
        wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);*/
      } else {
        auto lPose1 = landmarksOnField[FL_LB_GOALPOST];
        lPose1.y() = 0.f;
        auto lPose2 = landmarksOnField[FL_LT_GOALPOST];
        lPose2.y() = 0.f;
        auto wPose1 = lPose1.transform(rPose);
        auto wPose2 = lPose2.transform(rPose);
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
        if (lastKnownHalf == FieldHalf::teamHalf) {
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
    }
  }
  prevGoalInfoId = goalInfo.id;
}

void ParticleFilter::prediction()
{
  //Mat worldImage = Mat(vMapSize, CV_8UC3, Scalar(0, 0, 0));
  if(!positionInputs.empty()) {
    PositionInput<float> pi;
    while (!positionInputs.empty()) {
      pi += positionInputs.front();
      positionInputs.pop();
    }
    NormalDistribution<double> dist;
    for (auto& p : particles) {
      p = p.transform(pi);
      p.x() = dist(random, p.getX(), motionStd[0]);
      p.y() = dist(random, p.getY(), motionStd[1]);
      p.theta() = dist(random, p.getTheta(), motionStd[2]);
      p.ct = cos(p.theta());
      p.st = sin(p.theta());
      //drawParticle(p, worldImage);
    }
    //VisionUtils::displayImage("prediction", worldImage, 0.5);
  }// else {
   // addPredictionNoise();
  //}
  /*if(!velocityInputs.empty()) {
    while (!velocityInputs.empty()) {
      auto& input = velocityInputs.front();
      NormalDistribution<double> dist;
      for (auto& p : particles) {
        p += input * this->cycleTime;
        p.x() = dist(random, p.getX(), motionStd[0]);
        p.y() = dist(random, p.getY(), motionStd[1]);
        p.theta() = dist(random, p.getTheta(), motionStd[2]);
        p.ct = cos(p.theta());
        p.st = sin(p.theta());
      }
      velocityInputs.pop();
    }
  }*/
}

void ParticleFilter::prediction(const VelocityInput<double>& vI)
{
  NormalDistribution<double> dist;
  for (auto& p : particles) {
    auto thetaF = p.getTheta() + cycleTime * vI.getTheta();
    auto velMag = vI.get().segment(0, 2).norm();
    auto velAngle = atan2(vI.getY(), vI.getX());
    double deltaX;
    double deltaY;
    if (vI.getTheta() != 0.0) {
      auto arcRadius = velMag / vI.getTheta();
      deltaX =
        arcRadius * (sin(thetaF + velAngle) - sin(p.getTheta() + velAngle));
      deltaY =
        arcRadius * (-cos(thetaF + velAngle) + cos(p.getTheta() + velAngle));
    } else {
      deltaX = cycleTime * vI.getX();
      deltaY = cycleTime * vI.getY();
    }
    p.x() += deltaX;
    p.y() += deltaY;
    p.theta() = thetaF;
    p.x() = dist(random, p.getX(), motionStd[0]);
    p.y() = dist(random, p.getY(), motionStd[1]);
    p.theta() = dist(random, p.getTheta(), motionStd[2]);
    p.ct = cos(p.theta());
    p.st = sin(p.theta());
  }
}

void ParticleFilter::addPredictionNoise()
{
  NormalDistribution<double> dist;
  for (auto& p : particles) {
    p.x() = dist(random, p.getX(), predictionStd[0]);
    p.y() = dist(random, p.getY(), predictionStd[1]);
    p.theta() = dist(random, p.getTheta(), predictionStd[2]);
    p.ct = cos(p.theta());
    p.st = sin(p.theta());
  }
}

void ParticleFilter::updateWeights(const vector<LandmarkPtr>& obsLandmarks)
{
  //auto worldImage = Mat(vMapSize, CV_8UC3, Scalar(0,0,0));
  /*for (const auto& ll : fieldLandmarks) {
    if (!ll) continue;
    VisionUtils::drawPoint(
      Point(ll->pos.x * 100 + 500, 350 - 100 * ll->pos.y),
      worldImage, Scalar(0,0,255));
  }*/
  sumWeights = 0.0;
  maxWeight = 0.0;
  static const auto outOfRangeConst = 0.25 / sqrt(2);
  static const auto obsMatchMaxDist = 0.10;
  if (!obsLandmarks.empty()) {
    for (auto& p : particles) {
      //worldImage = Scalar(0,0,0);
      //drawParticle(p, worldImage, Scalar(0,0,255));
      auto& w = p.weight;
      vector<bool> lMarked(fieldLandmarks.size(), false);
      for (const auto& obs : obsLandmarks) {
        if (!obs) continue;
        bool badObs = false;
        auto tPos = static_cast<RobotPose2D<float>>(p).transform(obs->pos);
        if (abs(tPos.x) > 4.75 || abs(tPos.y) > 3.25) {
          badObs = true;
        }
        //VisionUtils::drawPoint(Point(tPos.x * 100 + 500, 350 - 100 * tPos.y), worldImage, Scalar(255,0,0));
        if (!badObs) {
          auto tPosWorld = worldToVoronoiMap(tPos);
          auto label = voronoiLabels.at<uint>(tPosWorld.y, tPosWorld.x);
          auto minDist = 1000.0;
          //if (!labelledLandmarks[label-1].empty()) {
          //  index = labelledLandmarks[label-1][0]->id;
          //  minDist = norm(labelledLandmarks[label-1][0]->pos - tPos);
          //}
          LandmarkPtr matchedLandmark;
          for (const auto& ll : labelledLandmarks[label - 1]) {
            if (!lMarked[ll->id]) {
              if (obs->type == ll->type) {
                auto d = norm(ll->pos - tPos);
                if (d < minDist) {
                  minDist = d;
                  matchedLandmark = ll;
                }
              }
            }
          }
          if (minDist < obsMatchMaxDist) {
            auto dx = (tPos.x - matchedLandmark->pos.x);
            dx *= dx;
            auto dy = (tPos.y - matchedLandmark->pos.y);
            dy *= dy;
            w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
            //VisionUtils::drawPoint(Point(matchedLandmark->pos.x * 100 + 500, 350 - 100 * matchedLandmark->pos.y), worldImage, Scalar(0,255,0));
            lMarked[matchedLandmark->id] = true;
          } else {
            auto dx = outOfRangeConst;
            dx *= dx;
            auto dy = outOfRangeConst;
            dy *= dy;
            w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
          }
        } else {
          auto dx = outOfRangeConst;
          dx *= dx;
          auto dy = outOfRangeConst;
          dy *= dy;
          w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
        }
      }
      sumWeights += w;
      if (w > maxWeight) {
        maxWeight = w;
        bestParticle = &p;
      }
      //VisionUtils::displayImage("worldImage", worldImage, 0.5);
      //waitKey(0);
    }
  }
  //drawParticle(Particle(avgState.getX(), avgState.getY(), avgState.getTheta()), worldImage);
  //for (const auto& obs : obsLandmarks) {
  //  auto tPos = static_cast<RobotPose2D<float>>(avgState).transform(obs->pos);
  //  VisionUtils::drawPoint(Point(tPos.x * 100 + 500, 350 - 100 * tPos.y), worldImage, Scalar(255,0,0));
  //}
  //VisionUtils::displayImage("worldImage", worldImage, 0.25);
  //waitKey(0);
  auto avgWeights = sumWeights / nParticles;
  wSlow += ALPHA_SLOW * (avgWeights - wSlow);
  wFast += ALPHA_FAST * (avgWeights - wFast);
}

void ParticleFilter::normalizeWeights(const bool& noData)
{
  ///< If no data or the highest weight is close to zero
  if (noData) {
    for (auto& p : particles)
      p.weight = 1.0 / (float) nParticles;
  } else {
    if (maxWeight < 1e-9) {
      lostCount++;
      if (lostCount > MAX_LOST_COUNT) {
        cout << "lostCount > MAXLOSTCOUNT" << endl;
        reset();
        lostCount = 0;
      } else {
        for (auto& p : particles)
          p.weight = 1.0 / (float) nParticles;
      }
    } else {
      for (auto& p : particles)
        p.weight /= sumWeights;
      lostCount = 0;
    }
  }
}

void ParticleFilter::resample()
{
  vector<Particle> resampled;
  auto rNumber = random.FloatU() * (1.0 / nParticles);
  auto w = particles[0].weight;
  int j = 0;
  auto vl = 0;
  auto threshold = 1.0 - (wFast / wSlow);
  if (threshold > vl) vl = threshold;
  for (size_t i = 0; i < nParticles; ++i) {
    auto u = rNumber + i / (double) nParticles;
    while (u > w) {
      j++;
      w += particles[j].weight;
    }
    resampled.push_back(particles[j]);
  }

  avgState.x() = 0.f;
  avgState.y() = 0.f;
  avgState.theta() = 0.f;
  auto resampleWeight = 1.0 / (double)nParticles;
  for (auto& p : resampled) {
    avgState.x() += p.getX();
    avgState.y() += p.getY();
    avgState.theta() += (p.getTheta() < 0 ? MathsUtils::M_TWICE_PI + p.getTheta() : p.getTheta());
    p.weight = resampleWeight;
  }
  avgState /= (float)nParticles;
  avgState.theta() = MathsUtils::rangeToPi(avgState.getTheta());
  avgState.ct = cos(avgState.theta());
  avgState.st = sin(avgState.theta());
  particles = resampled;
}

void ParticleFilter::updatePositionConfidence()
{
  if (bestParticle)
    LAST_POSE_2D_OUT(LocalizationModule) = *bestParticle;
  auto std = 0.0;
  for (const auto& p : particles) {
    std += (p - avgState).get().segment(0, 2).norm();
  }
  std = sqrt(std / nParticles);
  auto& positionConfidence = POSITION_CONFIDENCE_OUT(LocalizationModule);
  static auto maxEstimateStd = 1.0;
  positionConfidence = (1 - min(std, maxEstimateStd) / maxEstimateStd) * 100;
  if (POSITION_CONFIDENCE_OUT(LocalizationModule) < 25) {
    localized = false;
  } else {
    localized = true;
  }
}

void ParticleFilter::updateSideConfidence()
{
  if (!bestParticle)
    return;
  const auto& goalInfo = GOAL_INFO_IN(LocalizationModule);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found && goalInfo.type != GoalPostType::unknown) {
      if (goalInfo.mid == goalInfo.mid) { // NAN
        Point2f goalInMap;
        const auto& ct = bestParticle->getCTheta();
        const auto& st = bestParticle->getSTheta();
        goalInMap.x =
          bestParticle->x() + goalInfo.mid.x * ct - goalInfo.mid.y * st;
        goalInMap.y =
          bestParticle->y() + goalInfo.mid.x * st + goalInfo.mid.y * ct;
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
    for (auto& p : particles) {
      p.x() = -p.x();
      p.y() = -p.y();
      p.theta() = p.theta() + M_PI;
      p.ct = cos(p.theta());
      p.st = sin(p.theta());
    }
    SIDE_CONFIDENCE_OUT(LocalizationModule) = 70;
  } else {
    lastKnownHalf = bestParticle->getX() > 0 ? FieldHalf::oppHalf : FieldHalf::teamHalf;
  }
}

RobotPose2D<float> ParticleFilter::getBestState() const {
  if (bestParticle)
    return *bestParticle;
  else
    return LAST_POSE_2D_OUT(LocalizationModule);
}

void ParticleFilter::addPositionInput(const PositionInput<float>& input)
{
  this->positionInputs.push(input);
}

void ParticleFilter::addVelocityInput(const VelocityInput<float>& input)
{
  this->velocityInputs.push(input);
}

void ParticleFilter::setKnownLandmarks(const vector<KnownLandmarkPtr>& landmarks)
{
  this->knownLandmarks = landmarks;
}

void ParticleFilter::setUnknownLandmarks(const vector<UnknownLandmarkPtr>& landmarks)
{
  this->unknownLandmarks = landmarks;
}

void ParticleFilter::drawParticle(const RobotPose2D<float>& p, Mat& image, const Scalar& color)
{
  if (!image.empty()) {
    auto lineEnd = static_cast<RobotPose2D<float>>(p).transform(Point2f(0.15, 0.0));
    circle(image, worldToVoronoiMap(Point2f(p.getX(), p.getY())), 10, color);
    line(
      image,
      worldToVoronoiMap(Point2f(p.getX(), p.getY())),
      worldToVoronoiMap(lineEnd),
      color
    );
  }
}
