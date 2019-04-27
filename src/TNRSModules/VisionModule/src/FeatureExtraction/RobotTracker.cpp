/**
 * @file MotionModule/src/KinematicsModule/RobotTracker.cpp
 *
 * This file implements the class RobotTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include <boost/make_shared.hpp>
#include "VisionModule/include/FeatureExtraction/RobotTracker.h"
#include "VisionModule/include/FeatureExtraction/RobotRegion.h"
#include "Utils/include/JsonUtils.h"

float RobotTracker::dt = 0.05;
Matrix<float, 12, 12> RobotTracker::A;
Matrix<float, 12, 12> RobotTracker::Q;
Matrix<float, 10, 10> RobotTracker::R;
Matrix<float, 12, 1> RobotTracker::B;
Matrix<float, 10, 12> RobotTracker::H;

void RobotTracker::init(
  const Matrix<float, 12, 1>& initState,
  const float& dt,
  const float& time)
{
  loadModel(dt);
  model =
    boost::shared_ptr<ProcessModel<float, 12, 1, 1> >(
      new ProcessModel<float, 12, 1, 1>(A, B));
  model->setNoiseCovMatrix(Q);
  model->setState(initState);
  filter =
    boost::shared_ptr<KalmanFilter<float, 12, 10, 1, 1> >(
      new KalmanFilter<float, 12, 10, 1, 1>(model, H));
  filter->setMeasMatrix(H);
  filter->setMeasNoiseCov(R);
  timeUpdated = time;
  initiated = true;
}

void RobotTracker::loadModel(const float& dt)
{
  static auto loaded = false;
  if (!loaded) {
    ///< Read the joint configuration
    Json::Value json;
    try {
      using namespace std;
      static string measFilePath =
        ConfigManager::getCommonConfigDirPath() + "RobotTracker.json";
      ifstream config(measFilePath, ifstream::binary);
      config >> json;
    } catch (const Json::Exception& e) {
      LOG_EXCEPTION(e.what());
    }

    this->dt = dt;
    ///< State Transition Matrix A
    A.setIdentity();
    A(0, 2) = dt;
    A(1, 3) = dt;

    ///< Control Input Matrix B
    B.setZero();
    ///< Process Noise Covariance Matrix Q
    Q.setZero();
    for (int i = 0; i < 12; ++i)
      Q(i, i) = json["procVariance"][i].asFloat();

    H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    for (int i = 0; i < 10; ++i)
      R(i, i) = json["measVariance"][i].asFloat();
    loaded = true;
  }
}

void RobotTracker::update(const Matrix<float, 10, 1>& meas, const float& time)
{
  Matrix<float, 10, 1> measFixed = meas;
  for (size_t i = 0; i < 10; ++i) {
    if (measFixed[i] != measFixed[i]) {
      filter->setMeasNoiseCov(1e9, i, i);
      measFixed[i] = 0.0;
    } else {
      filter->setMeasNoiseCov(R(i, i), i, i);
    }
  }
  filter->predict();
  filter->correct(measFixed);
  timeUpdated = time;
}

void RobotTracker::reset(const Matrix<float, 12, 1>& state)
{
  model->setState(state);
  filter->reset();
}

