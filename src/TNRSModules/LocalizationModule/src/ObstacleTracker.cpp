/**
 * @file MotionModule/src/KinematicsModule/ObstacleTracker.cpp
 *
 * This file implements the class ObstacleTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 April 2019
 */

#include <boost/make_shared.hpp>
#include "LocalizationModule/include/ObstacleTracker.h"
#include "Utils/include/JsonUtils.h"

float ObstacleTracker::dt = 0.05;
Matrix<float, STATE_SIZE, STATE_SIZE> ObstacleTracker::A;
Matrix<float, STATE_SIZE, STATE_SIZE> ObstacleTracker::Q;
Matrix<float, MEAS_SIZE, MEAS_SIZE> ObstacleTracker::R;
Matrix<float, STATE_SIZE, 1> ObstacleTracker::B;
Matrix<float, MEAS_SIZE, STATE_SIZE> ObstacleTracker::H;

void ObstacleTracker::init(
  const Matrix<float, STATE_SIZE, 1>& initState,
  const float& dt,
  const float& time)
{
  loadModel(dt);
  model =
    boost::shared_ptr<ProcessModel<float, STATE_SIZE, 1, 1> >(
      new ProcessModel<float, STATE_SIZE, 1, 1>(A, B));
  model->setNoiseCovMatrix(Q);
  model->setState(initState);
  filter =
    boost::shared_ptr<KalmanFilter<float, STATE_SIZE, MEAS_SIZE, 1, 1> >(
      new KalmanFilter<float, STATE_SIZE, MEAS_SIZE, 1, 1>(model, H));
  filter->setMeasMatrix(H);
  filter->setMeasNoiseCov(R);
  timeUpdated = time;
  initiated = true;
}

void ObstacleTracker::loadModel(const float& dt)
{
  static auto loaded = false;
  if (!loaded) {
    ///< Read the joint configuration
    Json::Value json;
    try {
      using namespace std;
      static string measFilePath =
        ConfigManager::getCommonConfigDirPath() + "ObstacleTracker.json";
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
    for (int i = 0; i < STATE_SIZE; ++i)
      Q(i, i) = json["procVariance"][i].asFloat();

    H << 1, 0, 0, 0;
         0, 1, 0, 0;

    for (int i = 0; i < MEAS_SIZE; ++i)
      R(i, i) = json["measVariance"][i].asFloat();
    loaded = true;
  }
}

void ObstacleTracker::update(const Matrix<float, MEAS_SIZE, 1>& meas, const float& time)
{
  Matrix<float, MEAS_SIZE, 1> measFixed = meas;
  for (size_t i = 0; i < MEAS_SIZE; ++i) {
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

void ObstacleTracker::reset(const Matrix<float, STATE_SIZE, 1>& state)
{
  model->setState(state);
  filter->reset();
}

