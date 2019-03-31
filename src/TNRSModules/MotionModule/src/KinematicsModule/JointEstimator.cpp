/**
 * @file MotionModule/src/KinematicsModule/JointEstimator.cpp
 *
 * This file implements the class JointEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include <boost/make_shared.hpp>
#include "MotionModule/include/KinematicsModule/JointEstimator.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/Constants.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/JsonUtils.h"

template <typename Scalar>
void JointEstimator<Scalar>::init(
  const Joints& jointIndex,
  const Matrix<Scalar, 2, 1>& initState,
  const Scalar& dt)
{
  //! Read the joint configuration
  Json::Value json;
  try {
    using namespace std;
    static string measFilePath =
      ConfigManager::getConfigDirPath() + "JointEstimator.json";
    ifstream config(measFilePath, ifstream::binary);
    config >> json;
  } catch (const Json::Exception& e) {
    LOG_EXCEPTION(e.what());
  }
  auto jointConfig = json[Constants::jointNames[toUType(jointIndex)]];
  useInput = jointConfig["useInput"].asBool();

  this->dt = dt;
  Matrix<Scalar, 2, 2> A, Q;
  Matrix<Scalar, 2, 1> B;
  //! State Transition Matrix A
  A << 1.0, dt, 0, 1.0;
  //! Control Input Matrix B
  B << dt * dt / 2, dt;
  model =
    boost::shared_ptr<ProcessModel<Scalar, 2, 1, 1> >(
      new ProcessModel<Scalar, 2, 1, 1>(A, B));
  //! Process Noise Covariance Matrix Q
  Q.setZero();
  Q(0, 0) = jointConfig["procVariance"][0].asFloat();
  Q(1, 1) = jointConfig["procVariance"][1].asFloat();
  model->setNoiseCovMatrix(Q);

  //! Initial state
  model->setState(initState);
  lastCmd = initState[0];
  Matrix<Scalar, 1, 2> H;
  //! Measure Matrix H
  H << 1.0, 0.0;
  filter =
    boost::shared_ptr<KalmanFilter<Scalar, 2, 1, 1, 1> >(
      new KalmanFilter<Scalar, 2, 1, 1, 1>(model, H));
  filter->setMeasMatrix(H);
  R(0, 0) = jointConfig["measVariance"].asFloat();;
  filter->setMeasNoiseCov(R);

  Matrix<Scalar, 3, 1> pid;
  pid[0] = jointConfig["pid"][0].asFloat();
  pid[1] = jointConfig["pid"][1].asFloat();
  pid[2] = jointConfig["pid"][2].asFloat();
  pidC[0] = pid[0] + pid[1] * dt / 2 + pid[2] / dt;
  pidC[1] = -pid[0] + pid[1] * dt / 2 - 2 * pid[2] / dt;
  pidC[2] = pid[2] / dt;
  initiated = true;
}

template <typename Scalar>
void JointEstimator<Scalar>::setControl(const Scalar& cmd)
{
  lastCmd = cmd;
}

template <typename Scalar>
void JointEstimator<Scalar>::setPidGains(const Matrix<Scalar, 3, 1>& pid) {
  pidC[0] = pid[0] + pid[1] * dt / 2 + pid[2] / dt;
  pidC[1] = -pid[0] + pid[1] * dt / 2 - 2 * pid[2] / dt;
  pidC[2] = pid[2] / dt;
}

template <typename Scalar>
void JointEstimator<Scalar>::updateModel()
{
  #ifndef NAOQI_MOTION_PROXY_AVAILABLE
  if (useInput) {
    auto error = (lastCmd - model->getState()[0]);
    auto input = prevInput + pidC[0] * error + pidC[1] * prevError1 + pidC[2] * prevError2;
    model->setInput(input);
    prevError2 = prevError1;
    prevError1 = error;
    prevInput = input;
  }
  #endif
  filter->predict();
}

template <typename Scalar>
void JointEstimator<Scalar>::update(const Scalar& meas)
{
  Matrix<Scalar, 1, 1> measFixed;
  measFixed[0] = meas;
  if (measFixed[0] != measFixed[0]) {// || MathsUtils::almostEqual(measFixed[i], lastMeas[i], (Scalar)1e-5)) {
    // High covariance for unknown measurement
    filter->setMeasNoiseCov(1e9, 0, 0);
    measFixed[0] = 0.0;
  } else {
    filter->setMeasNoiseCov(R(0, 0), 0, 0);
  }
  updateModel();
  filter->correct(measFixed);
  lastMeas = meas;
}

template <typename Scalar>
void JointEstimator<Scalar>::reset(const Matrix<Scalar, 2, 1>& state)
{
  model->setState(state);
  filter->reset();
  lastCmd = state[0];
  if (useInput) {
    prevError2 = 0.0;
    prevError1 = 0.0;
    prevInput = 0.0;
  }
}


template class JointEstimator<float>;
template class JointEstimator<double>;

