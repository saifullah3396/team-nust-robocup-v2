/**
 * @file MotionModule/src/KinematicsModule/ComEstimator.cpp
 *
 * This file implements the class ComEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include <boost/make_shared.hpp>
#include "MotionModule/include/KinematicsModule/ComEstimator.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ConfigMacros.h"

template <typename Scalar, size_t MeasSize>
void ComEstimator<Scalar, MeasSize>::init(
  const Matrix<Scalar, 3, 1>& initState,
  const Scalar& comHeight,
  const Scalar& dT)
{
  Matrix<Scalar, 3, 3> A, Q;
  Matrix<Scalar, 3, 1> B;
  //! State Transition Matrix A
  //! [ 1   dT  dT^2/2 ] [  x     ]
  //! [ 0   1   dT     ] [  xdot  ]
  //! [ 0   0   1      ] [  xddot ]
  A << 1.f, dT, dT * dT / 2, 0.f, 1.f, dT, 0.f, 0.f, 1.f;
  //! Control Input Matrix B
  //! [ dT^3/6 ]
  //! [ dT^2/2 ]
  //! [ dT ]
  B << dT * dT * dT/ 6, dT * dT / 2, dT;
  C << 1, 0, -comHeight / Constants::gravity;
  model =
    boost::shared_ptr<ProcessModel<Scalar, 3, 1, 1> >(
      new ProcessModel<Scalar, 3, 1, 1> (A, B, C));
  //! Process Noise Covariance Matrix Q
  //! [ eX   0     0   ]
  //! [ 0    eVX   0   ]
  //! [ 0    0     eAx ]
  Scalar procVar1, procVar2, procVar3;
  GET_CONFIG(
    "KinCalibration",
    (Scalar, ComEstimator.procPosVar, procVar1),
    (Scalar, ComEstimator.procVelVar, procVar2),
    (Scalar, ComEstimator.procAccVar, procVar3),
  );
  Q.setZero();
  Q(0, 0) = procVar1;
  Q(1, 1) = procVar2;
  Q(2, 2) = procVar3;
  model->setNoiseCovMatrix(Q);

  //! Initial state
  model->setState(initState);
  //! Measure Matrix H
  //! [ x    ] [ 1 0    0 ] [ x    ]
  //! [ xddot] [ 0 0    1 ] [ xdot ]
  //! [ zmp  ] [ 1 0 -z/g ] [ xddot]
  H << 1, 0, 0, 0, 0, 1, 1, 0, -comHeight / Constants::gravity;
  filter =
    boost::shared_ptr<KalmanFilter<Scalar, 3, 3, 1, 1> >(
      new KalmanFilter<Scalar, 3, 3, 1, 1>(model, H));
  filter->setMeasMatrix(H);
  Scalar measVar1, measVar2, measVar3;
  GET_CONFIG(
    "KinCalibration",
    (Scalar, ComEstimator.measPosVar, measVar1),
    (Scalar, ComEstimator.measAccVar, measVar2),
    (Scalar, ComEstimator.measZmpVar, measVar3),
  );
  //LOG_INFO("Q:\n: " << Q);
  this->R(0, 0) = measVar1;
  this->R(1, 1) = measVar2;
  this->R(2, 2) = measVar3;
  //LOG_INFO("this->R:\n: " << this->R);
  filter->setMeasNoiseCov(R);
  initiated = true;
}

template <typename Scalar, size_t MeasSize>
void ComEstimator<Scalar, MeasSize>::update(const Matrix<Scalar, MeasSize, 1>& meas)
{
  Matrix<Scalar, MeasSize, 1> measFixed = meas;
  for (size_t i = 0; i < meas.rows(); ++i) {
    if (measFixed[i] != measFixed[i]) {// || MathsUtils::almostEqual(measFixed[i], lastMeas[i], (Scalar)1e-5)) {
      // High covariance for unknown measurement
      filter->setMeasNoiseCov(1e9, i, i);
      measFixed[i] = 0.0;
    } else {
      filter->setMeasNoiseCov(R(i, i), i, i);
    }
  }
  //cout << "Updating com estimator..." << endl;
  filter->predict();
  filter->correct(measFixed);
  lastMeas = meas;
}

template class ComEstimator<MType, 3>;
