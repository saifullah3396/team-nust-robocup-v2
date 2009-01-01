/**
 * @file MotionModule/src/KinematicsModule/ImuFilter.cpp
 *
 * This file implements the class ImuFilter
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "MotionModule/include/KinematicsModule/ImuFilter.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PrintUtils.h"

template <typename Scalar>
ImuFilter<Scalar>::ImuFilter(
  const Scalar& cycleTime) :
  cycleTime(cycleTime)
{
}

template <typename Scalar>
void ImuFilter<Scalar>::initiate() {
  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;
  beta = sqrt(3.0/4.0) * gyrMeasError;
}

template <typename Scalar>
void ImuFilter<Scalar>::update(
  Eigen::Matrix<Scalar, 3, 1> measAcc,
  const Eigen::Matrix<Scalar, 3, 1>& measGyr)
{
  for (size_t i = 0; i < maxIterations; ++i) {
    Eigen::Matrix<Scalar, 4, 1> qHalf = 0.5 * q;
    Eigen::Matrix<Scalar, 4, 1> qTwo = 2.0 * q;
    measAcc /= measAcc.norm();
    Eigen::Matrix<Scalar, 3, 1> f;
    f << qTwo[1] * q[3] - qTwo[0] * q[2] - measAcc[0],
         qTwo[0] * q[1] + qTwo[2] * q[3] - measAcc[1],
         1.0 - qTwo[1] * q[1] - qTwo[2] * q[2] - measAcc[2];
    Eigen::Matrix<Scalar, 3, 4> jacobian;
    jacobian << -qTwo[2], qTwo[3], -qTwo[0], qTwo[1],
                qTwo[1], qTwo[0], qTwo[3], qTwo[2],
                0.0, -2 * qTwo[1], -2 * qTwo[2], 0.0;
    Eigen::Matrix<Scalar, 4, 1> grad = jacobian.transpose() * f;
    auto norm = grad.norm();
    if (norm > 0.0)
      grad /= norm;
    Eigen::Matrix<Scalar, 4, 1> qd;
    qd[0] = -qHalf[1] * measGyr[0] - qHalf[2] * measGyr[1] - qHalf[3] * measGyr[2];
    qd[1] = qHalf[0] * measGyr[0] + qHalf[2] * measGyr[2] - qHalf[3] * measGyr[1];
    qd[2] = qHalf[0] * measGyr[1] - qHalf[1] * measGyr[2] + qHalf[3] * measGyr[0];
    qd[3] = qHalf[0] * measGyr[2] + qHalf[1] * measGyr[1] - qHalf[2] * measGyr[0];
    q += (qd - 0.5 * grad) * cycleTime;
    q /= q.norm();
  }
  Quaternion<Scalar> qq;
  qq.w() = q[0];
  qq.x() = q[1];
  qq.y() = q[2];
  qq.z() = q[3];
  LOG_INFO("qMat:\n" << qq.toRotationMatrix().matrix());
}

template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> ImuFilter<Scalar>::getQuaternion() {
  return q;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> ImuFilter<Scalar>::getRotation() {
  return Quaternion<Scalar>(Quaternion<Scalar>(q[0], q[1], q[2], q[3])).toRotationMatrix().matrix();
}

template class ImuFilter<float>;
template class ImuFilter<double>;
