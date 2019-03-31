/**
 * @file MotionModule/src/TrajectoryPlanner/JointInterpolator.cpp
 *
 * This file implements the class JointInterpolator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#include "MotionModule/include/TrajectoryPlanner/JointInterpolator.h"

/**
 * @class JointInterpolator
 * @brief The class to provide an interface for joint interpolation
 */
template <typename Scalar>
JointInterpolator<Scalar>::JointInterpolator() {}

template <typename Scalar>
JointInterpolator<Scalar>::~JointInterpolator() {}

template <typename Scalar> Matrix<Scalar, Dynamic, 1>
JointInterpolator<Scalar>::interpolate(const Scalar& step)
{
  auto multiplier =
    6 * pow(step, 5) - 15 * pow(step, 4) + 10 * pow(step, 3);
  return this->jointsI + this->jointsDelta * multiplier;
}

template class JointInterpolator<MType>;
