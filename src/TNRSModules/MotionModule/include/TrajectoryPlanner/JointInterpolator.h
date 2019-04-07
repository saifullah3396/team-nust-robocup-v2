/**
 * @file MotionModule/include/TrajectoryPlanner/JointInterpolator.h
 *
 * This file declares the class JointInterpolator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once

#include "Eigen/Dense"
#include "MotionModule/include/MTypeHeader.h"

using namespace Eigen;

/**
 * @class JointInterpolator
 * @brief The class to provide an interface for joint interpolation
 */
template <typename Scalar>
class JointInterpolator
{
public:
  /**
   * @brief JointInterpolator Constructor.
   *
   * @param motionModule Pointer to base motion module
   */
  JointInterpolator();

  /**
   * @brief ~JointInterpolator Destructor
   */
  ~JointInterpolator();

  /**
   * @brief interpolate Returns the interpolated value for step between 0.0 to 1.0
   * @param step Current step
   * @return Current Joint values
   */
  Matrix<Scalar, Dynamic, 1> interpolate(const Scalar& step);

protected:
  ///< Joint configuration initial
  Matrix<Scalar, Dynamic, 1> jointsI;

  ///< Joint configuration difference
  Matrix<Scalar, Dynamic, 1> jointsDelta;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

