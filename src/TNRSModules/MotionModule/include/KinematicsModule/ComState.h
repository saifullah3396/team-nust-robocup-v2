/**
 * @file MotionModule/include/KinematicsModule/ComState.h
 *
 * This file defines the struct ComState
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "Utils/include/MathsUtils.h"
#include "Utils/include/HardwareIds.h"

/**
 * @struct ComState
 * @brief Robot torso state definition
 */
template <typename Scalar>
struct ComState
{
  /**
   * Constructor
   */
  ComState()
  {
    position.setZero();
    velocity.setZero();
    accel.setZero();
    zmp.setZero();
    baseFrame = RobotFeet::lFoot;
    eeIndex = 0;
  }

  //! The inertial frame for com position.
  RobotFeet baseFrame;
  unsigned eeIndex;

  Matrix<Scalar, 3, 1> position;
  Matrix<Scalar, 3, 1> velocity;
  Matrix<Scalar, 3, 1> accel;
  Matrix<Scalar, 2, 1> zmp;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
