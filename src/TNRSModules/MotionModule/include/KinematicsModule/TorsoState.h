/**
 * @file MotionModule/include/KinematicsModule/TorsoState.h
 *
 * This file defines the struct TorsoState
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "Utils/include/MathsUtils.h"

/**
 * @struct TorsoState
 * @brief Robot torso state definition
 */
template<typename Scalar>
struct TorsoState
{
  /**
   * Constructor
   */ 
  TorsoState()
  {
    velocity.setZero();
    angularVelocity.setZero();
    accel.setZero();
    rot.setIdentity();
    bias.setZero();
    scale.setZero();
  }
  
  //! Torso velocity
  Matrix<Scalar, 3, 1> velocity;

  //! Torso angular velocity
  Matrix<Scalar, 3, 1> angularVelocity;
  
  //! Torso acceleration
  Matrix<Scalar, 3, 1> accel;
  
  //! Torso rotation
  Matrix<Scalar, 4, 4> rot;

  //! Imu Bias
  Matrix<Scalar, 3, 1> bias;

  //! Imu scale factor
  Matrix<Scalar, 3, 3> scale;
};
template struct TorsoState<float>;
template struct TorsoState<double>;
