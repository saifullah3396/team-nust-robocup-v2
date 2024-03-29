/**
 * @file MotionModule/include/MovementModule/WalkParameters.h
 *
 * This file defines the struct WalkParameters
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/make_shared.hpp>
#include "Utils/include/AngleDefinitions.h"

enum class RobotFeet : unsigned int;

/**
 * @struct WalkParameters
 * @brief A struct that holds information about the walk parameters
 */
template <typename Scalar>
struct WalkParameters
{
  WalkParameters() = default;
  WalkParameters(const WalkParameters&) = default;
  WalkParameters(WalkParameters&&) = default;
  WalkParameters& operator=(const WalkParameters&) & = default;
  WalkParameters& operator=(WalkParameters&&) & = default;
  virtual ~WalkParameters() {}

  void loadConfig();
  Scalar getFootSeparation(const RobotFeet& stepFoot);

  Scalar comHeight = {0.26};
  Scalar stepHeight = {0.02};
  Scalar minStepLengthX = {-0.025};
  Scalar minStepLengthY = {-0.0};
  Scalar minStepRotation = {0.0};
  Scalar maxStepLengthX = {0.05};
  Scalar maxStepLengthY = {0.025};
  Scalar maxStepRotation = {Angle::DEG_15};
  Scalar nPreviews = {100};
  Scalar totalStepTime = {0.2};
  Scalar firstStepTime = {0.6};
};
