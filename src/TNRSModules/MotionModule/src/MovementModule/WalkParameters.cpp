/**
 * @file MotionModule/src/MovementModule/WalkParameters.cpp
 *
 * This file implements the struct WalkParameters
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/MovementModule/WalkParameters.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/Constants.h"
#include "Utils/include/ConfigMacros.h"

template <typename Scalar>
Scalar WalkParameters<Scalar>::getFootSeparation(const RobotFeet& stepFoot)
{
  if (stepFoot == RobotFeet::lFoot) return Constants::footSeparation;
  else return -Constants::footSeparation;
}

template <typename Scalar>
void WalkParameters<Scalar>::loadConfig()
{
  GET_CONFIG(
    "WalkConfig",
    (Scalar, comHeight, comHeight),
    (Scalar, stepHeight, stepHeight),
    (Scalar, maxStepLengthX, maxStepLengthX),
    (Scalar, maxStepLengthY, maxStepLengthY),
    (Scalar, maxStepRotation, maxStepRotation),
    (Scalar, nPreviews, nPreviews),
    (Scalar, totalStepTime, totalStepTime),
    (Scalar, firstStepTime, firstStepTime),
  )
}

template struct WalkParameters<float>;
template struct WalkParameters<double>;
