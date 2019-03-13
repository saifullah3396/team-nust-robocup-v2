/**
 * @file MotionModule/src/PostureModule/Type/InterpToPosture.cpp
 *
 * This file implements the class InterpToPosture
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/PostureModule/Types/InterpToPosture.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/JointRequest.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/AngleDefinitions.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "Utils/include/MathsUtils.h"

template <typename Scalar>
InterpToPosture<Scalar>::InterpToPosture(
  MotionModule* motionModule,
  const boost::shared_ptr<MBPostureConfig>& config) :
  PostureModule<Scalar>(motionModule, config, "InterpToPosture")
{
}

template <typename Scalar>
bool InterpToPosture<Scalar>::initiate()
{
  LOG_INFO("InterpToPosture.initiate() called...")
  //! Set target joints from degrees to radians
  auto& jointsToReach = this->getBehaviorCast()->jointsToReach;
  jointsToReach *= MathsUtils::DEG_TO_RAD;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  jointsI = this->kM->getJointPositions();
  jointsDelta = jointsToReach - jointsI;
  Matrix<bool, Dynamic, 1> activeJoints = jointsDelta.cwiseAbs().array() > Angle::DEG_1;
  if (!activeJoints.any()) { //! Posture already reached
    POSTURE_STATE_OUT(MotionModule) = this->getBehaviorCast()->targetPosture;
    return false;
  } else {
    auto timeStep = this->cycleTime;
    vector<Matrix<Scalar, Dynamic, 1> > joints;
    vector<Scalar> times;
    while (timeStep <= this->getBehaviorCast()->timeToReachP) {
      auto timeParam = timeStep / this->getBehaviorCast()->timeToReachP;
      auto multiplier =
        6 * pow(timeParam, 5) - 15 * pow(timeParam, 4) + 10 * pow(timeParam, 3);
      joints.push_back(jointsI + jointsDelta * multiplier);
      times.push_back(timeStep);
      timeStep += this->cycleTime;
    }
    this->naoqiJointInterpolation(joints, times, activeJoints, true);
    return true;
  }
  #else
  jointsI = this->kM->getJointPositions();
  jointsDelta = jointsToReach - jointsI;
  Matrix<bool, Dynamic, 1> activeJoints = jointsDelta.cwiseAbs().array() > Angle::DEG_1;
  if (!activeJoints.any()) { //! Posture already reached
    POSTURE_STATE_OUT(MotionModule) = this->getBehaviorCast()->targetPosture;
    return false;
  } else {
    return true;
  }
  #endif
}

template <typename Scalar>
void InterpToPosture<Scalar>::update()
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->runTime > this->getBehaviorCast()->timeToReachP) {
    POSTURE_STATE_OUT(MotionModule) = this->getBehaviorCast()->targetPosture;
    finish();
  }
  #else
  if (this->runTime > this->getBehaviorCast()->timeToReachP) {
    POSTURE_STATE_OUT(MotionModule) =
      this->getBehaviorCast()->targetPosture;
    finish();
  } else {
    auto timeParam = this->runTime / this->getBehaviorCast()->timeToReachP;
    auto multiplier =
      6 * pow(timeParam, 5) - 15 * pow(timeParam, 4) + 10 * pow(timeParam, 3);
    this->setJointCmds(this->jointsI + this->jointsDelta * multiplier);
  }
  #endif
}

template <typename Scalar>
void InterpToPosture<Scalar>::finish()
{
  LOG_INFO("InterpToPosture.finish() called...")
  this->inBehavior = false;
}

template class InterpToPosture<MType>;
