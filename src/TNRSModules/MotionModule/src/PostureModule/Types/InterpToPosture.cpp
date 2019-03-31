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
  const boost::shared_ptr<InterpToPostureConfig>& config) :
  PostureModule<Scalar>(motionModule, config, "InterpToPosture")
{
}

template <typename Scalar>
boost::shared_ptr<InterpToPostureConfig> InterpToPosture<Scalar>::getBehaviorCast()
{
  return SPC(InterpToPostureConfig, this->config);
}

template <typename Scalar>
bool InterpToPosture<Scalar>::initiate()
{
  LOG_INFO("InterpToPosture.initiate() called...")
  //! Set target joints from degrees to radians
  auto& jointsToReach = this->getBehaviorCast()->jointsToReach;
  jointsToReach *= MathsUtils::DEG_TO_RAD;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->jointsI = this->kM->getJointPositions();
  this->jointsDelta = jointsToReach - this->jointsI;
  Matrix<bool, Dynamic, 1> activeJoints = this->jointsDelta.cwiseAbs().array() > Angle::DEG_1;
  if (!activeJoints.any()) { //! Posture already reached
    POSTURE_STATE_OUT(MotionModule) = this->getBehaviorCast()->targetPosture;
    return false;
  } else {
    auto timeStep = this->cycleTime;
    vector<Matrix<Scalar, Dynamic, 1> > joints;
    vector<Scalar> times;
    while (timeStep <= this->getBehaviorCast()->timeToReachP) {
      auto timeParam = timeStep / this->getBehaviorCast()->timeToReachP;
      joints.push_back(this->interpolate(timeParam));
      times.push_back(timeStep);
      timeStep += this->cycleTime;
    }
    this->naoqiJointInterpolation(joints, times, activeJoints, true);
    return true;
  }
  #else
  this->jointsI = this->kM->getJointPositions();
  this->jointsDelta = jointsToReach - this->jointsI;
  Matrix<bool, Dynamic, 1> activeJoints = this->jointsDelta.cwiseAbs().array() > Angle::DEG_1;
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
    auto step = this->runTime / this->getBehaviorCast()->timeToReachP;
    this->setJointCmds(this->interpolate(step));
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
