/**
 * @file MotionModule/src/BalanceModule/Types/MPComControl.cpp
 *
 * This file implements the class MPComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "MotionModule/include/BalanceModule/Types/MPComControl.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/MotionModule.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/AngleDefinitions.h"

template<typename Scalar>
MPComControl<Scalar>::MPComControl(
  MotionModule* motionModule,
  const boost::shared_ptr<MPComControlConfig>& config) :
  BalanceModule<Scalar>(motionModule, config, "MPComControl")
{
}

template<typename Scalar>
bool MPComControl<Scalar>::initiate()
{
  LOG_INFO("MPComControl.initiate()")
  Matrix<Scalar, Dynamic, 1> jointsToReach;
  if (getBehaviorCast()->supportLeg == LinkChains::lLeg) {
    jointsToReach = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[0][0],
      sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
  } else {
    jointsToReach = Matrix<Scalar, Dynamic, 1>::Map(
      &balanceDefs[1][0],
      sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
  }
  cout << "jointsToReach: " << jointsToReach * MathsUtils::RAD_TO_DEG << endl;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  this->jointsI = this->kM->getJointPositions();
  this->jointsDelta = jointsToReach - this->jointsI;
  Matrix<bool, Dynamic, 1> activeJoints = this->jointsDelta.cwiseAbs().array() > Angle::DEG_1;
  if (!activeJoints.any()) { ///< Posture already reached
    return false;
  } else {
    auto timeStep = this->cycleTime;
    vector<Matrix<Scalar, Dynamic, 1> > joints;
    vector<Scalar> times;
    while (timeStep <= this->getBehaviorCast()->timeToReachB) {
      auto timeParam = timeStep / this->getBehaviorCast()->timeToReachB;
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
  if (!activeJoints.any()) { ///< Posture already reached
    return false;
  } else {
    return true;
  }
  #endif
}

template<typename Scalar>
void MPComControl<Scalar>::update()
{
  auto& timeToReachB = getBehaviorCast()->timeToReachB;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->runTime > timeToReachB)
    finish();
  #else
  if (this->runTime > timeToReachB) {
    finish();
  } else {
    auto step = this->runTime / timeToReachB;
    this->setJointCmds(this->interpolate(step));
  }
  #endif
}

template<typename Scalar>
void MPComControl<Scalar>::finish()
{
  LOG_INFO("MPComControl.finish() called...")
  this->inBehavior = false;
}

template<typename Scalar>
MPComControlConfigPtr MPComControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <MPComControlConfig> (this->config);
}

template class MPComControl<MType>;
