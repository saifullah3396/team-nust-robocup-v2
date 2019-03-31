/**
 * @file MotionModule/src/MotionBehavior.cpp
 *
 * This file implements the class MotionBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include <boost/filesystem.hpp>
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionGenerator.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/MTypeHeader.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "Utils/include/HardwareIds.h"

template <typename Scalar>
MotionBehavior<Scalar>::MotionBehavior(
  MotionModule* motionModule,
  const boost::shared_ptr<MBConfig>& config,
  const string& name) :
  Behavior(config, name),
  MemoryBase(motionModule),
  motionModule(motionModule)
{
  kM = motionModule->getKinematicsModule();
  mG = motionModule->getMotionGenerator();
  cycleTime = motionModule->getPeriodMinMS() / 1000.f;
}

template <typename Scalar>
JsonLoggerPtr MotionBehavior<Scalar>::makeLogger()
{
  auto logger =
    boost::shared_ptr<MotionLogger<Scalar> >(new MotionLogger<Scalar>(
      motionModule, this->logsDirPath + "/" + this->name + ".json"));
  mG->setMotionLogger(logger);
  return logger;
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionBehavior<Scalar>::killAllMotions()
{
  mG->killAllMotions();
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionBehavior<Scalar>::stopMove()
{
  mG->stopMove();
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
AL::ALValue MotionBehavior<Scalar>::getNaoqiTaskList()
{
  return mG->getNaoqiTaskList();
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
AL::ALValue MotionBehavior<Scalar>::getFootsteps()
#else
vector<vector<float> > MotionBehavior<Scalar>::getFootsteps()
#endif
{
  return mG->getFootsteps();
}
#endif

template <typename Scalar>
void MotionBehavior<Scalar>::openHand(const RobotHands& handIndex)
{
  mG->openHand(handIndex);
}

template <typename Scalar>
void MotionBehavior<Scalar>::closeHand(const RobotHands& handIndex)
{
  mG->closeHand(handIndex);
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
bool MotionBehavior<Scalar>::naoqiMoveIsActive()
{
  return mG->naoqiMoveIsActive();
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
void MotionBehavior<Scalar>::naoqiSetAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
#else
void MotionBehavior<Scalar>::naoqiSetAngles(
  const vector<string>& names,
  const vector<Scalar>& angles,
  const float& fractionMaxSpeed)
#endif
{
  mG->naoqiSetAngles(names, angles, fractionMaxSpeed);
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
void MotionBehavior<Scalar>::naoqiChangeAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
#else
void MotionBehavior<Scalar>::naoqiChangeAngles(
  const vector<string>& names,
  const vector<Scalar>& angles,
  const float& fractionMaxSpeed)
#endif
{
  mG->naoqiChangeAngles(names, angles, fractionMaxSpeed);
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionBehavior<Scalar>::naoqiMoveToward(
  const float& vx,
  const float& vy,
  const float& vt)
{
  mG->naoqiMoveToward(vx, vy, vt);
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
void MotionBehavior<Scalar>::naoqiSetFootsteps(
  const AL::ALValue& footName,
  const AL::ALValue& footSteps,
  const AL::ALValue& timeList,
  const bool& clearExisting)
#else
void MotionBehavior<Scalar>::naoqiSetFootsteps(
  const vector<string>& footName,
  const vector<vector<float> >& footSteps,
  const vector<string>& timeList,
  const bool& clearExisting)
#endif
{
  mG->naoqiSetFootsteps(footName, footSteps, timeList, true);
}
#endif

template <typename Scalar>
void MotionBehavior<Scalar>::setJointCmds(
  const Matrix<Scalar, Dynamic, 1>& jr)
{
  mG->setJointCmds(jr);
}

template <typename Scalar>
void MotionBehavior<Scalar>::addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task)
  { mG->addMotionTask(task); }

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
void MotionBehavior<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timeLists,
  const AL::ALValue& positionLists,
  const bool& postCommand)
#else
void MotionBehavior<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const vector<vector<float> >& timeLists,
  const vector<vector<float> >& positionLists,
  const bool& postCommand)
#endif
{
  if (this->config->logData)
    mG->naoqiJointInterpolation(ids, timeLists, positionLists, postCommand, MOTION_LOGGER);
  else
    mG->naoqiJointInterpolation(ids, timeLists, positionLists, postCommand);
}

template <typename Scalar>
 void MotionBehavior<Scalar>::naoqiJointInterpolation(
  const vector<Matrix<Scalar, Dynamic, 1> >& joints,
  const vector<Scalar>& times,
  const Matrix<bool, Dynamic, 1>& activeJoints,
  const bool& postCommand)
{
  if (this->config->logData)
    mG->naoqiJointInterpolation(joints, times, activeJoints, postCommand, MOTION_LOGGER);
  else
    mG->naoqiJointInterpolation(joints, times, activeJoints, postCommand);
}
#endif

template class MotionBehavior<float>;
