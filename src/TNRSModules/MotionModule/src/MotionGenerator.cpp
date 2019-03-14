/**
 * @file MotionModule/src/MotionGenerator.cpp
 *
 * This file implements the class MotionGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "MotionModule/include/MotionGenerator.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/JointRequest.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/Constants.h"

template <typename Scalar>
void MotionGenerator<Scalar>::update()
{
  #ifndef NAOQI_MOTION_PROXY_AVAILABLE
  if (!cmdsReceived && motionTasks.empty()) {
    this->kM->setJointPositionCmd(jointCmds); // repeat last sent cmd
    if (motionLogger)
      motionLogger->logJointStates(motionModule->getModuleTime());
    return;
  }
  if (!motionTasks.empty()) {
    this->kM->setStateFromTo(JointStateType::actual, JointStateType::sim);
    jointCmds = this->kM->solveTasksIK(motionTasks, 1);
    motionTasks.clear();
  }
  auto jr = boost::make_shared<JointRequest>();
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    jr->setValue(jointCmds[i], i);
  }
  this->kM->setJointPositionCmd(jointCmds);
  if (motionLogger)
    motionLogger->logJointStates(motionModule->getModuleTime());
  BaseModule::publishModuleRequest(jr);
  this->cmdsReceived = false;
  #else
  if (motionLogger)
    motionLogger->logJointStates(motionModule->getModuleTime());
  #endif
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiSetAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
{
  motionProxy->setAngles(names, angles, fractionMaxSpeed);
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiChangeAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
{
  motionProxy->changeAngles(names, angles, fractionMaxSpeed);
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
bool MotionGenerator<Scalar>::naoqiMoveIsActive() {
  return motionProxy->moveIsActive();
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiMoveToward(const float& vx, const float& vy, const float& vtheta)
{
  motionProxy->moveToward(vx, vy, vtheta);
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiSetFootsteps(
  const AL::ALValue& footName, 
  const AL::ALValue& footSteps, 
  const AL::ALValue& timeList, 
  const bool& clearExisting) 
{
  motionProxy->setFootSteps(footName, footSteps, timeList, clearExisting);
}
#endif  

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timeLists,
  const AL::ALValue& positionLists,
  const bool& postCommand,
  const MotionLoggerPtr& logger)
{
  ASSERT(
    ids.size() == timeLists.getSize() &&
    timeLists.getSize() == positionLists.getSize()
  );
  AL::ALValue names;
  names.clear();
  names.arraySetSize(ids.size());
  for (int i = 0; i < ids.size(); ++i)
    names[i] = Constants::jointNames[ids[i]];

  if (logger)
    logger->recordJointCmds(positionLists, timeLists, ids);

  try {
    if (postCommand) {
      motionProxy->post.angleInterpolation(
        names,
        positionLists,
        timeLists,
        true
      );
    } else {
      motionProxy->angleInterpolation(
        names,
        positionLists,
        timeLists,
        true
      );
    }
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}

template <typename Scalar>
void MotionGenerator<Scalar>::naoqiJointInterpolation(
  const vector<Matrix<Scalar, Dynamic, 1> >& joints,
  const vector<Scalar>& times,
  const Matrix<bool, Dynamic, 1>& activeJoints,
  const bool& postCommand,
  const MotionLoggerPtr& logger)
{
  AL::ALValue names;
  AL::ALValue positionLists;
  AL::ALValue timesList;
  names.arraySetSize(activeJoints.count());
  positionLists.arraySetSize(activeJoints.count());
  timesList.arraySetSize(activeJoints.count());
  for (size_t i = 0, k = 0; i < toUType(Joints::count); ++i) {
    if (activeJoints[i]) {
      names[k] = Constants::jointNames[i];
      ++k;
    }
  }
  for (size_t i = 0; i < joints.size(); ++i) {
    int k = 0;
    for (size_t j = 0; j < joints[i].size(); ++j) {
      if (activeJoints[j]) {
        positionLists[k].arrayPush(joints[i][j]);
        timesList[k].arrayPush(times[i]);
        ++k;
      }
    }
  }

  if (logger)
    logger->recordJointCmds(positionLists, timesList, activeJoints);

  try {
    if (postCommand) {
      motionProxy->post.angleInterpolation(
        names,
        positionLists,
        timesList,
        true
      );
    } else {
      motionProxy->angleInterpolation(
        names,
        positionLists,
        timesList,
        true
      );
    }
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}

#endif

template <typename Scalar>
void MotionGenerator<Scalar>::addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task)
{
  if (validateTask(task))
    motionTasks.push_back(task);
}

template <typename Scalar>
bool MotionGenerator<Scalar>::validateTask(const boost::shared_ptr<MotionTask<Scalar> >& task)
{
  /*if (motionTasks.empty()) {
    return true;
  } else {
    for (size_t i = 0; i < motionTasks.size(); ++i) {
      if (motionTasks[i]->checkConflict(task)) {
        return false;
      }
    }
  }*/
  return true;
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
Scalar MotionGenerator<Scalar>::runKeyFrameMotion(
  const vector<Matrix<Scalar, Dynamic, 1>>& targetJoints, const Matrix<Scalar, Dynamic, 1>& times)
{
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(toUType(Joints::count));
  jointPositions.arraySetSize(toUType(Joints::count));

  Scalar time = 0;
  vector<unsigned> jointIds;
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    jointIds.push_back(i);
    jointPositions[i].arraySetSize(targetJoints.size());
    jointTimes[i].arraySetSize(targetJoints.size());
    time = 0;
    for (size_t j = 0; j < targetJoints.size(); ++j) {
      time += times[j];
      jointPositions[i][j] = targetJoints[j][i] * M_PI / 180.f;
      jointTimes[i][j] = time;
    }
  }
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  return time;
}
#endif

template class MotionGenerator<MType>;
