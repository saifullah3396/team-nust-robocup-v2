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
void MotionGenerator<Scalar>::killAllMotions() {
  #ifndef V6_CROSS_BUILD
    motionProxy->killAll();
  #else
    motionProxy.call<void>("killAll");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::stopMove() {
  #ifndef V6_CROSS_BUILD
    motionProxy->stopMove();
  #else
    motionProxy.call<void>("stopMove");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
vector<vector<string> > MotionGenerator<Scalar>::getNaoqiTaskList() {
  #ifndef V6_CROSS_BUILD
    auto value = motionProxy->getTaskList();
    auto size = value.getSize();
    vector<vector<string> > tasks;
    if (size != 0) {
      tasks.resize(size);
      for (int i = 0; i < size; ++i) {
        value.ToStringArray(tasks[i]);
      }
    }
    return tasks;
  #else
    return motionProxy.call<vector<vector<string>>>("getNaoqiTaskList");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD
AL::ALValue MotionGenerator<Scalar>::getFootsteps()
#else
vector<vector<float> > MotionGenerator<Scalar>::getFootsteps()
#endif
{
  #ifndef V6_CROSS_BUILD
    return motionProxy->getFootSteps();
  #else
    return vector<vector<float>>();//motionProxy.call<void>("getFootSteps");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::openHand(const RobotHands& handIndex) {
  #ifndef V6_CROSS_BUILD
    if (handIndex == RobotHands::lHand)
      motionProxy->post.openHand("LHand");
    else if (handIndex == RobotHands::rHand)
      motionProxy->post.openHand("RHand");
  #else
    if (handIndex == RobotHands::lHand)
      motionProxy.async<void>("openHand", "LHand");
    else if (handIndex == RobotHands::rHand)
      motionProxy.async<void>("openHand", "RHand");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::closeHand(const RobotHands& handIndex) {
  #ifndef V6_CROSS_BUILD
    if (handIndex == RobotHands::lHand)
      motionProxy->post.closeHand("LHand");
    else if (handIndex == RobotHands::rHand)
      motionProxy->post.closeHand("RHand");
  #else
    if (handIndex == RobotHands::lHand)
      motionProxy.async<void>("closeHand", "LHand");
    else if (handIndex == RobotHands::rHand)
      motionProxy.async<void>("closeHand", "RHand");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD
void MotionGenerator<Scalar>::naoqiSetAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
#else
void MotionGenerator<Scalar>::naoqiSetAngles(
  const vector<string>& names,
  const vector<Scalar>& angles,
  const float& fractionMaxSpeed)
#endif
{
  #ifndef V6_CROSS_BUILD
    motionProxy->setAngles(names, angles, fractionMaxSpeed);
  #else
    motionProxy.call<void>("setAngles", names, angles, fractionMaxSpeed);
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD
void MotionGenerator<Scalar>::naoqiChangeAngles(
  const AL::ALValue& names,
  const AL::ALValue& angles,
  const float& fractionMaxSpeed)
#else
void MotionGenerator<Scalar>::naoqiChangeAngles(
  const vector<string>& names,
  const vector<Scalar>& angles,
  const float& fractionMaxSpeed)
#endif
{
  #ifndef V6_CROSS_BUILD
    motionProxy->changeAngles(names, angles, fractionMaxSpeed);
  #else
    motionProxy.call<void>("changeAngles", names, angles, fractionMaxSpeed);
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
bool MotionGenerator<Scalar>::naoqiMoveIsActive() {
  #ifndef V6_CROSS_BUILD
    return motionProxy->moveIsActive();
  #else
    motionProxy.call<void>("moveIsActive");
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiMoveToward(const float& vx, const float& vy, const float& vtheta)
{
  #ifndef V6_CROSS_BUILD
    motionProxy->moveToward(vx, vy, vtheta);
  #else
    motionProxy.call<void>("moveToward", vx, vy, vtheta);
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD
void MotionGenerator<Scalar>::naoqiSetFootsteps(
  const AL::ALValue& footName,
  const AL::ALValue& footSteps,
  const AL::ALValue& timeList,
  const bool& clearExisting)
#else
void MotionGenerator<Scalar>::naoqiSetFootsteps(
  const vector<string>& footName,
  const vector<vector<float> >& footSteps,
  const vector<string>& timeList,
  const bool& clearExisting)
#endif
{
  #ifndef V6_CROSS_BUILD
    motionProxy->setFootSteps(footName, footSteps, timeList, clearExisting);
  #else
    motionProxy.call<void>("setFootSteps", footName, footSteps, timeList, clearExisting);
  #endif
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD
void MotionGenerator<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timesList,
  const AL::ALValue& positionLists,
  const bool& postCommand,
  const MotionLoggerPtr& logger)
#else
void MotionGenerator<Scalar>::naoqiJointInterpolation(
  const vector<unsigned>& ids,
  const vector<vector<float> >& timesList,
  const vector<vector<float> >& positionLists,
  const bool& postCommand,
  const MotionLoggerPtr& logger)
#endif
{
  #ifndef V6_CROSS_BUILD
    ASSERT(
      ids.size() == timesList.getSize() &&
      timesList.getSize() == positionLists.getSize()
    );
    AL::ALValue names;
    names.clear();
    names.arraySetSize(ids.size());
  #else
    ASSERT(
      ids.size() == timesList.size() &&
      timesList.getSize() == positionLists.size()
    );
    vector<string> names;
    names.clear();
    names.resize(ids.size());
  #endif
  for (int i = 0; i < ids.size(); ++i)
    names[i] = Constants::jointNames[ids[i]];

  if (logger)
    logger->recordJointCmds(positionLists, timesList, ids);

  try {
    if (postCommand) {
      #ifndef V6_CROSS_BUILD
        motionProxy->post.angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy.call<void>(
          "angleInterpolation", names, positionLists, timesList, true);
      #endif
    } else {
      #ifndef V6_CROSS_BUILD
        motionProxy->angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy.async<void>(
          "angleInterpolation", names, positionLists, timesList, true);
      #endif
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
  #ifndef V6_CROSS_BUILD
  AL::ALValue names;
  AL::ALValue positionLists;
  AL::ALValue timesList;
  names.arraySetSize(activeJoints.count());
  positionLists.arraySetSize(activeJoints.count());
  timesList.arraySetSize(activeJoints.count());
  #else
  vector<string> names;
  vector<vector<Scalar> > positionLists;
  vector<vector<Scalar> > timesList;
  names.resize(activeJoints.count());
  positionLists.resize(activeJoints.count());
  timesList.resize(activeJoints.count());
  #endif
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
        #ifndef V6_CROSS_BUILD
        positionLists[k].arrayPush(joints[i][j]);
        timesList[k].arrayPush(times[i]);
        #else
        positionLists[k].push_back(joints[i][j]);
        timesList[k].push_back(times[i]);
        #endif
        ++k;
      }
    }
  }

  if (logger)
    logger->recordJointCmds(positionLists, timesList, activeJoints);

  try {
    if (postCommand) {
      #ifndef V6_CROSS_BUILD
        motionProxy->post.angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy.call<void>(
          "angleInterpolation", names, positionLists, timesList, true);
      #endif
    } else {
      #ifndef V6_CROSS_BUILD
        motionProxy->angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy.async<void>(
          "angleInterpolation", names, positionLists, timesList, true);
      #endif
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
  #ifndef V6_CROSS_BUILD
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(toUType(Joints::count));
  jointPositions.arraySetSize(toUType(Joints::count));
  #else
  vector<vector<Scalar>> jointTimes;
  vector<vector<Scalar>> jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.resize(toUType(Joints::count));
  jointPositions.resize(toUType(Joints::count));
  #endif

  Scalar time = 0;
  vector<unsigned> jointIds;
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    jointIds.push_back(i);
    #ifndef V6_CROSS_BUILD
    jointPositions[i].arraySetSize(targetJoints.size());
    jointTimes[i].arraySetSize(targetJoints.size());
    #else
    jointPositions[i].resize(targetJoints.size());
    jointTimes[i].resize(targetJoints.size());
    #endif
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
