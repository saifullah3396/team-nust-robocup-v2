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
#include "MotionModule/include/HandsRequest.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/Constants.h"

template <typename Scalar>
MotionGenerator<Scalar>::MotionGenerator(
  MotionModule* motionModule) :
  MemoryBase(motionModule),
  motionModule(motionModule),
  cmdsReceived(false)
{
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  motionProxy = motionModule->getSharedMotionProxy();
  #endif
  kM = motionModule->getKinematicsModule();
  cycleTime = motionModule->getPeriodMinMS() / 1000.f;
  jointCmds.resize(toUType(Joints::count));
  additionalJointOffsets.resize(toUType(Joints::count));
  additionalJointOffsets.setZero();
  jointCmds = kM->getJointPositions();
}

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
    jointCmds += additionalJointOffsets;
    //cout << "JointCmds lhipyawpitch:" << jointCmds[toUType(Joints::lHipYawPitch)] * 180 / 3.14 << endl;
    //cout << "JointCmds rhipyawpitch:" << jointCmds[toUType(Joints::rHipYawPitch)] * 180 / 3.14 << endl;
    motionTasks.clear();
  }

  //cout << "Setting joint cmd:" << jointCmds.transpose() << endl;
  auto jr = boost::shared_ptr<JointRequest>(new JointRequest());
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

template <typename Scalar>
void MotionGenerator<Scalar>::reset()
{
  additionalJointOffsets.setZero();
}

template <typename Scalar>
void MotionGenerator<Scalar>::setAdditionalJointOffset(const Joints& joint, const Scalar& offset) {
  this->additionalJointOffsets[toUType(joint)] = offset;
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::killAllMotions() {
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->killAll();
    #else
      motionProxy.call<void>("killAll");
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::stopMove() {
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->stopMove();
    #else
      motionProxy.call<void>("stopMove");
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
AL::ALValue MotionGenerator<Scalar>::getNaoqiTaskList() {
  try {
    #ifndef V6_CROSS_BUILD
      return motionProxy->getTaskList();
    #else
      return motionProxy.call<AL::ALValue>("getNaoqiTaskList");
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
    return AL::ALValue();
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
AL::ALValue MotionGenerator<Scalar>::getFootsteps()
#else
vector<vector<float> > MotionGenerator<Scalar>::getFootsteps()
#endif
{
  try {
    #ifndef V6_CROSS_BUILD
      return motionProxy->getFootSteps();
    #else
      return motionProxy.call<AL::ALValue>("getFootSteps");
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
    return AL::ALValue();
  }
}
#endif

template <typename Scalar>
void MotionGenerator<Scalar>::openHand(const RobotHands& handIndex) {
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    try {
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
    } catch (exception& e) {
      LOG_EXCEPTION(e.what());
    }
  #else
    auto hr = boost::shared_ptr<HandsRequest>(new HandsRequest());
    for (size_t i = 0; i < toUType(RobotHands::count); ++i) {
      hr->setValue(1.0, i);
    }
    BaseModule::publishModuleRequest(hr);
  #endif
}

template <typename Scalar>
void MotionGenerator<Scalar>::closeHand(const RobotHands& handIndex) {
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    try {
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
    } catch (exception& e) {
      LOG_EXCEPTION(e.what());
    }
#else
  auto hr = boost::shared_ptr<HandsRequest>(new HandsRequest());
  for (size_t i = 0; i < toUType(RobotHands::count); ++i) {
    hr->setValue(0.0, i);
  }
  BaseModule::publishModuleRequest(hr);
#endif
}

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
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
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->setAngles(names, angles, fractionMaxSpeed);
    #else
      motionProxy.call<void>("setAngles", names, angles, fractionMaxSpeed);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
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
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->changeAngles(names, angles, fractionMaxSpeed);
    #else
      motionProxy.call<void>("changeAngles", names, angles, fractionMaxSpeed);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
bool MotionGenerator<Scalar>::naoqiMoveIsActive() {
  try {
    #ifndef V6_CROSS_BUILD
      return motionProxy->moveIsActive();
    #else
      motionProxy.call<void>("moveIsActive");
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
    return false;
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
AL::ALValue MotionGenerator<Scalar>::getMoveConfig()
{
  try {
    #ifndef V6_CROSS_BUILD
      return motionProxy->getMoveConfig("Default");
    #else
      motionProxy.call<void>("getMoveConfig");
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
    return AL::ALValue();
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::setMoveConfig(const AL::ALValue& config)
{
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->setMotionConfig(config);
    #else
      motionProxy.call<void>("setMotionConfig", config);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
vector<float> MotionGenerator<Scalar>::getRobotPosition(const bool& useSensors)
{
  try {
    #ifndef V6_CROSS_BUILD
      return motionProxy->getRobotPosition(useSensors);
    #else
      return motionProxy.call<vector<float>>("getRobotPosition", useSensors);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
    return vector<float>({NAN, NAN, NAN});
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiMoveToward(const float& vx, const float& vy, const float& vtheta)
{
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->moveToward(vx, vy, vtheta);
    #else
      motionProxy.call<void>("moveToward", vx, vy, vtheta);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
void MotionGenerator<Scalar>::naoqiMoveTo(const float& x, const float& y, const float& t)
{
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->post.moveTo(x, y, t);
    #else
      motionProxy.async<void>("moveTo", x, y, t);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
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
  try {
    #ifndef V6_CROSS_BUILD
      motionProxy->setFootSteps(footName, footSteps, timeList, clearExisting);
    #else
      motionProxy.call<void>("setFootSteps", footName, footSteps, timeList, clearExisting);
    #endif
  } catch (exception& e) {
    LOG_EXCEPTION(e.what());
  }
}
#endif

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
template <typename Scalar>
#ifndef V6_CROSS_BUILD_REMOVED
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
  #ifndef V6_CROSS_BUILD_REMOVED
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
        motionProxy->killAll();
        motionProxy->post.angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy.call<void>("killAll");
        motionProxy.async<void>(
          "angleInterpolation", names, positionLists, timesList, true);

        /*auto tasks = this->getNaoqiTaskList();
        cout <<"tasls: "<< tasks << endl;
        while(tasks.getSize() == 0) {
          tasks = this->getNaoqiTaskList();
        }
        cout << "tasks: " << tasks << endl;*/

      #endif
    } else {
      #ifndef V6_CROSS_BUILD
        motionProxy->killAll();
        motionProxy->angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy->killAll();
        motionProxy.call<void>(
          "angleInterpolation", names, positionLists, timesList, true);
      #endif
    }
  } catch (exception &e) {
    LOG_EXCEPTION(e.what());
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
  #ifndef V6_CROSS_BUILD_REMOVED
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
        #ifndef V6_CROSS_BUILD_REMOVED
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
        motionProxy->killAll();
        motionProxy->post.angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy.call<void>("killAll");
        motionProxy.async<void>(
          "angleInterpolation", names, positionLists, timesList, true);

        /*auto tasks = this->getNaoqiTaskList();
        cout <<"tasls: "<< tasks << endl;
        while(tasks.getSize() == 0) {
          tasks = this->getNaoqiTaskList();
        }
        cout << "tasks: " << tasks << endl;*/

      #endif
    } else {
      #ifndef V6_CROSS_BUILD
        motionProxy->killAll();
        motionProxy->angleInterpolation(
          names, positionLists, timesList, true);
      #else
        motionProxy->killAll();
        motionProxy.call<void>(
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
  try {
    #ifndef V6_CROSS_BUILD_REMOVED
    AL::ALValue jointTimes;
    AL::ALValue jointPositions;
    jointTimes.clear();
    jointPositions.clear();
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
      AL::ALValue jointPosition;
      AL::ALValue jointTime;
      bool ignore = false;
      time = 0;
      for (size_t j = 0; j < targetJoints.size(); ++j) {
        if (targetJoints[j][i] == targetJoints[j][i]) {
          time += times[j];
          jointPosition.arrayPush(targetJoints[j][i] * M_PI / 180.f);
          jointTime.arrayPush(time);
        } else {
          ignore = true;
          break;
        }
      }
      if (!ignore) {
        jointIds.push_back(i);
        jointPositions.arrayPush(jointPosition);
        jointTimes.arrayPush(jointTime);
      }
    }
    naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
    return time;
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
    return 0.0;
  }
}
#endif

template class MotionGenerator<MType>;
