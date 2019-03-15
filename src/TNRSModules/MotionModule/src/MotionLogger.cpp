/**
 * @file MotionModule/src/MotionLogger.cpp
 *
 * This file implements the class MotionLogger
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#ifndef V6_CROSS_BUILD
#include <alvalue/alvalue.h>
#endif
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "Utils/include/Constants.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/JsonUtils.h"

template <typename Scalar>
MotionLogger<Scalar>::MotionLogger(
  MotionModule* motionModule,
  const string& path,
  const Json::Value& root,
  const high_resolution_clock::time_point& refTime) :
  kM(motionModule->getKinematicsModule()),
  Utils::JsonLogger(path, root),
  refTime(refTime)
{
}

template <typename Scalar>
#ifdef V6_CROSS_BUILD
void MotionLogger<Scalar>::recordJointCmds(
  const AL::ALValue& cmds, const AL::ALValue& time, const vector<unsigned>& ids)
#else
void recordJointCmds(
   const vector<Scalar>& cmds, const vector<Scalar>& time, const vector<unsigned>& ids)
#endif
{
  #ifndef V6_CROSS_BUILD
  ASSERT(cmds.getSize() != 0);
  #else
  ASSERT(cmds.size() != 0);
  #endif
  high_resolution_clock::time_point timeNow = high_resolution_clock::now();
  double timeStart = (duration<double>(timeNow - refTime)).count();  
  vector<int> jointIndices(toUType(Joints::count), -1);
  for (size_t i = 0; i < ids.size(); ++i) {
    jointIndices[ids[i]] = i;
  }
  #ifndef V6_CROSS_BUILD
  for (size_t i = 0; i < time[0].getSize(); ++i) {
  #else
  for (size_t i = 0; i < time[0].size(); ++i) {
  #endif
    JSON_APPEND(
      root["jointCommands"], 
      "time", 
      double(time[0][i]) + timeStart
    );
    for (size_t j = 0; j < jointIndices.size(); ++j) {
      if (jointIndices[j] < 0) {
        JSON_APPEND(
          root["jointCommands"], 
          Constants::jointNames[j] + "Cmd",
          "NAN"
        );
      } else {
        JSON_APPEND(
          root["jointCommands"], 
          Constants::jointNames[j] + "Cmd",
          (MType)cmds[jointIndices[j]][i]
        );
      }
    }
  }
}

template <typename Scalar>
#ifdef V6_CROSS_BUILD
void MotionLogger<Scalar>::recordJointCmds(
  const AL::ALValue& cmds,
  const AL::ALValue& time,
  const Matrix<bool, Dynamic, 1> activeJoints)
#else
void recordJointCmds(
  const vector<Scalar>& cmds,
  const vector<Scalar>& time,
  const Matrix<bool, Dynamic, 1> activeJoints)
#endif
{
  #ifndef V6_CROSS_BUILD
  ASSERT(cmds.getSize() != 0);
  #else
  ASSERT(cmds.size() != 0);
  #endif
  high_resolution_clock::time_point timeNow = high_resolution_clock::now();
  double timeStart = (duration<double>(timeNow - refTime)).count();
  #ifndef V6_CROSS_BUILD
  for (size_t i = 0; i < time[0].getSize(); ++i) {
  #else
  for (size_t i = 0; i < time[0].size(); ++i) {
  #endif
    JSON_APPEND(
      root["jointCommands"],
      "time",
      double(time[0][i]) + timeStart
    );
    int k = 0;
    for (size_t j = 0; j < toUType(Joints::count); ++j) {
      if (activeJoints[j]) {
        JSON_APPEND(
          root["jointCommands"],
          Constants::jointNames[j] + "Cmd",
          (MType)cmds[k][i]
        );
        k++;
      } else {
        JSON_APPEND(
          root["jointCommands"],
          Constants::jointNames[j] + "Cmd",
          "NAN"
        );
      }
    }
  }
}

template <typename Scalar>
void MotionLogger<Scalar>::logJointStates(const Scalar& time)
{
  auto joints = this->kM->getJoints();
  for (const auto& joint : joints)
    joint->logJointState(this->root, time);
}  

template class MotionLogger<MType>;
