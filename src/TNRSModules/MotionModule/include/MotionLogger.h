/**
 * @file Utils/include/MotionLogger.h
 *
 * This file defines the class MotionLogger
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#ifndef V6_CROSS_BUILD
#include <alvalue/alvalue.h>
#else
#include <qi/alvalue.h>
#endif
#include <chrono>
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/JsonLogger.h"

using namespace std::chrono;

class MotionModule;
template <typename Scalar>
class KinematicsModule;

/**
 * @class MotionLogger
 * @brief Class that provides a json based motion data logger
 */
template <typename Scalar>
class MotionLogger : public Utils::JsonLogger
{
public:

  /**
   * @brief MotionLogger constructor.
   *
   * @param path: Path to json file
   * @param root: Root of json object
   * @param reftime: Reference start time for motion data logs
   */
  MotionLogger(
    MotionModule* motionModule,
    const string& path,
    const Json::Value& root = Json::Value(),
    const high_resolution_clock::time_point& refTime = high_resolution_clock::now());

  /**
   * @brief recordJointCmds Logs the joint commands sent by naoqi-based methods
   * @param cmds Joint commands
   * @param time Joint command times
   * @param ids Ids of joints updated
   */
  #ifndef V6_CROSS_BUILD_REMOVED
  void recordJointCmds(
     const AL::ALValue& cmds, const AL::ALValue& time, const vector<unsigned>& ids);
  #else
  void recordJointCmds(
    const vector<vector<Scalar> >& cmds,
    const vector<vector<Scalar> >& time,
    const vector<unsigned>& ids);
  #endif

  /**
   * @brief recordJointCmds Logs the joint commands sent by naoqi-based methods
   * @param cmds Joint commands
   * @param time Joint command times
   * @param activeJoints Active joints
   */
  #ifndef V6_CROSS_BUILD_REMOVED
  void recordJointCmds(
    const AL::ALValue& cmds,
    const AL::ALValue& time,
    const Matrix<bool, Dynamic, 1> activeJoints);
  #else
  void recordJointCmds(
    const vector<vector<Scalar> >& cmds,
    const vector<vector<Scalar> >& time,
    const Matrix<bool, Dynamic, 1> activeJoints);
  #endif

  /**
   * @brief logJointStates Logs current state of the joints
   * @param time Time of logging
   */
  void logJointStates(const Scalar& time);

  //! Setters
  void setRefTime(const high_resolution_clock::time_point& refTime)
    { this->refTime = refTime; }

private:
  //! Reference time from which all other readings are recorded
  high_resolution_clock::time_point refTime;

  //! Kinematics module object
  boost::shared_ptr<KinematicsModule<Scalar> > kM;
};
typedef boost::shared_ptr<MotionLogger<MType> > MotionLoggerPtr;
