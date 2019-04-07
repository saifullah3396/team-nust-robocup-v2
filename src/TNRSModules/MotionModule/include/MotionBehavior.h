/**
 * @file MotionModule/include/MotionBehavior.h
 *
 * This file declares the class MotionBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#ifndef V6_CROSS_BUILD
#include <alvalue/alvalue.h>
#else
#include <qi/alvalue.h>
#include <qi/anyobject.hpp>
#endif
#include <boost/filesystem.hpp>
#include "BehaviorManager/include/Behavior.h"
#include "MotionModule/include/MotionGenerator.h"
#include "TNRSBase/include/MemoryBase.h"

template <typename Scalar>
class MotionTask;
class MotionModule;
template <typename Scalar>
class KinematicsModule;
template <typename Scalar>
class MotionGenerator;
#define MOTION_LOGGER static_pointer_cast<MotionLogger<MType> >(this->dataLogger)
enum class RobotHands : unsigned int;

/**
 * @class MotionBehavior
 * @brief A base class for all kinds of motion behaviors
 */
template <typename Scalar>
class MotionBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * @brief MotionBehavior Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  MotionBehavior(
    MotionModule* motionModule,
    const boost::shared_ptr<MBConfig>& config,
    const string& name = "MotionBehavior");

  /**
   * @brief ~MotionBehavior Destructor
   */
  virtual~MotionBehavior() {}

  /**
   * @brief Creates a JSON logger for this motion behavior.
   *   See Behavior::makeLogger()
   */
  virtual JsonLoggerPtr makeLogger() override;

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief killAllMotions Wrapper for MotionGenerator::killAllMotions()
   */
  void killAllMotions();
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief stopMove Wrapper for MotionGenerator::stopMove()
   */
  void stopMove();
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief getNaoqiTaskList Wrapper for MotionGenerator::getNaoqiTaskList()
   */
  AL::ALValue getNaoqiTaskList();
  #endif


  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief getFootsteps Wrapper for MotionGenerator::stopMove()
   */
    #ifndef V6_CROSS_BUILD_REMOVED
    AL::ALValue getFootsteps();
    #else
    vector<vector<float> > getFootsteps();
    #endif
  #endif

  /**
   * @brief openHand Wrapper for MotionGenerator::openHand()
   */
  void openHand(const RobotHands& handIndex);

  /**
   * @brief closeHand Wrapper for MotionGenerator::closeHand()
   */
  void closeHand(const RobotHands& handIndex);

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiMoveIsActive Wrapper for MotionGenerator::naoqiMoveIsActive()
   */
  bool naoqiMoveIsActive();
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiSetAngles Wrapper for MotionGenerator::naoqiSetAngles()
   */
  #ifndef V6_CROSS_BUILD_REMOVED
    void naoqiSetAngles(
      const AL::ALValue& names,
      const AL::ALValue& angles,
      const float& fractionMaxSpeed);
  #else
    void naoqiSetAngles(
      const vector<string>& names,
      const vector<Scalar>& angles,
      const float& fractionMaxSpeed);
  #endif
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiChangeAngles Wrapper for MotionGenerator::naoqiChangeAngles()
   */
  #ifndef V6_CROSS_BUILD_REMOVED
  void naoqiChangeAngles(
    const AL::ALValue& names,
    const AL::ALValue& angles,
    const float& fractionMaxSpeed);
  #else
    void naoqiChangeAngles(
      const vector<string>& names,
      const vector<Scalar>& angles,
      const float& fractionMaxSpeed);
  #endif
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiMoveToward Wrapper for MotionGenerator::naoqiMoveToward()
   */
  void naoqiMoveToward(
    const float& vx,
    const float& vy,
    const float& vt);
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiSetFootsteps Wrapper for MotionGenerator::naoqiSetFootsteps()
   */
  #ifndef V6_CROSS_BUILD_REMOVED
    void naoqiSetFootsteps(
      const AL::ALValue& footName,
      const AL::ALValue& footSteps,
      const AL::ALValue& timeList,
      const bool& clearExisting);
  #else
    void naoqiSetFootsteps(
      const vector<string>& footName,
      const vector<vector<float> >& footSteps,
      const vector<string>& timeList,
      const bool& clearExisting);
  #endif
  #endif

  /**
   * @brief setJointCmds Wrapper for MotionGenerator::setJointsCmds
   * @param jr Joints to be sent as a command
   */
  void setJointCmds(const Matrix<Scalar, Dynamic, 1>& jr);

  ///< Getters
  boost::shared_ptr<KinematicsModule<Scalar> > getKinematicsModule()
    { return kM; }

protected:
  /**
   * Takes a 2D vector of key frames of size [times][joints], maps
   * it into eigen based vectors and calls the overloaded
   * runKeyFrameMotion();
   *
   * @param keyFrames: A 2D-vector consisting of times and joint values.
   *   This vector is of size (1 + numJoints) x (numKeyFrames). A single
   *   key frame must be defined as... (time, q1, q2, ... qN).
   */
  template<typename type, std::size_t times, std::size_t joints>
  Scalar runKeyFrameMotion(
    const type (&keyFrames)[times][joints])
  {
    vector<Matrix<Scalar, Dynamic, 1> > targetJoints(times);
    Matrix<Scalar, Dynamic, 1> targetTimes(times);
    for (int i = 0; i < times; ++i) {
      targetJoints[i] = Matrix<Scalar, Dynamic, 1>::Map(
        &keyFrames[i][0] + 1,
        (sizeof(keyFrames[i]) - sizeof(keyFrames[i][0])) / sizeof(keyFrames[i][0]));
      targetTimes[i] = keyFrames[i][0];
    }
    return mG->runKeyFrameMotion(targetJoints, targetTimes);
  }

  /**
   * Wrapper for MotionGenerator::addMotionTask()
   */
  void addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task);

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * Wrapper for MotionGenerator::naoqiJointInterpolation()
   */
  #ifndef V6_CROSS_BUILD_REMOVED
  void naoqiJointInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timeLists,
    const AL::ALValue& positionLists,
    const bool& postCommand);
  #else
  void naoqiJointInterpolation(
    const vector<unsigned>& ids,
    const vector<vector<float> >& timeLists,
    const vector<vector<float> >& positionLists,
    const bool& postCommand);
  #endif

  /**
   * Wrapper for MotionGenerator::naoqiJointInterpolation()
   */
  void naoqiJointInterpolation(
    const vector<Matrix<Scalar, Dynamic, 1> >& joints,
    const vector<Scalar>& times,
    const Matrix<bool, Dynamic, 1>& activeJoints,
    const bool& postCommand);
  #endif

  ///< Kinematics module object
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

  ///< Motion generator module object
  boost::shared_ptr<MotionGenerator<Scalar> > mG;

  ///< Base MotionModule object pointer
  MotionModule* motionModule;
};
