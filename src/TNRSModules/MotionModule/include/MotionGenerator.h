/**
 * @file MotionModule/include/MotionGenerator.h
 *
 * This file declares the class MotionGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#ifndef V6_CROSS_BUILD
#include <alvalue/alvalue.h>
#else
#include <qi/alvalue.h>
#endif
#include <boost/filesystem.hpp>
#include "BehaviorManager/include/Behavior.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/MathsUtils.h"

///< Forward declaration
template <typename Scalar>
class MotionTask;

/**
 * @class MotionGenerator
 * @brief A class that recieves tasks from several motion modules and
 *   generates feasible joint requests
 */
template <typename Scalar>
class MotionGenerator : public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   */
  MotionGenerator(MotionModule* motionModule) :
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
    jointCmds = kM->getJointPositions();
  }

  /**
   * Destructor
   */
  virtual
  ~MotionGenerator()
  {
  }

  /**
   * Transforms requested tasks to joint requests and sends it to dcm
   */
  void update();

  /**
   * Takes a vector of joint positions to be called at given times
   *
   * @param joints: Required joint positions at given times
   * @param times: Times vector
   *
   * @return The cumulative time of this keyframe motion
   */
  Scalar runKeyFrameMotion(
    const vector<Matrix<Scalar, Dynamic, 1> >& joints, const Matrix<Scalar, Dynamic, 1>& times);

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief naoqiJointInterpolation Interpolates joints using naoqi joint interpolation
   *
   * @param ids: Ids of all the joints under consideration
   * @param timesList: Times for interpolation of each joint
   * @param positionLists: List of joint angular positions
   * @param postCommand: Whether this command is to be run in a
   *   separate thread or as a blocking call
   * @param Pointer to logger if present
   */
  #ifndef V6_CROSS_BUILD_REMOVED
  void naoqiJointInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timesList,
    const AL::ALValue& positionLists,
    const bool& postCommand,
    const MotionLoggerPtr& logger = MotionLoggerPtr());
  #else
  void naoqiJointInterpolation(
    const vector<unsigned>& ids,
    const vector<vector<float> >& timesList,
    const vector<vector<float> >& positionLists,
    const bool& postCommand,
    const MotionLoggerPtr& logger = MotionLoggerPtr());
  #endif
  /**
   * @brief naoqiJointInterpolation Interpolates joints using naoqi joint interpolation
   * @param joints Joint commands
   * @param times Times for joint commands
   * @param activeJoints Active joints
   * @param postCommand: Whether this command is to be run in a
   *   separate thread or as a blocking call
   * @param Pointer to logger if present
   */
  void naoqiJointInterpolation(
    const vector<Matrix<Scalar, Dynamic, 1> >& joints,
    const vector<Scalar>& times,
    const Matrix<bool, Dynamic, 1>& activeJoints,
    const bool& postCommand,
    const MotionLoggerPtr& logger = MotionLoggerPtr());
  #endif

  /**
   * Stops all naoqi generated motions
   */
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  void killAllMotions();
  #endif

  /**
   * Stops all naoqi generated movement
   */
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  void stopMove();
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * @brief getNaoqiTaskList Gets the list of currently running tasks in a thread
   */
  AL::ALValue getNaoqiTaskList();
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    /**
     * Uses naoqi api to get the list of current footsteps in queue
     */
    #ifndef V6_CROSS_BUILD_REMOVED
    AL::ALValue getFootsteps();
    #else
    vector<vector<float> > getFootsteps();
    #endif
  #endif

  /**
   * Uses naoqi api to open the given robot hand
   *
   * @handIndex: Hand index defined in Utils/Hardwareids.h
   */
  void openHand(const RobotHands& handIndex);

  /**
   * Uses naoqi api to close the given robot hand
   *
   * @handIndex: Hand index defined in Utils/Hardwareids.h
   */
  void closeHand(const RobotHands& handIndex);

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * Returns true if naoqi based walk is active
   */
  bool naoqiMoveIsActive();
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * Uses naoqi setAngles() to set the desired angles for given joints
   *
   * @param names: Joint names as defined in naoqi
   * @param angles: Joint angles as requested
   * @param fractionMaxSpeed: Maximum speed limit
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
   * Uses naoqi changeAngles() to set the desired angles for given joints
   *
   * @param names: Joint names as defined in naoqi
   * @param angles: Joint angles as requested
   * @param fractionMaxSpeed: Maximum speed limit
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
   * @brief naoqiMoveToward Moves the robot with desired velocity
   * @param vx Normalized velocity in x
   * @param vy Normalized velocity in y
   * @param vtheta Normalized angular velocity
   */
  void naoqiMoveToward(const float& vx, const float& vy, const float& vtheta);
  #endif

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * Uses naoqi setFootsteps() to set the required footsteps
   *
   * @param footNames: A vector of names for required feet
   * @param footSteps: Corresponding positions of those feet
   * @param timeList: Times at which feet are to be placed
   * @param clearExisting: Whether to clear existing commanded footsteps
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
   * Adds the given task to motion tasks vector if it is not conflicting with other tasks
   *
   * @param task: The desired task
   */
  void addMotionTask(const boost::shared_ptr<MotionTask<Scalar> >& task);

  /**
   * Returns the kinematics module
   *
   * @return KinematicsModulePtr
   */
  boost::shared_ptr<KinematicsModule<Scalar> > getKinematicsModule()
    { return kM; }

  /**
   * @brief setJointCmds
   * @param jr
   */
  void setJointCmds(const Matrix<Scalar, Dynamic, 1>& jr) {
    this->jointCmds = jr;
    this->cmdsReceived = true;
  }

  /**
   * @brief setMotionLogger Sets the motion logger
   * @param motionLogger Logger
   */
  void setMotionLogger(const MotionLoggerPtr& motionLogger) {
    this->motionLogger = motionLogger;
  }

private:
  /**
   * Validates the given motion task
   *
   * @param task: The validated task
   *
   * @return returns true if the given task is not conflicting with tasks already present
   */
  bool validateTask(const boost::shared_ptr<MotionTask<Scalar> >& task);

  ///< Cycle time of this motion behavior
  Scalar cycleTime;

  ///< Kinematics module object
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    ///< NaoQi's motion proxy
    #ifndef V6_CROSS_BUILD
    ALMotionProxyPtr motionProxy;
    #else
    qi::AnyObject motionProxy;
    #endif
  #endif

  ///< Base MotionModule object pointer
  MotionModule* motionModule;

  ///< Tasks vector containing all the tasks recieved from several motion modules.
  ///< All the tasks are solved together to give a resultant joint motion for the
  ///< given motion cycle.
  vector<boost::shared_ptr<MotionTask<Scalar> > > motionTasks;

  ///< Joint velocity commands for joint estimator
  Matrix<Scalar,  Dynamic, 1> jointCmds;

  ///< Commands are recieved in this cycle
  bool cmdsReceived;

  ///< A pointer to the logger associated with running behavior
  MotionLoggerPtr motionLogger;
};
