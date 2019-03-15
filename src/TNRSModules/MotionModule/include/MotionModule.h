/**
 * @file MotionModule/include/MotionModule.h
 *
 * This file declares the class MotionModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <map>
#ifndef V6_CROSS_BUILD
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#endif
#include "MotionModule/include/MTypeHeader.h"
#include "TNRSBase/include/BaseIncludes.h"

//! Forward declarations
class SensorLayer;
typedef boost::shared_ptr<SensorLayer> SensorLayerPtr;
class ActuatorLayer;
typedef boost::shared_ptr<ActuatorLayer> ActuatorLayerPtr;
template <typename Scalar> class MBManager;
typedef boost::shared_ptr<MBManager<MType> > MBManagerPtr;
template <typename Scalar> class FallDetector;
typedef boost::shared_ptr<FallDetector<MType> > FallDetectorPtr;
template <typename Scalar> class KinematicsModule;
typedef boost::shared_ptr<KinematicsModule<MType> > KinematicsModulePtr;
template <typename Scalar> class MotionGenerator;
typedef boost::shared_ptr<MotionGenerator<MType> > MotionGeneratorPtr;
template <typename Scalar> class TrajectoryPlanner;
typedef boost::shared_ptr<TrajectoryPlanner<MType> > TrajectoryPlannerPtr;
#ifndef V6_CROSS_BUILD
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;
#endif
typedef map<unsigned, BehaviorInfo> BehaviorInfoMap;

/**
 * @class MotionModule
 * @brief A class for handling all activities related to the robot 
 *   kinematics, dynamics, motion.
 */
class MotionModule : public BaseModule
{
  DECLARE_INPUT_CONNECTOR(
    motionThreadPeriod,
    jointStiffnessSensors,
    touchSensors,
    ballInfo,
    goalInfo,
    robotPose2D,
    landmarksFound
  )
  DECLARE_OUTPUT_CONNECTOR(
    jointPositionSensors,
    inertialSensors,
    fsrSensors,
    nFootsteps,
    upperCamInFeet,
    lowerCamInFeet,
    lFootOnGround,
    rFootOnGround,
    postureState,
    robotFallen,
    kickTarget,
    robotInMotion,
    mBehaviorInfo,
    footOnGround,
    currentStepLeg
  )
public:
#ifdef NAOQI_MOTION_PROXY_AVAILABLE
  #ifndef V6_CROSS_BUILD
    /**
     * Constructor
     *
     * @param parent: parent: Pointer to parent module
     * @param memoryProxy: Pointer to NaoQi's memory proxy
     * @param dcmProxy: Pointer to NaoQi's DCM proxy
     * @param motionProxy: Pointer to NaoQi's motion proxy
     */
    MotionModule(
     void* parent,
     const ALMemoryProxyPtr& memoryProxy,
     const ALDCMProxyPtr& dcmProxy,
     const ALMotionProxyPtr& motionProxy);
  #else
    /**
     * Constructor
     *
     * @param parent: parent: Pointer to parent module
     * @param memoryProxy: Pointer to NaoQi's memory proxy
     * @param motionProxy: Pointer to NaoQi's motion proxy
     */
    MotionModule(
     void* parent,
     const qi::AnyObject& memoryProxy,
     const qi::AnyObject& motionProxy);
  #endif
#else
  #ifndef V6_CROSS_BUILD
    /**
     * Constructor
     *
     * @param parent: parent: Pointer to parent module
     * @param memoryProxy: Pointer to NaoQi's memory proxy
     * @param dcmProxy: Pointer to NaoQi's DCM proxy
     */
    MotionModule(
     void* parent,
     const ALMemoryProxyPtr& memoryProxy,
     const ALDCMProxyPtr& dcmProxy);
  #else
    /**
     * Constructor
     *
     * @param parent: parent: Pointer to parent module
     * @param memoryProxy: Pointer to NaoQi's memory proxy
     */
    MotionModule(
     void* parent,
     const qi::AnyObject& memoryProxy);
  #endif
#endif

  /**
   * Destructor
   */
  ~MotionModule();

  /**
   * Derived from BaseModule
   */
  void init();

  /**
   * Derived from BaseModule
   */
  void handleRequests();

  /**
   * Derived from BaseModule
   */
  void mainRoutine();

  /**
   * Derived from BaseModule
   */
  void initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void setThreadPeriod();

  /**
   * Gets the kinematics module
   *
   * @return KinematicsModulePtr
   */
  KinematicsModulePtr getKinematicsModule();

  /**
   * Gets the motion generator module
   *
   * @return MotionGeneratorPtr
   */
  MotionGeneratorPtr getMotionGenerator();

  /**
   * Gets the trajectory planner module
   *
   * @return TrajectoryPlannerPtr
   */
  TrajectoryPlannerPtr getTrajectoryPlanner();
  
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  /**
   * Gets the motion proxy module
   *
   * @return MotionProxyPtr
   */
    #ifndef V6_CROSS_BUILD
      ALMotionProxyPtr getSharedMotionProxy();
    #else
      qi::AnyObject getSharedMotionProxy();
    #endif
  #endif

private:
  /**
   * Sets up the motion sensors
   */
  void setupSensors();

  /**
   * Sets up the motion actuators
   */
  void setupActuators();

  /**
   * Updates sensor values from NaoQi ALMemory to our local
   * shared memory
   */
  void sensorsUpdate();

  /**
   * Sends the requested actuator commands to NaoQi DCM for execution
   */
  void actuatorsUpdate();

	//! Motion behaviors manager shared object
  map<unsigned, MBManagerPtr> mbManagers;

  //! Fall detector module object
  FallDetectorPtr fallDetector;

  //! Kinematics module object
  KinematicsModulePtr kinematicsModule;

  //! Motion generator module object
  MotionGeneratorPtr motionGenerator;

  //! Trajectory planner module object
  TrajectoryPlannerPtr trajectoryPlanner;

  //! Vector of pointer to SensorLayer objects
  vector<SensorLayerPtr> sensorLayers;

  //! Vector of pointer to ActuatorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers;

  #ifndef V6_CROSS_BUILD
    //! Pointer to NaoQi internal memory proxy
    ALMemoryProxyPtr memoryProxy;

    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      //! Pointer to NaoQi internal motion class
      ALMotionProxyPtr motionProxy;
    #endif

    //! Pointer to NaoQi internal dcm proxy
    ALDCMProxyPtr dcmProxy;
  #else
    qi::AnyObject memoryProxy;
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      qi::AnyObject motionProxy;
    #endif
  #endif

  enum class MotionSensors : unsigned {
    jointPosition,
    inertial,
    fsr,
    count
  };

  enum class MotionActuators : unsigned {
    jointActuators,
    count
  };
};
