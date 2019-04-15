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
#else
#include <qi/anyobject.hpp>
#endif
#include "MotionModule/include/MTypeHeader.h"
#include "TNRSBase/include/BaseIncludes.h"

///< Forward declarations
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
  #ifndef V6_CROSS_BUILD
  DECLARE_INPUT_CONNECTOR(
    motionThreadPeriod,
    jointStiffnessSensors,
    touchSensors,
    ballInfo,
    goalInfo,
    robotPose2D,
    landmarksFound
  );
  DECLARE_OUTPUT_CONNECTOR(
    motionThreadTimeTaken,
    jointPositionSensors,
    handSensors,
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
  );
  #else
  DECLARE_INPUT_CONNECTOR(
    motionThreadPeriod,
    #ifdef REALTIME_LOLA_AVAILABLE
    jointPositionSensors,
    handSensors,
    inertialSensors,
    fsrSensors,
    #endif
    jointStiffnessSensors,
    touchSensors,
    ballInfo,
    goalInfo,
    robotPose2D,
    landmarksFound
  );
  DECLARE_OUTPUT_CONNECTOR(
    motionThreadTimeTaken,
    #ifndef REALTIME_LOLA_AVAILABLE
    jointPositionSensors,
    handSensors,
    inertialSensors,
    fsrSensors,
    #endif
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
  );
  #endif
public:
#ifndef V6_CROSS_BUILD
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE // V5 with motion proxy
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
  #else // V5 without motion proxy
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
  #endif
#else
  #ifndef REALTIME_LOLA_AVAILABLE // V6 with no realtime support
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
  #else // V6 with realtime LoLA support
    /**
     * Constructor
     *
     * @param parent: parent: Pointer to parent module
     */
    MotionModule(void* parent);
  #endif
#endif

  /**
   * @brief ~MotionModule Destructor
   */
  ~MotionModule();

  /**
   * @brief init See BaseModule::init()
   */
  void init() final;

  /**
   * @brief mainRoutine See BaseModule::mainRoutine()
   */
  void mainRoutine() final;

  /**
   * @brief handleRequests See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * @brief initMemoryConn See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * @brief setThreadPeriod See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

  /**
   * @brief setThreadTimeTaken See BaseModule::setThreadTimeTaken()
   */
  void setThreadTimeTaken() final;

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

  /**
   * Gets the motion proxy module
   *
   * @return MotionProxyPtr
   */
  #ifndef V6_CROSS_BUILD
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      ALMotionProxyPtr getSharedMotionProxy();
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      qi::AnyObject getSharedMotionProxy();
    #endif
  #endif

private:
  #ifndef V6_CROSS_BUILD
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
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      /**
       * Sets up the motion sensors
       */
      void setupSensors();

      /**
       * Updates sensor values from NaoQi ALMemory to our local
       * shared memory
       */
      void sensorsUpdate();
    #endif
  #endif

  ///< Motion behaviors manager shared object
  map<unsigned, MBManagerPtr> mbManagers;

  ///< Fall detector module object
  FallDetectorPtr fallDetector;

  ///< Kinematics module object
  KinematicsModulePtr kinematicsModule;

  ///< Motion generator module object
  MotionGeneratorPtr motionGenerator;

  ///< Trajectory planner module object
  TrajectoryPlannerPtr trajectoryPlanner;

  #ifndef V6_CROSS_BUILD
    ///< Vector of pointer to SensorLayer objects
    vector<SensorLayerPtr> sensorLayers;

    ///< Vector of pointer to ActuatorLayer objects
    vector<ActuatorLayerPtr> actuatorLayers;
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
    ///< Vector of pointer to SensorLayer objects
    vector<SensorLayerPtr> sensorLayers;
    #endif
  #endif

  #ifndef V6_CROSS_BUILD
    ///< Pointer to NaoQi internal memory proxy
    ALMemoryProxyPtr memoryProxy;

    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      ///< Pointer to NaoQi internal motion class
      ALMotionProxyPtr motionProxy;
    #endif

    ///< Pointer to NaoQi internal dcm proxy
    ALDCMProxyPtr dcmProxy;
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      qi::AnyObject memoryProxy;
      qi::AnyObject motionProxy;
    #endif
  #endif

  #ifndef V6_CROSS_BUILD
    enum class MotionSensors : unsigned {
      jointPosition,
      handSensors,
      inertial,
      fsr,
      count
    };

    enum class MotionActuators : unsigned {
      jointActuators,
      handActuators,
      count
    };
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      enum class MotionSensors : unsigned {
        jointPosition,
        handSensors,
        inertial,
        fsr,
        count
      };
    #endif
  #endif
};
