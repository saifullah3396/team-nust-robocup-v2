/**
 * @file SBModule/include/SBModule.h
 *
 * This file declares the class SBModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "TNRSBase/include/BaseIncludes.h"

class SBManager;
typedef boost::shared_ptr<SBManager> SBManagerPtr;
class SensorLayer;
typedef boost::shared_ptr<SensorLayer> SensorLayerPtr;
class ActuatorLayer;
typedef boost::shared_ptr<ActuatorLayer> ActuatorLayerPtr;
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

/**
 * @class SBModule
 * @brief The base module that handles all kinds of static behaviors
 */
class SBModule : public BaseModule
{
  //! Memory input connection
  DECLARE_INPUT_CONNECTOR(
    sbThreadPeriod,
    switchSensors
  )

  //! Memory output connection
  DECLARE_OUTPUT_CONNECTOR(
    jointStiffnessSensors,
    ledSensors,
    stiffnessState,
    whistleDetected,
    sBehaviorInfo
	)

public:
  /**
   * @brief SBModule Constructor
   *
   * @param parent: parent: Pointer to parent module
   * @param memoryProxy: Pointer to NaoQi's memory proxy
   * @param dcmProxy: Pointer to NaoQi's DCM proxy
   * @param motionProxy: Pointer to NaoQi's motion proxy
   */
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  SBModule(
   void* parent,
   const ALMemoryProxyPtr& memoryProxy,
   const ALDCMProxyPtr& dcmProxy,
   const ALMotionProxyPtr& motionProxy);
  #else
    /**
     * @brief SBModule Constructor
     *
     * @param parent: parent: Pointer to parent module
     * @param memoryProxy: Pointer to NaoQi's memory proxy
     * @param dcmProxy: Pointer to NaoQi's DCM proxy
     */
  SBModule(
   void* parent,
   const ALMemoryProxyPtr& memoryProxy,
   const ALDCMProxyPtr& dcmProxy);
  #endif

  /**
   * @brief ~SBModule Destructor
   */
  ~SBModule()
  {    
		delete inputConnector;
    delete outputConnector;
  }

  /**
   * @brief setThreadPeriod See BaseModule::init()
   */
  void init() final;

  /**
   * @brief setThreadPeriod See BaseModule::handleRequests()
   */
  void handleRequests() final;

  /**
   * @brief setThreadPeriod See BaseModule::mainRoutine()
   */
  void mainRoutine() final;

  /**
   * @brief setThreadPeriod See BaseModule::initMemoryConn()
   */
  void initMemoryConn() final;

  /**
   * @brief setThreadPeriod See BaseModule::setThreadPeriod()
   */
  void setThreadPeriod() final;

  //! Getters
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  boost::shared_ptr<AL::ALMotionProxy> getSharedMotionProxy()
  {
    return motionProxy;
  }
  #endif
private:
  /**
   * @brief sensorsUpdate Updates sensor values from NaoQi
   *   ALMemory to our local shared memory
   */
  void sensorsUpdate();

  /**
   * @brief actuatorsUpdate Sends the requested actuator commands
   *   to NaoQi DCM for execution
   */
  void actuatorsUpdate();

  SBManagerPtr sbManager; //! Static behaviors manager shared object
  vector<SensorLayerPtr> sensorLayers; //! Vector of pointer to SensorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers; //! Vector of pointer to ActuatorLayer objects
  ALMemoryProxyPtr memoryProxy; //! Pointer to NaoQi internal memory proxy
  ALDCMProxyPtr dcmProxy; //! Pointer to NaoQi internal dcm proxy
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  boost::shared_ptr<AL::ALMotionProxy> motionProxy; //! Pointer to NaoQi internal motion class
  #endif

  /**
   * @enum SBSensors
   * @brief Enumeration for the sensors handled by this class
   */
  enum class SBSensors : unsigned {
    jointStiffnesses,
    led,
    count
  };

  /**
   * @enum SBSensors
   * @brief Enumeration for the actuators handled by this class
   */
  enum class SBActuators : unsigned {
    jointStiffnesses,
    led,
    count
  };
};
