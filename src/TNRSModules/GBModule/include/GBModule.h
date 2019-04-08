/**
 * @file GBModule/include/GBModule.h
 *
 * This file declares the class GBModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#ifndef V6_CROSS_BUILD
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#else
#include <qi/anyobject.hpp>
#endif
#include "TNRSBase/include/BaseIncludes.h"

class GBManager;
typedef boost::shared_ptr<GBManager> GBManagerPtr;
class SensorLayer;
typedef boost::shared_ptr<SensorLayer> SensorLayerPtr;
class ActuatorLayer;
typedef boost::shared_ptr<ActuatorLayer> ActuatorLayerPtr;
#ifndef V6_CROSS_BUILD
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;
#endif

/**
 * @class GBModule
 * @brief The base module that handles all kinds of static behaviors
 */
class GBModule : public BaseModule
{
  ///< Memory input connection
  DECLARE_INPUT_CONNECTOR(
    gbThreadPeriod,
    switchSensors
  );

  ///< Memory output connection
  DECLARE_OUTPUT_CONNECTOR(
    gbThreadTimeTaken,
    jointStiffnessSensors,
    ledSensors,
    stiffnessState,
    whistleDetected,
    gBehaviorInfo
  );

public:
  #ifndef V6_CROSS_BUILD
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      /**
       * @brief GBModule Constructor
       *
       * @param parent: parent: Pointer to parent module
       * @param memoryProxy: Pointer to NaoQi's memory proxy
       * @param dcmProxy: Pointer to NaoQi's DCM proxy
       * @param motionProxy: Pointer to NaoQi's motion proxy
       */
      GBModule(
       void* parent,
       const ALMemoryProxyPtr& memoryProxy,
       const ALDCMProxyPtr& dcmProxy,
       const ALMotionProxyPtr& motionProxy);
    #else
      /**
       * @brief GBModule Constructor
       *
       * @param parent: parent: Pointer to parent module
       * @param memoryProxy: Pointer to NaoQi's memory proxy
       * @param dcmProxy: Pointer to NaoQi's DCM proxy
       */
      GBModule(
       void* parent,
       const ALMemoryProxyPtr& memoryProxy,
       const ALDCMProxyPtr& dcmProxy);
    #endif
  #else
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      /**
       * @brief GBModule Constructor
       *
       * @param parent: parent: Pointer to parent module
       * @param memoryProxy: Pointer to NaoQi's memory proxy
       * @param motionProxy: Pointer to NaoQi's motion proxy
       */
      GBModule(
       void* parent,
       const qi::AnyObject& memoryProxy,
       const qi::AnyObject& motionProxy);
    #else
      /**
       * @brief GBModule Constructor
       *
       * @param parent: parent: Pointer to parent module
       * @param memoryProxy: Pointer to NaoQi's memory proxy
       */
      GBModule(
       void* parent,
       const qi::AnyObject& memoryProxy);
    #endif
  #endif

  /**
   * @brief ~GBModule Destructor
   */
  ~GBModule()
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

  /**
   * @brief setThreadTimeTaken See BaseModule::setThreadTimeTaken()
   */
  void setThreadTimeTaken() final;

  ///< Getters
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    #ifndef V6_CROSS_BUILD
      boost::shared_ptr<AL::ALMotionProxy> getSharedMotionProxy()
        { return motionProxy; }
    #else
      qi::AnyObject getSharedMotionProxy()
        { return motionProxy; }
    #endif
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

  GBManagerPtr gbManager; ///< Static behaviors manager shared object
  vector<SensorLayerPtr> sensorLayers; ///< Vector of pointer to SensorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers; ///< Vector of pointer to ActuatorLayer objects

  #ifndef V6_CROSS_BUILD
    ALMemoryProxyPtr memoryProxy; ///< Pointer to NaoQi internal memory proxy
    ALDCMProxyPtr dcmProxy; ///< Pointer to NaoQi internal dcm proxy
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      boost::shared_ptr<AL::ALMotionProxy> motionProxy; ///< Pointer to NaoQi internal motion class
    #endif
  #else
    qi::AnyObject memoryProxy; ///< Pointer to NaoQi internal memory proxy
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      qi::AnyObject motionProxy; ///< Pointer to NaoQi internal motion class
    #endif
  #endif

  /**
   * @enum GBSensors
   * @brief Enumeration for the sensors handled by this class
   */
  enum class GBSensors : unsigned {
    jointStiffnesses,
    led,
    count
  };

  /**
   * @enum GBSensors
   * @brief Enumeration for the actuators handled by this class
   */
  enum class GBActuators : unsigned {
    jointStiffnesses,
    led,
    count
  };
};
