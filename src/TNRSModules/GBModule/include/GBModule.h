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
  #ifndef V6_CROSS_BUILD
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
  #else
  ///< Memory input connection
  DECLARE_INPUT_CONNECTOR(
    gbThreadPeriod,
    #ifdef REALTIME_LOLA_AVAILABLE
    jointStiffnessSensors,
    ledSensors,
    #endif
    switchSensors
  );

  ///< Memory output connection
  DECLARE_OUTPUT_CONNECTOR(
    gbThreadTimeTaken,
    #ifndef REALTIME_LOLA_AVAILABLE
    jointStiffnessSensors,
    ledSensors,
    #endif
    stiffnessState,
    whistleDetected,
    gBehaviorInfo
  );
  #endif
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
    #ifndef REALTIME_LOLA_AVAILABLE
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
       */
      GBModule(void* parent);
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
  #ifndef V6_CROSS_BUILD
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      boost::shared_ptr<AL::ALMotionProxy> getSharedMotionProxy()
        { return motionProxy; }
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      qi::AnyObject getSharedMotionProxy()
        { return motionProxy; }
    #endif
  #endif
private:
  #ifndef V6_CROSS_BUILD
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
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      /**
       * @brief sensorsUpdate Updates sensor values from NaoQi
       *   ALMemory to our local shared memory
       */
      void sensorsUpdate();
    #endif
  #endif

  GBManagerPtr gbManager; ///< Static behaviors manager shared object

  #ifndef V6_CROSS_BUILD
  vector<SensorLayerPtr> sensorLayers; ///< Vector of pointer to SensorLayer objects
  vector<ActuatorLayerPtr> actuatorLayers; ///< Vector of pointer to ActuatorLayer objects
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      vector<SensorLayerPtr> sensorLayers; ///< Vector of pointer to SensorLayer objects
    #endif
  #endif

  #ifndef V6_CROSS_BUILD
    ALMemoryProxyPtr memoryProxy; ///< Pointer to NaoQi internal memory proxy
    ALDCMProxyPtr dcmProxy; ///< Pointer to NaoQi internal dcm proxy
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
      boost::shared_ptr<AL::ALMotionProxy> motionProxy; ///< Pointer to NaoQi internal motion class
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      qi::AnyObject memoryProxy; ///< Pointer to NaoQi internal memory proxy
      qi::AnyObject motionProxy; ///< Pointer to NaoQi internal motion class
    #endif
  #endif

  #ifndef V6_CROSS_BUILD
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
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
    /**
     * @enum GBSensors
     * @brief Enumeration for the sensors handled by this class
     */
    enum class GBSensors : unsigned {
      jointStiffnesses,
      led,
      count
    };

    //! No actuator layers in V6 in either case
    #endif
  #endif
};
