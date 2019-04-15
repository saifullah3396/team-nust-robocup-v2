/**
 *@file GBModule/src/GBModule.cpp
 *
 * This file implements the class GBModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#ifdef V6_CROSS_BUILD
#include <qi/anyobject.hpp>
#endif
#include "ControlModule/include/ActuatorRequests.h"
#include "ControlModule/include/HardwareLayer.h"
#include "MotionModule/include/MotionRequest.h"
#include "GBModule/include/LedRequest.h"
#include "GBModule/include/GBRequest.h"
#include "GBModule/include/GBModule.h"
#include "GBModule/include/GBManager.h"
#include "GBModule/include/GeneralBehavior.h"
#include "GBModule/include/StiffnessRequest.h"
#include "TeamNUSTSPL/include/TeamNUSTSPL.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/StiffnessState.h"

#ifndef V6_CROSS_BUILD
/**
 * Definition of input connector and variables for this module
 */
DEFINE_INPUT_CONNECTOR(GBModule,
  (int, gbThreadPeriod),
  (vector<float>, switchSensors),
)

/**
 * Definition of output connector and variables for this module
 */
DEFINE_OUTPUT_CONNECTOR(GBModule,
  (int, gbThreadTimeTaken),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, ledSensors),
  (StiffnessState, stiffnessState),
  (bool, whistleDetected),
  (BehaviorInfo, gBehaviorInfo),
)
#else
/**
 * Definition of input connector and variables for this module
 */
DEFINE_INPUT_CONNECTOR(GBModule,
  (int, gbThreadPeriod),
  #ifdef REALTIME_LOLA_AVAILABLE
  (vector<float>, jointStiffnessSensors),
  (vector<float>, ledSensors),
  #endif
  (vector<float>, switchSensors),
)

/**
 * Definition of output connector and variables for this module
 */
DEFINE_OUTPUT_CONNECTOR(GBModule,
  (int, gbThreadTimeTaken),
  #ifndef REALTIME_LOLA_AVAILABLE
  (vector<float>, jointStiffnessSensors),
  (vector<float>, ledSensors),
  #endif
  (StiffnessState, stiffnessState),
  (bool, whistleDetected),
  (BehaviorInfo, gBehaviorInfo),
)
#endif

#ifndef V6_CROSS_BUILD
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    GBModule::GBModule(
      void* parent,
      const ALMemoryProxyPtr& memoryProxy,
      const ALDCMProxyPtr& dcmProxy,
      const ALMotionProxyPtr& motionProxy) :
      BaseModule(parent, TNSPLModules::gb, "GBModule"),
      memoryProxy(memoryProxy),
      dcmProxy(dcmProxy),
      motionProxy(motionProxy)
    {
    }
  #else
    GBModule::GBModule(
      void* parent,
      const ALMemoryProxyPtr& memoryProxy,
      const ALDCMProxyPtr& dcmProxy) :
      BaseModule(parent, TNSPLModules::gb, "GBModule"),
      memoryProxy(memoryProxy),
      dcmProxy(dcmProxy)
    {
    }
  #endif
#else
  #ifndef REALTIME_LOLA_AVAILABLE
    GBModule::GBModule(
      void* parent,
      const qi::AnyObject& memoryProxy,
      const qi::AnyObject& motionProxy) :
      BaseModule(parent, TNSPLModules::gb, "GBModule"),
      memoryProxy(memoryProxy),
      motionProxy(motionProxy)
    {
    }
  #else
    GBModule::GBModule(void* parent) :
      BaseModule(parent, TNSPLModules::gb, "GBModule")
    {
    }
  #endif
#endif

void GBModule::setThreadPeriod()
{
  setPeriodMinMS(GB_PERIOD_IN(GBModule));
}

void GBModule::setThreadTimeTaken()
{
  GB_TIME_TAKEN_OUT(GBModule) = lastIterationTimeMS;
}

void GBModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void GBModule::init()
{
  LOG_INFO("Initializing static behaviors manager...")
  gbManager = boost::make_shared<GBManager>(this);
  ///< Make new layers for sensors directly related with gbmodule
  LOG_INFO("Initializing static behavior module sensor layers...")
  #ifndef V6_CROSS_BUILD
    sensorLayers.resize(toUType(GBSensors::count));
    sensorLayers[toUType(GBSensors::jointStiffnesses)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness),
        OVAR_PTR(vector<float>, GBModule::Output::jointStiffnessSensors), memoryProxy);
    sensorLayers[toUType(GBSensors::led)] =
      SensorLayer::makeSensorLayer(
        toUType(SensorTypes::ledSensors),
        OVAR_PTR(vector<float>, GBModule::Output::ledSensors), memoryProxy);
    ///< Update the sensors
    sensorsUpdate();

    #ifndef MODULE_IS_REMOTE
      ((TeamNUSTSPL*) getParent())->getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&GBModule::sensorsUpdate, this));
    #endif
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
      sensorLayers.resize(toUType(GBSensors::count));
      sensorLayers[toUType(GBSensors::jointStiffnesses)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness),
          OVAR_PTR(vector<float>, GBModule::Output::jointStiffnessSensors), memoryProxy);
      sensorLayers[toUType(GBSensors::led)] =
        SensorLayer::makeSensorLayer(
          toUType(SensorTypes::ledSensors),
          OVAR_PTR(vector<float>, GBModule::Output::ledSensors), memoryProxy);
      ///< Update the sensors
      sensorsUpdate();
    #endif
  #endif

  LOG_INFO("Initializing static behavior module actuator layers...")
  ///< Make new layers for actuators directly related with gbmodule
  #ifndef V6_CROSS_BUILD
    //! No need for realtime actuator if naoqi motion proxy is present
    #ifndef NAOQTI_MOTION_PROXY_AVAILABLE
      actuatorLayers.resize(toUType(GBActuators::count));
        actuatorLayers[toUType(GBActuators::jointStiffnesses)] =
          ActuatorLayer::makeActuatorLayer(
            toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::hardness),
            dcmProxy);
      #ifndef MODULE_IS_REMOTE
        ((TeamNUSTSPL*) getParent())->getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&GBModule::actuatorsUpdate, this));
      #endif
    #endif

    actuatorLayers[toUType(GBActuators::led)] =
      ActuatorLayer::makeActuatorLayer(toUType(ActuatorTypes::ledActuators), dcmProxy);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE // No need for actuator layers in either case for V6
    #endif
  #endif
  LOG_INFO("Initializing GBModule Output Variables...")
  #ifndef V6_CROSS_BUILD
    JOINT_STIFFNESSES_OUT(GBModule) = vector<float>(toUType(Joints::count), 0.f);
    LED_SENSORS_OUT(GBModule) = vector<float>(toUType(LedActuators::count), 0.f);
  #else
    #ifndef REALTIME_LOLA_AVAILABLE
    JOINT_STIFFNESSES_OUT(GBModule) = vector<float>(toUType(Joints::count), 0.f);
    LED_SENSORS_OUT(GBModule) = vector<float>(toUType(LedActuators::count), 0.f);
    #endif
  #endif
  STIFFNESS_STATE_OUT(GBModule) = StiffnessState::unknown;
  WHISTLE_DETECTED_OUT(GBModule) = false;
  GB_INFO_OUT(GBModule) = BehaviorInfo();
}

void GBModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <GBRequest>(request)) {
    auto reqId = request->getRequestId();
    #ifndef V6_CROSS_BUILD
    //! Realtime actuator requests through GBModule are only in V5s
    if (reqId == toUType(GBRequestIds::stiffnessRequest)) {
      actuatorLayers[toUType(GBActuators::jointStiffnesses)]->addRequest(
        boost::static_pointer_cast<StiffnessRequest>(request)
      );
    } else if (reqId == toUType(GBRequestIds::ledRequest)) {
      actuatorLayers[toUType(GBActuators::led)]->addRequest(
        boost::static_pointer_cast<LedRequest>(request)
      );
    } else if (reqId == toUType(GBRequestIds::behaviorRequest)) {
    #else
    if (reqId == toUType(GBRequestIds::behaviorRequest)) {
    #endif
      auto rsb =
        boost::static_pointer_cast<RequestGeneralBehavior>(request);
      gbManager->manageRequest(rsb);
    } else if (reqId == toUType(GBRequestIds::killBehavior)) {
      gbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void GBModule::mainRoutine()
{
  // Update led sensors in mainRoutine()...
  #ifndef V6_CROSS_BUILD
    sensorLayers[toUType(GBSensors::led)]->update();
    #ifdef MODULE_IS_REMOTE
      sensorsUpdate();
    #endif
  #else
  #ifndef REALTIME_LOLA_AVAILABLE
    sensorLayers[toUType(GBSensors::led)]->update();
    sensorsUpdate();
  #endif
  #endif
  gbManager->update();
  GB_INFO_OUT(GBModule) = gbManager->getBehaviorInfo();
  #ifndef V6_CROSS_BUILD
    #ifdef MODULE_IS_REMOTE
      actuatorsUpdate();
    #endif
  #else //! No actuators
  #endif
}

#ifndef V6_CROSS_BUILD
void GBModule::sensorsUpdate()
{
  for (size_t i = 0; i < sensorLayers.size(); ++i) {
    // LEDSensors are updated in mainRoutine() because of their large number
    if (i != toUType(GBSensors::led))
      sensorLayers[i]->update();
  }
}

void GBModule::actuatorsUpdate()
{
  for (size_t i = 0; i < actuatorLayers.size(); ++i) {
    actuatorLayers[i]->update();
  }
}
#else
#ifndef REALTIME_LOLA_AVAILABLE
void GBModule::sensorsUpdate()
{
  for (size_t i = 0; i < sensorLayers.size(); ++i) {
    // LEDSensors are updated in mainRoutine() because of their large number
    if (i != toUType(GBSensors::led))
      sensorLayers[i]->update();
  }
}
#endif
#endif
