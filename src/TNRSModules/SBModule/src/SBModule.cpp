/**
 *@file SBModule/src/SBModule.cpp
 *
 * This file implements the class SBModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "ControlModule/include/ActuatorRequests.h"
#include "ControlModule/include/HardwareLayer.h"
#include "MotionModule/include/MotionRequest.h"
#include "SBModule/include/LedRequest.h"
#include "SBModule/include/SBRequest.h"
#include "SBModule/include/SBModule.h"
#include "SBModule/include/SBManager.h"
#include "SBModule/include/StaticBehavior.h"
#include "SBModule/include/StiffnessRequest.h"
#include "TeamNUSTSPL/include/TeamNUSTSPL.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/DataHolders/StiffnessState.h"

/**
 * Definition of input connector and variables for this module
 */
DEFINE_INPUT_CONNECTOR(SBModule,
  (int, sbThreadPeriod),
  (vector<float>, switchSensors),
)

/**
 * Definition of output connector and variables for this module
 */
DEFINE_OUTPUT_CONNECTOR(SBModule,
  (vector<float>, jointStiffnessSensors),
  (vector<float>, ledSensors),
  (StiffnessState, stiffnessState),
  (bool, whistleDetected),
  (BehaviorInfo, sBehaviorInfo),
)

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
SBModule::SBModule(
  void* parent,
  const ALMemoryProxyPtr& memoryProxy,
  const ALDCMProxyPtr& dcmProxy,
  const ALMotionProxyPtr& motionProxy) :
  BaseModule(parent, toUType(TNSPLModules::sb), "SBModule"),
  memoryProxy(memoryProxy),
  dcmProxy(dcmProxy),
  motionProxy(motionProxy)
{
}
#else
SBModule::SBModule(
  void* parent,
  const ALMemoryProxyPtr& memoryProxy,
  const ALDCMProxyPtr& dcmProxy) :
  BaseModule(parent, toUType(TNSPLModules::sb), "SBModule"),
  memoryProxy(memoryProxy),
  dcmProxy(dcmProxy)
{
}
#endif

void SBModule::setThreadPeriod()
{
  setPeriodMinMS(SB_PERIOD_IN(SBModule));
}

void SBModule::initMemoryConn()
{
  inputConnector = 
		new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector = 
		new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void SBModule::init()
{
  LOG_INFO("Initializing static behaviors manager...")
  sbManager = boost::make_shared<SBManager>(this);
  //! Make new layers for sensors directly related with sbmodule
  LOG_INFO("Initializing static behavior module sensor layers...")
  sensorLayers.resize(toUType(SBSensors::count));
  sensorLayers[toUType(SBSensors::jointStiffnesses)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness),
      OVAR_PTR(vector<float>, SBModule::Output::jointStiffnessSensors), memoryProxy);
  sensorLayers[toUType(SBSensors::led)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::ledSensors),
      OVAR_PTR(vector<float>, SBModule::Output::ledSensors), memoryProxy);
  //! Update the sensors
  sensorsUpdate();
  #ifndef MODULE_IS_REMOTE
  ((TeamNUSTSPL*) getParent())->getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&SBModule::sensorsUpdate, this));
  #endif
  LOG_INFO("Initializing static behavior module actuator layers...")
  //! Make new layers for actuators directly related with sbmodule
  actuatorLayers.resize(toUType(SBActuators::count));
  actuatorLayers[toUType(SBActuators::jointStiffnesses)] =
    ActuatorLayer::makeActuatorLayer(
      toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::hardness),
      dcmProxy);
  actuatorLayers[toUType(SBActuators::led)] =
    ActuatorLayer::makeActuatorLayer(toUType(ActuatorTypes::ledActuators), dcmProxy);
  #ifndef MODULE_IS_REMOTE
  ((TeamNUSTSPL*) getParent())->getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&SBModule::actuatorsUpdate, this));
  #endif
  LOG_INFO("Initializing SBModule Output Variables...")
  JOINT_STIFFNESSES_OUT(SBModule) = vector<float>(toUType(Joints::count), 0.f);
  LED_SENSORS_OUT(SBModule) = vector<float>(toUType(LedActuators::count), 0.f);
  STIFFNESS_STATE_OUT(SBModule) = StiffnessState::unknown;
  WHISTLE_DETECTED_OUT(SBModule) = false;
  SB_INFO_OUT(SBModule) = BehaviorInfo();
}

void SBModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <SBRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == toUType(SBRequestIds::stiffnessRequest)) {
      actuatorLayers[toUType(SBActuators::jointStiffnesses)]->addRequest(
        boost::static_pointer_cast<StiffnessRequest>(request)
      );
    } else if (reqId == toUType(SBRequestIds::ledRequest)) {
      actuatorLayers[toUType(SBActuators::led)]->addRequest(
        boost::static_pointer_cast<LedRequest>(request)
      );
    } else if (reqId == toUType(SBRequestIds::behaviorRequest)) {
      auto rsb = 
        boost::static_pointer_cast<RequestStaticBehavior>(request);
      sbManager->manageRequest(rsb);
    } else if (reqId == toUType(SBRequestIds::killBehavior)) {
      sbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void SBModule::mainRoutine()
{
  // Update led sensors in mainRoutine()...
  sensorLayers[toUType(SBSensors::led)]->update();
  #ifdef MODULE_IS_REMOTE
  sensorsUpdate();
  #endif
  sbManager->update();
  SB_INFO_OUT(SBModule) = sbManager->getBehaviorInfo();
  #ifdef MODULE_IS_REMOTE
  actuatorsUpdate();
  #endif
}

void SBModule::sensorsUpdate()
{
  for (size_t i = 0; i < sensorLayers.size(); ++i) {
    // LEDSensors are updated in mainRoutine() because of their large number
    if (i != toUType(SBSensors::led))
      sensorLayers[i]->update();
  }
}

void SBModule::actuatorsUpdate()
{
  for (size_t i = 0; i < actuatorLayers.size(); ++i) {
    actuatorLayers[i]->update();
  }
}
