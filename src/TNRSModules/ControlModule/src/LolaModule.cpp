/**
 * @file LolaModule/src/LolaModule.cpp
 *
 * This file implements the class LolaModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "ControlModule/include/LolaModule.h"
#include "ControlModule/include/LolaRequest.h"
#include "ControlModule/include/HardwareLayer.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "MotionModule/include/JointRequest.h"
#include "MotionModule/include/HandsRequest.h"
#include "GBModule/include/StiffnessRequest.h"
#include "GBModule/include/LedRequest.h"
#include "Utils/include/Constants.h"
#include "Utils/include/EnumUtils.h"
#include "Utils/include/PrintUtils.h"

DEFINE_INPUT_CONNECTOR(LolaModule,
  (int, lolaThreadPeriod),
);

DEFINE_OUTPUT_CONNECTOR(LolaModule,
  (int, lolaThreadTimeTaken),
  (vector<float>, jointPositionSensors),
  (vector<float>, jointStiffnessSensors),
  (vector<float>, handSensors),
  (vector<float>, inertialSensors),
  (vector<float>, fsrSensors),
  (vector<float>, ledSensors),
  (vector<float>, jointTemperatureSensors),
  (vector<float>, jointCurrentSensors),
  (vector<float>, touchSensors),
  (vector<float>, switchSensors),
  (vector<float>, batterySensors),
  (vector<float>, sonarSensors),
);

LolaModule::LolaModule(void* teamNUSTSPL) :
  BaseModule(teamNUSTSPL, TNSPLModules::lola, "LolaModule")
{
}

LolaModule::~LolaModule()
{
  // When finishing, set all stiffnesses to -1. Also it's necessary to get another packet otherwise we can't send.
  /*char* buffer;
  size_t size;
  socket->receive(boost::asio::buffer(data, maxLen));
  set_stiffness(-1.f, &lolaFrameHandler->actuator_frame.joints.legs);
  set_stiffness(-1.f, &lolaFrameHandler->actuator_frame.joints.arms);
  set_stiffness(-1.f, &lolaFrameHandler->actuator_frame.joints.head);
  tie(buffer, size) = lolaFrameHandler->pack();
  socket->send(boost::asio::buffer(buffer, size));*/
  delete inputConnector;
  delete outputConnector;
}

void LolaModule::setThreadPeriod()
{
  setPeriodMinMS(LOLA_PERIOD_IN(LolaModule));
}

void LolaModule::setThreadTimeTaken()
{
  LOLA_TIME_TAKEN_OUT(LolaModule) = lastIterationTimeMS;
}

void LolaModule::initMemoryConn()
{
  inputConnector =
    new InputConnector(this, getModuleName() + "InputConnector");
  outputConnector =
    new OutputConnector(this, getModuleName() + "OutputConnector");
  inputConnector->initConnector();
  outputConnector->initConnector();
}

void LolaModule::init()
{
  LOG_INFO("Initiating LolaModule...");
  ioService = boost::shared_ptr<boost::asio::io_service> (new boost::asio::io_service());
  auto work = boost::make_shared<boost::asio::io_service::work>(*ioService);
  ioService->run();
  socket = boost::make_shared<local::stream_protocol::socket>(*ioService);
  socket->connect("/tmp/robocup");
  #ifdef V6_CROSS_BUILD
  ///< Make new layers for sensors
  sensorLayers.resize(toUType(LolaSensors::count));
  sensorLayers[toUType(LolaSensors::jointPosition)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::joints) + toUType(JointSensorTypes::position),
      OVAR_PTR(vector<float>, LolaModule::Output::jointPositionSensors));
  sensorLayers[toUType(LolaSensors::jointStiffnesses)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness),
      OVAR_PTR(vector<float>, LolaModule::Output::jointStiffnessSensors));
  sensorLayers[toUType(LolaSensors::handSensors)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::handSensors),
      OVAR_PTR(vector<float>, LolaModule::Output::handSensors));
  sensorLayers[toUType(LolaSensors::inertial)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::inertialSensors),
      OVAR_PTR(vector<float>, LolaModule::Output::inertialSensors));
  sensorLayers[toUType(LolaSensors::fsr)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::fsrSensors),
      OVAR_PTR(vector<float>, LolaModule::Output::fsrSensors));
  sensorLayers[toUType(LolaSensors::jointTemps)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::joints) + toUType(JointSensorTypes::temp),
      OVAR_PTR(vector<float>, LolaModule::Output::jointTemperatureSensors));
  sensorLayers[toUType(LolaSensors::jointCurrents)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::joints) + toUType(JointSensorTypes::current),
      OVAR_PTR(vector<float>, LolaModule::Output::jointCurrentSensors));
  sensorLayers[toUType(LolaSensors::touchSensors)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::touchSensors),
      OVAR_PTR(vector<float>, LolaModule::Output::touchSensors));
  sensorLayers[toUType(LolaSensors::switchSensors)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::switchSensors),
      OVAR_PTR(vector<float>, LolaModule::Output::switchSensors));
  sensorLayers[toUType(LolaSensors::batterySensors)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::batterySensors),
      OVAR_PTR(vector<float>, LolaModule::Output::batterySensors));
  sensorLayers[toUType(LolaSensors::led)] =
    SensorLayer::makeSensorLayer(
      toUType(SensorTypes::ledSensors),
      OVAR_PTR(vector<float>, LolaModule::Output::ledSensors));

  ///< Make new layers for actuators
  actuatorLayers.resize(toUType(LolaActuators::count));
  actuatorLayers[toUType(LolaActuators::jointActuators)] =
    ActuatorLayer::makeActuatorLayer(
      toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::angles));
  actuatorLayers[toUType(LolaActuators::handActuators)] =
    ActuatorLayer::makeActuatorLayer(
      toUType(ActuatorTypes::handActuators));
  actuatorLayers[toUType(LolaActuators::jointStiffnesses)] =
    ActuatorLayer::makeActuatorLayer(
      toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::hardness));
  actuatorLayers[toUType(LolaActuators::led)] =
    ActuatorLayer::makeActuatorLayer(toUType(ActuatorTypes::ledActuators));
  #endif
  #ifdef V6_CROSS_BUILD
  JOINT_STIFFNESSES_OUT(LolaModule) = vector<float>(toUType(Joints::count), 0.f);
  LED_SENSORS_OUT(LolaModule) = vector<float>(toUType(LedActuators::count), 0.f);
  JOINT_POSITIONS_OUT(LolaModule) = vector<float>(toUType(Joints::count), 0.f);
  HAND_SENSORS_OUT(LolaModule) = vector<float>(toUType(RobotHands::count), 0.f);
  INERTIAL_SENSORS_OUT(LolaModule) = vector<float>(toUType(InertialSensors::count), 0.f);
  INERTIAL_SENSORS_OUT(LolaModule)[toUType(InertialSensors::accelerometerX)] = 0.0;
  INERTIAL_SENSORS_OUT(LolaModule)[toUType(InertialSensors::accelerometerY)] = 0.0;
  INERTIAL_SENSORS_OUT(LolaModule)[toUType(InertialSensors::accelerometerZ)] = -Constants::gravity;
  FSR_SENSORS_OUT(LolaModule) = vector<float>(toUType(FsrSensors::count), 0.f);
  JOINT_TEMPERATURES_OUT(LolaModule) = vector<float>(toUType(Joints::count), 0.f);
  JOINT_CURRENTS_OUT(LolaModule) = vector<float>(toUType(Joints::count), 0.f);
  TOUCH_SENSORS_OUT(LolaModule) = vector<float>(toUType(TouchSensors::count), 0.f);
  SWITCH_SENSORS_OUT(LolaModule) = vector<float>(toUType(SwitchSensors::count), 0.f);
  BATTERY_SENSORS_OUT(LolaModule) = vector<float>(toUType(BatterySensors::count), 0.f);
  SONAR_SENSORS_OUT(LolaModule) = vector<float>(toUType(SonarSensors::count), 0.f);
  #endif
}

void LolaModule::handleRequests()
{
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <LolaRequest>(request)) {
    auto reqId = request->getRequestId();
    if (reqId == toUType(LolaRequestIds::jointRequest)) {
      actuatorLayers[toUType(LolaActuators::jointActuators)]->addRequest(
        boost::static_pointer_cast<JointRequest>(request)
      );
    } else if (reqId == toUType(LolaRequestIds::handsRequest)) {
      actuatorLayers[toUType(LolaActuators::handActuators)]->addRequest(
        boost::static_pointer_cast<HandsRequest>(request)
      );
    } else if (reqId == toUType(LolaRequestIds::stiffnessRequest)) {
      actuatorLayers[toUType(LolaActuators::jointStiffnesses)]->addRequest(
        boost::static_pointer_cast<StiffnessRequest>(request)
      );
    } else if (reqId == toUType(LolaRequestIds::ledRequest)) {
      actuatorLayers[toUType(LolaActuators::led)]->addRequest(
        boost::static_pointer_cast<LedRequest>(request)
      );
    }
  }
  inRequests.popQueue();
}

void LolaModule::mainRoutine()
{
  size_t size = socket->receive(boost::asio::buffer(data, maxLen));
  msgpack::unpacker pac;
  pac.reserve_buffer(size);
  memcpy(pac.buffer(), data, size);
  pac.buffer_consumed(size);
  msgpack::object_handle oh;
  if (!pac.next(oh)) {
      cerr << "No MsgPack message in LoLA message." << endl;
      return;
  }
  const auto& map = oh.get().via.map;
  auto* category = map.ptr;
  //for (const auto& sl : sensorLayers) {
//    sl->update(category);
//  }
  cout << "map: " << oh.get() << endl;
  cout << "category: " << category << endl;

  /**char* buffer;
  size_t size;
  tie(buffer, size) = lolaFrameHandler->pack();
  socket->send(boost::asio::buffer(buffer, size));*/
}

#ifdef V6_CROSS_BUILD
void LolaModule::sensorsUpdate()
{
  for (const auto& sl : sensorLayers) {
    //if (sl) sl->update(map);
  }
}
#endif
