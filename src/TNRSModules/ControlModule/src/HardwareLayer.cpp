/**
 * @file ControlModule/src/HardwareLayer.cpp
 *
 * This file implements classes SensorLayer and ActuatorLayers,
 * and defines their child classes fir each type of sensors and
 * actuators
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 June 2017
 */

#include "Utils/include/PrintUtils.h"
#include "Utils/include/DataUtils.h"
#include "ControlModule/include/HardwareLayer.h"

/**
 * @brief init Initializes the sensor class for given
 *   sensors based on their keys
 */
void SensorLayer::init(const string& jsonFile) {
  try {
    auto path =
      ConfigManager::getCommonConfigDirPath() + "/Sensors/" + jsonFile + ".json";    Json::Value json;
    ifstream config(path, ifstream::binary);
    config >> json;
    if (json["keys"].size() > 0) {
      auto keysObj = json["keys"];
      size = keysObj.size();
      keys.resize(size);
      for (int i = 0; i < size; ++i) {
        keys[i] = keysObj[i].asString();
      }
    }
  } catch (Json::Exception& e) {
    LOG_EXCEPTION("Error reading hardware layer ids from json file:\t" << jsonFile << "\n\t" << e.what());
  }
  #ifndef V6_CROSS_BUILD
  setSensorPtr();
  #endif
}

#ifndef V6_CROSS_BUILD
void SensorLayer::update()
{
  try {
    #ifdef MODULE_IS_REMOTE
    *sensorHandle = memoryProxy->getListData(keys);
    #else
    for (size_t i = 0; i < size; ++i) {
      (*sensorHandle)[i] = *sensorPtrs[i];
    }
    #endif
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what())
  }
}
#else
#ifndef REALTIME_LOLA_AVAILABLE
void SensorLayer::update()
{
  try {
    *sensorHandle = memoryProxy.call<AL::ALValue>("getListData", keys);
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what())
  }
}
#else
void SensorLayer::update(const msgpack::object_map& map)
{
  try {
    for (size_t i = 0; i < size; ++i) {
      //(*sensorHandle)[i] = map[keys[i]];
    }
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what())
  }
}
#endif
#endif

#ifndef V6_CROSS_BUILD
void SensorLayer::setSensorPtr()
{
  if (memoryProxy) {
    #ifndef MODULE_IS_REMOTE
    sensorPtrs.resize(size);
    for (size_t i = 0; i < size; ++i) {
      sensorPtrs[i] = (float*) memoryProxy->getDataPtr(keys[i]);
    }
    #endif
  } else {
    LOG_ERROR("Cannot access Naoqi motion proxy at SensorLayer::setSensorPtrs().");
  }
}
#endif

#ifndef V6_CROSS_BUILD
SensorLayerPtr SensorLayer::makeSensorLayer(
  const unsigned& sensorIndex,
  vector<float>* sensorHandle,
  const NAOQI_MEMORY_PROXY_TYPE& memoryProxy)
{
  try {
    SensorLayerPtr sl;
    switch (sensorIndex) {
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::position):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::position);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::temp):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::temp);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::current):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::current);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::hardness);
        break;
      case toUType(SensorTypes::handSensors):
        sl = boost::make_shared<HandSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::touchSensors):
        sl = boost::make_shared<TouchSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::switchSensors):
        sl = boost::make_shared<SwitchSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::batterySensors):
        sl = boost::make_shared<BatterySensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::inertialSensors):
        sl = boost::make_shared<InertialSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::sonarSensors):
        sl = boost::make_shared<SonarSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::fsrSensors):
        sl = boost::make_shared<FsrSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::ledSensors):
        sl = boost::make_shared<LedSensorsLayer>(memoryProxy);
        break;
    }
    sl->setSensorHandle(sensorHandle);
    return sl;
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}
#else
#ifndef REALTIME_LOLA_AVAILABLE
SensorLayerPtr SensorLayer::makeSensorLayer(
  const unsigned& sensorIndex,
  vector<float>* sensorHandle,
  const NAOQI_MEMORY_PROXY_TYPE& memoryProxy)
{
  try {
    SensorLayerPtr sl;
    switch (sensorIndex) {
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::position):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::position);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::temp):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::temp);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::current):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::current);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness):
        sl = boost::make_shared<JointSensorsLayer>(memoryProxy, JointSensorTypes::hardness);
        break;
      case toUType(SensorTypes::handSensors):
        sl = boost::make_shared<HandSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::touchSensors):
        sl = boost::make_shared<TouchSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::switchSensors):
        sl = boost::make_shared<SwitchSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::batterySensors):
        sl = boost::make_shared<BatterySensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::inertialSensors):
        sl = boost::make_shared<InertialSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::sonarSensors):
        sl = boost::make_shared<SonarSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::fsrSensors):
        sl = boost::make_shared<FsrSensorsLayer>(memoryProxy);
        break;
      case toUType(SensorTypes::ledSensors):
        sl = boost::make_shared<LedSensorsLayer>(memoryProxy);
        break;
    }
    sl->setSensorHandle(sensorHandle);
    return sl;
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}
#else
SensorLayerPtr SensorLayer::makeSensorLayer(
  const unsigned& sensorIndex,
  vector<float>* sensorHandle)
{
  try {
    SensorLayerPtr sl;
    switch (sensorIndex) {
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::position):
        sl = boost::make_shared<JointSensorsLayer>(JointSensorTypes::position);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::temp):
        sl = boost::make_shared<JointSensorsLayer>(JointSensorTypes::temp);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::current):
        sl = boost::make_shared<JointSensorsLayer>(JointSensorTypes::current);
        break;
      case toUType(SensorTypes::joints) + toUType(JointSensorTypes::hardness):
        sl = boost::make_shared<JointSensorsLayer>(JointSensorTypes::hardness);
        break;
      case toUType(SensorTypes::handSensors):
        sl = boost::make_shared<HandSensorsLayer>();
        break;
      case toUType(SensorTypes::touchSensors):
        sl = boost::make_shared<TouchSensorsLayer>();
        break;
      case toUType(SensorTypes::switchSensors):
        sl = boost::make_shared<SwitchSensorsLayer>();
        break;
      case toUType(SensorTypes::batterySensors):
        sl = boost::make_shared<BatterySensorsLayer>();
        break;
      case toUType(SensorTypes::inertialSensors):
        sl = boost::make_shared<InertialSensorsLayer>();
        break;
      case toUType(SensorTypes::sonarSensors):
        sl = boost::make_shared<SonarSensorsLayer>();
        break;
      case toUType(SensorTypes::fsrSensors):
        sl = boost::make_shared<FsrSensorsLayer>();
        break;
      case toUType(SensorTypes::ledSensors):
        sl = boost::make_shared<LedSensorsLayer>();
        break;
    }
    sl->setSensorHandle(sensorHandle);
    return sl;
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}
#endif
#endif

void ActuatorLayer::update()
{
 try {
    if (requests.isEmpty())
      return;
    ///< Get the earliest request in queue
    cout << "Getting que front" << endl;
    auto request = requests.queueFront();
    #ifndef V6_CROSS_BUILD
      for (size_t i = 0; i < size; ++i) {
        commands[5][i][0] = (request->getValue())[i];
      }
      commands[4][0] = dcmProxy->getTime(20);
      dcmProxy->setAlias(commands);
    #else
    #ifdef REALTIME_LOLA_AVAILABLE
    ///< @todo Define lola based request handler for realtime usage
    #endif
    #endif
    cout << "popping que" << endl;
    ///< Execute in remove
    requests.popQueue();
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what())
  }
}

#ifndef V6_CROSS_BUILD
void ActuatorLayer::setActuatorAlias()
{
  AL::ALValue commandAlias;
  commandAlias.arraySetSize(2);
  commandAlias[0] = string(alias);
  commandAlias[1].arraySetSize(size);
  for (size_t i = 0; i < size; ++i)
    commandAlias[1][i] = string(keys[i]);
  try {
    dcmProxy->createAlias(commandAlias);
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what())
  }
}
#endif

#ifndef V6_CROSS_BUILD
void ActuatorLayer::setActuatorCommand()
{
  commands.arraySetSize(6);
  commands[0] = string(alias);
  commands[1] = string("ClearAll");
  commands[2] = string("time-separate");
  commands[3] = 0;
  commands[4].arraySetSize(1);
  commands[5].arraySetSize(size);
  for (size_t i = 0; i < size; ++i)
    commands[5][i].arraySetSize(1);
}
#endif

#ifndef V6_CROSS_BUILD
ActuatorLayerPtr ActuatorLayer::makeActuatorLayer(
  const unsigned& actuatorIndex,
  const ALDCMProxyPtr& dcmProxy)
#else
ActuatorLayerPtr ActuatorLayer::makeActuatorLayer(
  const unsigned& actuatorIndex)
#endif
{
  try {
    ActuatorLayerPtr al;
    switch (actuatorIndex)
    {
      case toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::angles):
        #ifndef V6_CROSS_BUILD
          al = boost::make_shared<JointActuatorsLayer>(dcmProxy, JointActuatorTypes::angles);
        #else
          al = boost::make_shared<JointActuatorsLayer>(JointActuatorTypes::angles);
        #endif
        break;
      case toUType(ActuatorTypes::jointActuators) + toUType(JointActuatorTypes::hardness):
        #ifndef V6_CROSS_BUILD
          al = boost::make_shared<JointActuatorsLayer>(dcmProxy, JointActuatorTypes::hardness);
        #else
          al = boost::make_shared<JointActuatorsLayer>(JointActuatorTypes::hardness);
        #endif
        break;
      case toUType(ActuatorTypes::handActuators):
        #ifndef V6_CROSS_BUILD
          al = boost::make_shared<HandActuatorsLayer>(dcmProxy);
        #else
          al = boost::make_shared<HandActuatorsLayer>();
        #endif
        break;
      case toUType(ActuatorTypes::ledActuators):
        #ifndef V6_CROSS_BUILD
          al = boost::make_shared<LedActuatorsLayer>(dcmProxy);
        #else
          al = boost::make_shared<LedActuatorsLayer>();
        #endif
        break;
    }
    return al;
  } catch (exception &e) {
    LOG_EXCEPTION(e.what())
  }
}
