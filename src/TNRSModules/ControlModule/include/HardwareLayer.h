/**
 * @file ControlModule/include/HardwareLayer.h
 *
 * This file declares classes SensorLayer and ActuatorLayers,
 * and defines their child classes for each type of sensors and 
 * actuators
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 June 2017
 */

#pragma once

#include <queue>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "ControlModule/include/ActuatorRequests.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/ThreadSafeQueue.h"
#include "Utils/include/JsonUtils.h"

typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;
typedef boost::shared_ptr<vector<float> > vectorFloatPtr;

#define DEFINE_SENSOR_LAYER(ClassName, FileName) \
  class ClassName : public SensorLayer \
  { \
  public: \
    /** \
     * Constructor \
     * \
     * @param memoryProxy: pointer to NaoQi's memory proxy. \
     */ \
    ClassName(const ALMemoryProxyPtr& memoryProxy) : \
      SensorLayer(memoryProxy) \
    { \
      init(FileName); \
    } \
};

#define DEFINE_ACTUATOR_LAYER(ClassName, FileName) \
  class ClassName : public ActuatorLayer \
  { \
  public: \
    /** \
     * Constructor \
     * \
     * @param memoryProxy: pointer to NaoQi's memory proxy. \
     */ \
    ClassName(const ALDCMProxyPtr& dcmProxy) : \
      ActuatorLayer(dcmProxy) \
    { \
      init(FileName); \
    } \
};

/**
 * @class SensorLayer
 * @brief Defines a layer of sensor interface handles
 */
class SensorLayer
{
public:
  /**
   * @brief SensorLayer Constructor
   * @param memoryProxy Naoqi memory proxy
   */
  SensorLayer(const ALMemoryProxyPtr& memoryProxy) :
    memoryProxy(memoryProxy)
  {
  }

  /**
   * @brief ~SensorLayer Destructor
   */
  virtual ~SensorLayer() {}

  /**
   * @brief update Updates the sensor values from NaoQi ALMemory
   */
  void update();

  /**
   * @brief setSensorHandle Sets the sensor container
   * @param sensorHandle Pointer to sensor container
   */
  void setSensorHandle(vector<float>*& sensorHandle)
  {
    this->sensorHandle = sensorHandle;
    this->sensorHandle->resize(size);
  }

  /**
   * @rief makeSensorLayer Constructs a sensor layer for given index
   *
   * @param sensorIndex: index of the sensor group
   * @param sensorHandle: pointer to the object recieving sensor values
   *
   * @return boost::shared_ptr<SensorLayer>
   */
  static boost::shared_ptr<SensorLayer> makeSensorLayer(
    const unsigned& sensorIndex,
    vector<float>* sensorHandle,
    const ALMemoryProxyPtr& memoryProxy);

protected:
  /**
   * @brief setSensorPtr Updates sensor pointers from naoqi memory
   */
  void setSensorPtr()
  {
    if (memoryProxy) {
      #ifndef MODULE_IS_REMOTE
      sensorPtrs.resize(size);
      for (size_t i = 0; i < size; ++i)
        sensorPtrs[i] = (float*) memoryProxy->getDataPtr(keys[i]);
      #endif
    } else {
      LOG_ERROR("Cannot access Naoqi motion proxy at SensorLayer::setSensorPtrs().");
    }
  }

  /**
   * @brief init Initializes the sensor class for given
   *   sensors based on their keys
   */
  void init(const string& jsonFile) {
    try {
      auto path =
        ConfigManager::getCommonConfigDirPath() + "/Sensors/" + jsonFile + ".json";
      Json::Value json;
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
    setSensorPtr();
  }

private:
  vector<string> keys; //! Vector of sensor keys
  size_t size; //! Size of Sensors
  ALMemoryProxyPtr memoryProxy; //! Pointer to NaoQi internal memory
  vector<float>* sensorHandle; //! Extracted values of the sensors
  vector<float*> sensorPtrs; //! Pointers to sensors of NaoQi ALMemory
};

typedef boost::shared_ptr<SensorLayer> SensorLayerPtr;

/**
 * @class JointSensors
 * @brief Defines the handles for JointSensors
 */
class JointSensorsLayer : public SensorLayer
{
public:
  /**
   * @brief JointSensors Constructor
   * @param memoryProxy Naoqi memory proxy
   * @param type Type of joint sensors
   */
  JointSensorsLayer(
    const ALMemoryProxyPtr& memoryProxy,
    const JointSensorTypes& type) :
    SensorLayer(memoryProxy)
  {
    string file;
    switch (type) {
      case JointSensorTypes::position:
        file = "JointPositionSensors"; break;
      case JointSensorTypes::hardness:
        file = "JointHardnessSensors"; break;
      case JointSensorTypes::temp:
        file = "JointTemperatureSensors"; break;
      case JointSensorTypes::current:
        file = "JointCurrentSensors"; break;
    }
    init(file);
  }
};

DEFINE_SENSOR_LAYER(TouchSensorsLayer, "TouchSensors")
DEFINE_SENSOR_LAYER(SwitchSensorsLayer, "SwitchSensors")
DEFINE_SENSOR_LAYER(BatterySensorsLayer, "BatterySensors")
DEFINE_SENSOR_LAYER(InertialSensorsLayer, "InertialSensors")
DEFINE_SENSOR_LAYER(FsrSensorsLayer, "FsrSensors")
DEFINE_SENSOR_LAYER(SonarSensorsLayer, "SonarSensors")
DEFINE_SENSOR_LAYER(LedSensorsLayer, "LedSensors")

/**
 * @class ActuatorLayer
 * @brief Defines a layer of actuator handles
 */
class ActuatorLayer
{
public:
  /**
   * @brief ActuatorLayer Constructor
   * @param dcmProxy Naoqi DCM Proxy
   */
  ActuatorLayer(
    const ALDCMProxyPtr& dcmProxy) : dcmProxy(dcmProxy) {}

  /**
   * @brief ~ActuatorLayer Destructor
   */
  virtual ~ActuatorLayer() {}

  /**
   * @brief init Initializes the actuators based on their keys
   * @param jsonFile Path to file containing actuator keys
   */
  void init(const string& jsonFile) {
    try {
      auto path =
        ConfigManager::getCommonConfigDirPath() + "/Actuators/" + jsonFile + ".json";
      Json::Value json;
      //cout << "path:" << path << endl;
      ifstream config(path, ifstream::binary);
      config >> json;
      alias = json["alias"].asString();
      if (json["keys"].size() > 0) {
        auto keysObj = json["keys"];
        size = keysObj.size();
        keys.resize(size);
        for (int i = 0; i < size; ++i) {
          keys[i] = keysObj[i].asString();
          //cout << i << ": " <<  keys[i] << endl;
        }
      }
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Error reading hardware layer ids from json file:\t" << jsonFile << "\n\t" << e.what());
    }
    if (dcmProxy) {
      setActuatorAlias();
      setActuatorCommand();
    } else {
      LOG_ERROR("Cannot access Naoqi DCM proxy at ActuatorLayer::init().");
    }
  }

  /**
   * @brief update Sends the actuator requests to NaoQi DCM for execution
   * @param request: The requested actuators
   */
  void update();

  /**
   * @brief makeActuatorLayer Constructs an actuator layer for given index
   * @param actuatorIndex Actuator layer index
   * @param dcmProxy Naoqi DCM Proxy
   * @return ActuatorLayerPtr
   */
  static boost::shared_ptr<ActuatorLayer>
  makeActuatorLayer(const unsigned& actuatorIndex, const ALDCMProxyPtr& dcmProxy);

  virtual void addRequest(const ActuatorRequestPtr& request)
    { requests.pushToQueue(request); }

private:
  /**
   * @brief setActuatorAlias Initializes the actuators command alias
   */
  void setActuatorAlias();

  /**
   * @brief setActuatorCommand Initializes actuator command object
   */
  void setActuatorCommand();

  vector<string> keys; //! Vector to memory keys
  unsigned size; //! Size of actuators
  string alias; //! Unique actuation request command alias
  ThreadSafeQueue<ActuatorRequestPtr> requests; //! Actuator requests queue
  float dcmTime; //! NaoQi DCM architecture time
  //AL::ALValue commands; //! Commands sent to DCM
  ALDCMProxyPtr dcmProxy; //! Pointer to NaoQi DCM
};

typedef boost::shared_ptr<ActuatorLayer> ActuatorLayerPtr;

/**
 * @class JointActuators
 * @brief Defines the handles for JointActuators
 */
class JointActuatorsLayer : public ActuatorLayer
{
public:
  /**
   * Constructor
   *
   * @param dcmProxy: pointer to NaoQi's dcm proxy.
   * @param type Joint actuator type
   */
  JointActuatorsLayer(
    const ALDCMProxyPtr& dcmProxy, 
    const JointActuatorTypes& type) :
    ActuatorLayer(dcmProxy)
  {
    string file;
    switch (type) {
      case JointActuatorTypes::angles:
        file = "PositionActuators";
        break;
      case JointActuatorTypes::hardness:
        file = "HardnessActuators";
        break;
    }
    init(file);
  }
};

DEFINE_ACTUATOR_LAYER(LedActuatorsLayer, "LedActuators")
