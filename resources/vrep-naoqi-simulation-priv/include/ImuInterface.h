/**
 * @file include/ImuInterface.h
 *
 * This file declares the class SensorInterface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#pragma once

#include "SensorInterface.h"

/**
 * @class ImuInterface
 * @brief Defines a sensor handle for joints of the robot
 */ 
class ImuInterface : public SensorInterface
{
public:
  /**
   * Constructor
   */ 
	ImuInterface(
    const string& naoqiHandleName,
    const simxChar* vrepHandleName, 
    Sim::Model* naoqiModel, 
    Sim::HALInterface* naoqiHal, 
    const int& clientId, 
    const int& periodMS) : 
    SensorInterface(
      naoqiHandleName, vrepHandleName, naoqiModel, naoqiHal, clientId, periodMS)
  {
  }
  
  /**
   * Destructor
   */ 
	virtual ~ImuInterface() { delete[] value; value = NULL; }
	const Sim::Sensor* getNaoqiSensor() { return naoqiHandle; }
protected:
  /**
   * Gets the vrep sensor handles
   */ 
	void getVrepHandle() {
    value = new simxFloat[NUM_IMU_SENSORS];
    for (size_t i = 0; i < NUM_IMU_SENSORS; ++i)
      simxGetFloatSignal(clientId, vrepImu[i], &value[i], simx_opmode_streaming);
  }
  
  /**
   * Gets the naoqi handle for this sensor
   */
  void getNaoqiHandle() {
    naoqiHandle = naoqiModel->inertialSensors()[0];
  } 
  
  /**
   * Updates the Naoqi sensors from vrep sensor values
   */ 
	void updateNaoqi() {
    bool flag = false;
    for (size_t i = 0; i < NUM_IMU_SENSORS; ++i) {
      if (simxGetFloatSignal(clientId, vrepImu[i], &value[i], simx_opmode_buffer)==simx_return_ok)
        flag = true;
    }
    if (flag) {
      naoqiHal->sendInertialSensorValues(static_cast<const Sim::InertialSensor*>(naoqiHandle), vector<float>(value, value + NUM_IMU_SENSORS));
    }
  }
};
