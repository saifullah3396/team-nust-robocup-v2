/**
 * @file include/JointInterface.h
 *
 * This file declares the class SensorInterface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#pragma once

#include "SensorInterface.h"

/**
 * @class JointInterface
 * @brief Defines a sensor handle for joints of the robot
 */ 
class JointInterface : public SensorInterface
{
public:
  /**
   * Constructor
   */ 
	JointInterface(
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
	virtual ~JointInterface() { delete value; value = NULL; }
	
  const Sim::Sensor* getNaoqiSensor() { return naoqiHandle; }
  
protected:
  /**
   * Gets the vrep sensor handles
   */ 
	void getVrepHandle() {
    value = new simxFloat();
    simxGetObjectHandle(
      clientId, vrepHandleName, &vrepHandle, simx_opmode_blocking);
    simxGetJointPosition(
      clientId, vrepHandle, value, simx_opmode_streaming);
  }
  
  /**
   * Gets the naoqi handle for this sensor
   */
  void getNaoqiHandle() {
    naoqiHandle = naoqiModel->angleSensor(naoqiHandleName);
  } 
   
  /**
   * Updates the Naoqi sensors from vrep sensor values
   */ 
	void updateNaoqi() {
    if(simxGetJointPosition(
      clientId, vrepHandle, value, simx_opmode_buffer)==simx_return_ok) {
      naoqiHal->sendAngleSensorValue(static_cast<const Sim::AngleSensor*>(naoqiHandle), *value);
    }
  }
};
