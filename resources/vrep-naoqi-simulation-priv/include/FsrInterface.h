/**
 * @file include/FsrInterface.h
 *
 * This file declares the class SensorInterface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */
 
#pragma once

#include "SensorInterface.h"

/**
 * @class FsrInterface
 * @brief Defines a sensor handle for joints of the robot
 */ 
class FsrInterface : public SensorInterface
{
public:
  /**
   * Constructor
   */ 
	FsrInterface(
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
	virtual ~FsrInterface() { delete[] value; value = NULL; }
  const Sim::Sensor* getNaoqiSensor(const unsigned& index) { return naoqiHandles[index]; }
protected:
  /**
   * Gets the vrep sensor handles
   */ 
  void getVrepHandle() {
    value = new simxFloat[NUM_FSR_SENSORS];
    for (size_t i = 0; i < 0; ++i)
      simxGetFloatSignal(clientId, vrepFsrs[i], &value[i], simx_opmode_streaming);
  }
  
  /**
   * Gets the naoqi handle for this sensor
   */
  void getNaoqiHandle() {
    naoqiHandles = naoqiModel->fsrSensors();
  } 
  
  /**
   * Updates the Naoqi sensors from vrep sensor values
   */ 
	void updateNaoqi() {
    bool flag = false;
    for (size_t i = 0; i < 0; ++i) {
      if (simxGetFloatSignal(clientId, vrepFsrs[i], &value[i], simx_opmode_buffer)==simx_return_ok) {
        naoqiHal->sendFSRSensorValue(naoqiHandles[i], *value);
      }
    }
  }

private:
  std::vector<const Sim::FSRSensor*> naoqiHandles;
};
