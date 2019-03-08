/**
 * @file include/CameraInterface.h
 *
 * This file declares the class SensorInterface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */
 
#pragma once

#include "SensorInterface.h"

/**
 * @class CameraInterface
 * @brief Defines a sensor handle for cameras of the robot
 */ 
class CameraInterface : public SensorInterface
{
public:
  /**
   * Constructor
   */ 
	CameraInterface(
    const string& naoqiHandleName,
    const simxChar* vrepHandleName, 
    Sim::Model* naoqiModel, 
    Sim::HALInterface* naoqiHal, 
    const int& clientId, 
    const int& periodMS) : 
    SensorInterface(
      naoqiHandleName, vrepHandleName, naoqiModel, naoqiHal, clientId, periodMS)
  {
    vrepCamRes = new simxInt();
  }
  
  /**
   * Destructor
   */ 
	virtual ~CameraInterface() { delete image; image = NULL; }
	
  /**
   * Initializes the update thread
   */ 
  void startThread() {
    pthread_create(&thread, NULL, createThread, this);
  }
  
  /**
   * Initializes the handle
   */ 
	void init() { 
    try {
      getVrepHandle();
      getNaoqiHandle();
      startThread();
    } catch(const exception& e) {
      cout << e.what();
    }
  }
  
  /**
   * Get the sensor state from simulation and update in nao sim
   */ 
	void update() { 
    auto tStart = high_resolution_clock::now();
    updateNaoqi(); 
    auto lastIterationTimeMS =
      duration_cast<milliseconds>(high_resolution_clock::now() - tStart).count();
    if(lastIterationTimeMS < periodMS) {
      int waitTimeMS = periodMS - lastIterationTimeMS;
      usleep(waitTimeMS*1000);
    }
  }
  
  bool isThreaded() { return true; }
  
protected:
  /**
   * Gets the vrep sensor handles
   */ 
	void getVrepHandle() {
    simxGetObjectHandle(clientId, vrepHandleName, &vrepHandle, simx_opmode_blocking);
  }
  
  /**
   * Gets the naoqi handle for this sensor
   */
  void getNaoqiHandle() {
    naoqiHandle = naoqiModel->cameraSensor(naoqiHandleName);
    if(naoqiHandle->name() == "CameraTop") {
      naoqiCamRes = Sim::RES_640_480;
    }
    else if(naoqiHandle->name() == "CameraBottom")
      naoqiCamRes = Sim::RES_320_240;
    vector<simxInt> res(2);	
    switch(naoqiCamRes) {
      case Sim::RES_80_60:
        res[0] = 80;
        res[1] = 60;
        break;
      case Sim::RES_160_120:
        res[0] = 160;
        res[1] = 120;
        break;
      case Sim::RES_320_240:
        res[0] = 320;
        res[1] = 240;
        break;
      case Sim::RES_640_480:
        res[0] = 640;
        res[1] = 480;
        break;
      case Sim::RES_1280_960:
        res[0] = 1280;
        res[1] = 960;
        break;
      case Sim::RES_UNKNOWN:
        res[0] = 0;
        res[1] = 0;
        break;
    }
    simxSetObjectIntParameter(
      clientId, 
      vrepHandle,
      sim_visionintparam_resolution_x, 
      res[0], 
      simx_opmode_blocking
    );
    simxSetObjectIntParameter(
      clientId, 
      vrepHandle,
      sim_visionintparam_resolution_y, 
      res[1], 
      simx_opmode_blocking
    );
    simxGetVisionSensorImage(
      clientId, 
      vrepHandle, 
      vrepCamRes, 
      &image, 
      0, 
      simx_opmode_streaming);
  }
  
  /**
   * Updates the Naoqi sensors from vrep sensor values
   */ 
	void updateNaoqi() {
		if (simxGetVisionSensorImage(clientId, vrepHandle, vrepCamRes, &image, 0, simx_opmode_buffer)==simx_return_ok) {
			naoqiHal->sendCameraSensorValue(static_cast<const Sim::CameraSensor*>(naoqiHandle), &image[0], (Sim::CameraResolution)naoqiCamRes, Sim::COL_SPACE_RGB);
		}
  }
  
private:
  //! Pointer to the recveived camera image
	simxUChar* image;

  //! Vrep pointer to camera resolution
  simxInt* vrepCamRes;
  
  //! Naoqi resolution
  Sim::CameraResolution naoqiCamRes;
  
  static void* createThread(void* ptr) {
    SensorInterface* pThis = static_cast<SensorInterface*>(ptr);
    while (true) {
      pThis->update();
    }
    pthread_exit(0);
    return NULL;
  }

  //! Update cycle thread for this sensor
  pthread_t thread;
};
