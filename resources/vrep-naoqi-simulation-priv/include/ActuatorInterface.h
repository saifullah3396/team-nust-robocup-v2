/**
 * @file Simulator/ActuatorInterface.h
 *
 * This file declares the class ActuatorInterface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#pragma once

#include <qi/log.hpp>
#include <alnaosim/alnaosim.h>
#include <alnaosim/alnaosim_camera_definitions.h>
#include <alrobotmodel/alrobotmodel.h>
#include <alsimutils/sim_launcher.h>
#include <alproxies/almemoryproxy.h>

#include <pthread.h>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include <unistd.h>

#include "SensorKeys.h"

extern "C" {
    #include "extApi.h"
}

using namespace std;
using namespace std::chrono;

/**
 * @class ActuatorInterface
 * @brief Defines an actuator interface that is responsible for updating 
 *   a vrep actuators from naoqi commands
 */ 
class ActuatorInterface
{
public:
  /**
   * Constructor
   */ 
	ActuatorInterface(
    const string& naoqiHandleName,
    const simxChar* vrepHandleName,
    Sim::Model* naoqiModel, 
    Sim::HALInterface* naoqiHal, 
    const int& clientId, 
    const int& periodMS) :
    naoqiHandleName(naoqiHandleName),
    vrepHandleName(vrepHandleName),
    naoqiModel(naoqiModel), 
    naoqiHal(naoqiHal), 
    clientId(clientId),
    periodMS(periodMS)
  {
  }
  
  /**
   * Destructor
   */ 
	virtual ~ActuatorInterface() {}
  
  /**
   * Initializes the handle
   */ 
	void init() { 
    try {
      getVrepHandle();
      getNaoqiHandle();
      updateVrep();
    } catch(const exception& e) {
      cout << e.what();
    }
  }
  
  /**
   * Initializes the update thread
   */ 
  //void startThread() {
  //  pthread_create(&thread, NULL, createThread, this);
  //}
  
  /**
   * Get the sensor state from simulation and update in nao sim
   */ 
	void update() { 
    //auto tStart = high_resolution_clock::now();
    updateVrep(); 
    /*auto lastIterationTimeMS =
      duration_cast<milliseconds>(high_resolution_clock::now() - tStart).count();
    if(lastIterationTimeMS < periodMS) {
      int waitTimeMS = periodMS - lastIterationTimeMS;
      usleep(waitTimeMS*1000);
    }*/
  }
	
protected:
  /**
   * Gets the vrep sensor handles
   */ 
	virtual void getVrepHandle() = 0;
  
  /**
   * Updates the Naoqi sensors from vrep sensor values
   */ 
	virtual void updateVrep() = 0;
  
  /**
   * Gets the naoqi sensor handle
   */ 
  virtual void getNaoqiHandle() = 0;

  //! Pointer to naoqi based robot model
  Sim::Model* naoqiModel;
  
  //! Pointer to naoqi based robot hardware abstraction layer 
  Sim::HALInterface* naoqiHal;
  
  //! Client id for connection with vrep sensor streaming server
  int clientId;
  
  //! Period of update for this sensor
  int periodMS;
  
  //! Handle to vrep sensor
	simxInt vrepHandle;
  
  //! Handle name for vrep sensor
	const simxChar* vrepHandleName;
  
  //! Handle name for naoqi sensor
	string naoqiHandleName;
  
  //! Handle to naoqi sensor
  const Sim::DCMActuator* naoqiHandle;
  
  //! Container for received joint position values
  simxFloat* value;
  
private:
  /*static void* createThread(void* ptr) {
    ActuatorInterface* pThis = static_cast<ActuatorInterface*>(ptr);
    while (true) {
      pThis->update();
    }
    pthread_exit(0);
    return NULL;
  }*/

  //! Update cycle thread for this sensor
  //pthread_t thread;
};
