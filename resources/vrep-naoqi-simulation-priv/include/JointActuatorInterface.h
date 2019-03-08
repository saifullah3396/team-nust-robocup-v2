/**
 * @file include/JointActuatorInterface.h
 *
 * This file declares the class ActuatorInterface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#pragma once

#include "ActuatorInterface.h"

/**
 * @class JointActuatorInterface
 * @brief Defines an interface for handling joint actuator commands to be
 *   sent to the robot
 */ 
class JointActuatorInterface : public ActuatorInterface
{
public:
  /**
   * Constructor
   */ 
	JointActuatorInterface(
    const string& naoqiHandleName,
    const simxChar* vrepHandleName, 
    Sim::Model* naoqiModel, 
    Sim::HALInterface* naoqiHal, 
    const int& clientId, 
    const int& periodMS) : 
    ActuatorInterface(
      naoqiHandleName, vrepHandleName, naoqiModel, naoqiHal, clientId, periodMS)
  {
  }
  
  /**
   * Destructor
   */ 
	virtual ~JointActuatorInterface() { delete value; value = NULL; }
	
protected:
  /**
   * Gets the vrep sensor handles
   */ 
	void getVrepHandle() {
    simxGetObjectHandle(
      clientId, vrepHandleName, &vrepHandle, simx_opmode_blocking);
  }
  
  /**
   * Gets the naoqi handle for this sensor
   */
  void getNaoqiHandle() {
    string actuatorName;
    if (naoqiHandleName == "RHipYawPitch")
      naoqiHandleName = "LHipYawPitch";
    naoqiHandle = naoqiModel->angleActuator(naoqiHandleName);
  } 
   
  /**
   * Updates the Naoqi sensors from vrep sensor values
   */ 
	void updateVrep() {
    float target = naoqiHal->fetchAngleActuatorValue(static_cast<const Sim::AngleActuator*>(naoqiHandle));
    if(target == target) {// && target != lastTarget) {
      simxSetJointTargetPosition(clientId, vrepHandle, target, simx_opmode_oneshot);
      //lastTarget = target;
    }
    //if (naoqiHandleName == "RKneePitch")
    //  cout << "target2: " << target * 180.0/3.14 << endl;
    //simxSetJointTargetPosition(clientId, vrepHandle, target, simx_opmode_oneshot);
  }
};
