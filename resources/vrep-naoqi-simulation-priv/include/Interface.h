/**
 * @file Simulator/Interface.h
 *
 * This file declares the class Interface
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#include <alnaosim/alnaosim.h>
#include <alnaosim/alnaosim_camera_definitions.h>
#include <alrobotmodel/alrobotmodel.h>
#include <alsimutils/sim_launcher.h>
#include <iostream>
#include <string>
#include <vector>

class SensorInterface;
class ActuatorInterface;

using namespace std;

#define VREP_SERVER_IP "127.0.0.1"
#define VREP_SERVER_PORT 27015
#define VREP_TOP_CAM_SERVER_PORT 27014
#define VREP_BOTTOM_CAM_SERVER_PORT 27013
#define ROBOT_MODEL "NAOH25V50"
#define ROBOT_PORT 9559

extern "C" {
    #include "extApi.h"
}

/**
 * @class Interface
 * @brief Sets up the connection between the vrep cpp remote api server
 *   and the naoqi's simulated robot.
 */ 
class Interface
{
public:
  /**
   * Constructor
   */ 
	Interface(const bool& useCameras);
  
  /**
   * Destructor
   */ 
	~Interface();
  
  /**
   * Initializes the interface
   */ 
	void init();
  
  /**
   * Updates the interface
   */ 
	void update();	
private:
  /**
   * Launches the naoqi simulated robot based on the given simulator
   * sdk and robot model along with naoqi-hal. Furthermore, it sets all
   * nao actuators to their zero position.
   */ 
	void setupNaoqi();
  
  /**
   * Sets up the connnection with vrep remote api servers running on the 
   * simulator
   */ 
	void setupVrep();
  
  /**
   * Sets up the connection between vrep sensor/actuator handles and 
   * naoqi simulated robot sensor/actuator handles
   */ 
  void setupInterface();
	
  //! Naoqi simulated robot launcher
  Sim::SimLauncher* naoqiSim;
  
  //! Naoqi simulator sdk path
  string naoqiSimPath;
  
  //! Pointer for naoqi based robot model
  Sim::Model* naoqiModel;
  
  //! Pointer for naoqi based robot hardware abstraction layer 
  Sim::HALInterface* naoqiHal;
  
  //! The nao model type used in the simulation
  string modelType;
  
  //! The port at which naoqi simulated robot is set up
  int naoqiPort;
  
  //! Client id for connection with vrep sensor streaming server
  int clientId;
  
  //! Client id for connection with vrep camera streaming server
  vector<int> camClientIds;
  
  //! A vector containing all the actuator interfaces
  vector<ActuatorInterface*> actuators;
  
  //! A vector containing all the sensor interfaces
  vector<SensorInterface*> sensors;
  
  std::vector<const Sim::AngleSensor*> jointHandles;
  std::vector<const Sim::FSRSensor*> fsrHandles;
  const Sim::InertialSensor* imuHandle;
  
  //! Incoming data stream
  simxUChar* dataStream;
  
  //! Incoming data stream size
  simxInt dataSize;
  
  //! Whether to update cameras or not
  bool useCameras;
  
  //! Actuators update period
  int actuatorsUpdatePeriod;
  
  //! Sensors update period
  int sensorsUpdatePeriod
};
