/**
 * @file src/Interface.cpp
 *
 * This file implements the class that starts a remote Api client to connect
 * with the robot simulated in Vrep.
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#include "CameraInterface.h"
#include "FsrInterface.h"
#include "Interface.h"
#include "ImuInterface.h"
#include "JointInterface.h"
#include "JointActuatorInterface.h"
#include "SensorKeys.h"

Interface::Interface(const bool& useCameras) :
  modelType(ROBOT_MODEL),
  naoqiPort(ROBOT_PORT),
  actuatorsUpdatePeriod(5),
  sensorsUpdatePeriod(20),
  useCameras(useCameras),
  camClientIds(NUM_CAMERA_SENSORS),
  fsrHandles(NUM_FSR_SENSORS)
{
  //! Set up a new nao simulator from naoqi-sdk
  naoqiSim = new Sim::SimLauncher();
  
  //! Get the path from the build variable
  naoqiSimPath = NAOQI_SIM_SDK;
}

Interface::~Interface() 
{
  if(naoqiSim) {
    delete naoqiSim;
    naoqiSim = NULL;
	}
  if(naoqiModel) {
    delete naoqiModel;
    naoqiModel = NULL;
	}
  if(naoqiHal) {
    delete naoqiHal;
    naoqiHal = NULL;
	}
	simxFinish(clientId);
  if (useCameras) {
    simxFinish(camClientIds[0]);
    simxFinish(camClientIds[1]);
  }
}

void Interface::init()
{
	setupNaoqi();
	setupVrep();
	setupInterface();
  //simxSynchronous(clientId,true);
  if (useCameras) {
    simxStartSimulation(camClientIds[0], simx_opmode_blocking);
    simxStartSimulation(camClientIds[1], simx_opmode_blocking);
  }
	simxStartSimulation(clientId, simx_opmode_blocking);
  simxGetStringSignal(clientId, "nao_data", &dataStream, &dataSize, simx_opmode_streaming);
}

void Interface::update()
{
  auto tStart = high_resolution_clock::now();
  //simxSynchronousTrigger(clientId);
  static auto lastActuatorUpdateTimeMS = high_resolution_clock::now();
  auto actuatorUpdateTimeDiffMS =
    duration_cast<milliseconds>(high_resolution_clock::now() - lastActuatorUpdateTimeMS).count();
  if (actuatorUpdateTimeDiffMS > actuatorsUpdatePeriod) {
    for (size_t i = 0; i < actuators.size(); ++i)
      actuators[i]->update();
    lastActuatorUpdateTimeMS = high_resolution_clock::now();
  }
  static auto lastSensorUpdateTimeMS = high_resolution_clock::now();
  auto sensorUpdateTimeDiffMS =
    duration_cast<milliseconds>(high_resolution_clock::now() - lastSensorUpdateTimeMS).count();
  if (sensorUpdateTimeDiffMS > sensorsUpdatePeriod) {
    if (simxGetStringSignal(clientId, "nao_data", &dataStream, &dataSize, simx_opmode_buffer)==simx_return_ok) {
      float* data = &((float*)dataStream)[0];
      vector<float> dataVec = vector<float> (data, data + dataSize/4);
      for (int i=0;i<dataSize/4;i++) {
        //cout << "i: " << i << " " << dataVec[i] << endl;
        if (i < NUM_JOINTS) {
          naoqiHal->sendAngleSensorValue(naoqiModel->angleSensor(naoqiJoints[i]), dataVec[i]);
        } else if (i >= NUM_JOINTS && i < NUM_JOINTS + NUM_FSR_SENSORS) {
          naoqiHal->sendFSRSensorValue(fsrHandles[i-NUM_JOINTS], dataVec[i]);
        } else if (i >= NUM_JOINTS + NUM_FSR_SENSORS && i < NUM_JOINTS + NUM_FSR_SENSORS + NUM_IMU_SENSORS) {
          auto imu = vector<float>(dataVec.begin() + i, dataVec.end());
          ///for (int j = 0 ;j < NUM_IMU_SENSORS;++j) {
          ///  cout << "j: " << j << " " << imu[j] << endl;
          ///}
          naoqiHal->sendInertialSensorValues(
              imuHandle,
              imu
            );
          i+=NUM_IMU_SENSORS;
        }
      }
    }
    lastSensorUpdateTimeMS = high_resolution_clock::now();
  }
  for (size_t i = 0; i < sensors.size(); ++i) {
    sensors[i]->update();
  }
    
}

void Interface::setupNaoqi()
{
	try {	
		string naoqiModelPath = naoqiSimPath + 
								 "/share/alrobotmodel/models/" + 
								 modelType + 
								 ".xml";
		naoqiModel = new Sim::Model(naoqiModelPath);
		naoqiHal = new Sim::HALInterface(naoqiModel, naoqiPort);
		if(!naoqiSim->launch(naoqiModel, naoqiPort, naoqiSimPath))
		{
			cout << "Could not launch naoqi sim." << endl;
		}
    jointHandles = naoqiModel->angleSensors();
    for (size_t i = 0; i < NUM_FSR_SENSORS; ++i) {
      fsrHandles[i] = naoqiModel->fsrSensor(naoqiFsrs[i]);
    }
    imuHandle = naoqiModel->inertialSensors()[0];
  } catch (exception &e){
		cout << e.what() << endl;
	}
}

void Interface::setupVrep()
{
	try {	
		simxFinish(-1);
		clientId = simxStart((simxChar*)VREP_SERVER_IP,
							   VREP_SERVER_PORT,true,true,2000,5);
    if (clientId != -1)
			cout << "Connected to vrep remote API sensors server." << endl;
		else
			cout << "Connection to vrep remote API sensors server failed." << endl;
    if (useCameras) {
      camClientIds[0] = simxStart((simxChar*)VREP_SERVER_IP,
      					   VREP_TOP_CAM_SERVER_PORT,true,true,2000,5);
      camClientIds[1] = simxStart((simxChar*)VREP_SERVER_IP,
      					   VREP_BOTTOM_CAM_SERVER_PORT,true,true,2000,5);
      if (camClientIds[0] != -1 && camClientIds[1] != -1)
      	cout << "Connected to vrep remote API camera servers." << endl;
      else
      	cout << "Connection to vrep remote API camera servers failed." << endl;
    }
    
    auto jointActuators = naoqiModel->angleActuators();
    for(vector<const Sim::AngleActuator*>::const_iterator it =
        jointActuators.begin(); it != jointActuators.end(); ++it)
    {
      float actuatorPosition = (*it)->startValue();
      const Sim::AngleSensor* sensor = naoqiModel->angleSensor((*it)->name());
      naoqiHal->sendAngleSensorValue(sensor, actuatorPosition);
    }
	} catch (exception &e){
		cout << e.what() << endl;
	}
}

void Interface::setupInterface()
{
	try {	
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      // Sensor values are now taken in the form of a single string signal
      // sensors.push_back(
      //   new JointInterface(naoqiJoints[i], vrepJoints[i], naoqiModel, naoqiHal, clientId, 5)
      // );
      actuators.push_back(
        new JointActuatorInterface(
          naoqiJoints[i], vrepJoints[i], naoqiModel, naoqiHal, clientId, 5)
      );
    }
    
    if (useCameras) {
      for (size_t i = 0; i < NUM_CAMERA_SENSORS; ++i) {
        sensors.push_back(
          new CameraInterface(naoqiCameras[i], vrepCameras[i], naoqiModel, naoqiHal, camClientIds[i], 30)
        );
      }
    }
    
    // Sensor values are now taken in the form of a single string signal
    // sensors.push_back(
    //   new FsrInterface("FsrSensors", "FsrSensors", naoqiModel, naoqiHal, clientId, 5)
    // );
  
    // Sensor values are now taken in the form of a single string signal
    // sensors.push_back(
    //   new ImuInterface("ImuSensor", "ImuSensor", naoqiModel, naoqiHal, clientId, 5)
    // );
    
    for (size_t i = 0; i < sensors.size(); ++i) {
      sensors[i]->init();
    }
    
    for (size_t i = 0; i < actuators.size(); ++i) {
      actuators[i]->init();
    }
	} catch (exception &e) {
		cout << e.what() << endl;
	}
}
