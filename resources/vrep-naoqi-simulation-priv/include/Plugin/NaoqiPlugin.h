#pragma once

#ifdef _WIN32
    #define VREP_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #define VREP_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */

// The 3 required entry points of the plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

#include <alnaosim/alnaosim.h>
#include <alnaosim/alnaosim_camera_definitions.h>
#include <alrobotmodel/alrobotmodel.h>
#include <alsimutils/sim_launcher.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

#define ROBOT_MODEL "NAOH25V50"
#define ROBOT_PORT 19999

//! Naoqi simulated robot launcher
Sim::SimLauncher* naoqiSim;

//! Naoqi simulator sdk path
string naoqiSimPath;

//! Pointer for naoqi based robot model
Sim::Model* naoqiModel;

//! Pointer for naoqi based robot hardware abstraction layer 
Sim::HALInterface* naoqiHal;
