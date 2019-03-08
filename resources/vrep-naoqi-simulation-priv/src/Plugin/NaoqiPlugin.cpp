#include "Plugin/NaoqiPlugin.h"
#include "scriptFunctionData.h"
#include <iostream>
#include "v_repLib.h"

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
    #define WIN_AFX_MANAGE_STATE
#endif /* __linux || __APPLE__ */

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_NAME "NaoqiInterface"

LIBRARY vrepLib;

void finish();
void setupNaoqi() {
  naoqiSim = new Sim::SimLauncher();
  naoqiSimPath = NAOQI_SIM_SDK;
	try {	
		string naoqiModelPath = naoqiSimPath + 
								 "/share/alrobotmodel/models/" + 
								 ROBOT_MODEL + 
								 ".xml";
    cout << "naoqiModelPath: " << naoqiModelPath << endl;
		naoqiModel = new Sim::Model(naoqiModelPath);
		naoqiHal = new Sim::HALInterface(naoqiModel, ROBOT_PORT);
		if(!naoqiSim->launch(naoqiModel, ROBOT_PORT, naoqiSimPath))
		{
			cout << "Could not launch naoqi sim." << endl;
      finish();
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

void finish() {
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
}


// --------------------------------------------------------------------------------------
// naoqi.start
// --------------------------------------------------------------------------------------
#define NAOQI_START_CMD "naoqiStart"

void LUA_START_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    int handle=-1;
    if (D.readDataFromStack(cb->stackID,NULL,0,NAOQI_START_CMD))
    {
      setupNaoqi();
    }
    //D.pushOutData(CScriptFunctionDataItem(handle));
    //D.writeDataToStack(cb->stackID);
}

// --------------------------------------------------------------------------------------
// naoqi.updateJoint
// --------------------------------------------------------------------------------------
#define JOINT_UPDATE_CMD "naoqiUpdateJoint"

const int inArgs_CREATE[]={
    2,
    sim_script_arg_string,0,
    sim_script_arg_float,0,
};

void LUA_CREATE_CALLBACK(SScriptCallBack* cb)
{
    CScriptFunctionData D;
    int handle=-1;
    if (D.readDataFromStack(cb->stackID,inArgs_CREATE,inArgs_CREATE[0],JOINT_UPDATE_CMD))
    {
      if (naoqiHal) {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        naoqiHal->sendAngleSensorValue(naoqiModel->angleSensor(inData->at(0).stringData[0]), inData->at(1).floatData[0]);
      }
    }
    //D.pushOutData(CScriptFunctionDataItem(handle));
    //D.writeDataToStack(cb->stackID);
}
// --------------------------------------------------------------------------------------

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ 
  setupNaoqi();
  // This is called just once, at the start of V-REP.
    // Dynamically load and bind V-REP functions:
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

    vrepLib=loadVrepLibrary(temp.c_str());
    if (vrepLib==NULL)
    {
        std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start 'NaoqiInterface' plugin.\n";
        return(0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in v_rep.dll. Cannot start 'NaoqiInterface' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    // Check the V-REP version:
    int vrepVer,vrepRev;
    simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
    simGetIntegerParameter(sim_intparam_program_revision,&vrepRev);
    if( (vrepVer<30400) )
    {
        std::cout << "Sorry, your V-REP copy is somewhat old, V-REP 3.4.0 rev9 or higher is required. Cannot start 'NaoqiInterface' plugin.\n";
        std::cout << "vrepRev: " << vrepRev << endl;
        unloadVrepLibrary(vrepLib);
        return(0); // Means error, V-REP will unload this plugin
    }

    simRegisterScriptVariable("naoqi","require('simExtNaoqiInterface')",0);

    cout << "craweting callback..." << endl;
    // Register the new functions:
    simRegisterScriptCallbackFunction(strConCat(NAOQI_START_CMD,"@",PLUGIN_NAME),strConCat("void",NAOQI_START_CMD,"(string naoqiJointHandle, float value)"),LUA_START_CALLBACK);
    simRegisterScriptCallbackFunction(strConCat(JOINT_UPDATE_CMD,"@",PLUGIN_NAME),strConCat("void",JOINT_UPDATE_CMD,"(string naoqiJointHandle, float value)"),LUA_CREATE_CALLBACK);
    return(9); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
    // version 1 was for V-REP versions before V-REP 2.5.12
    // version 2 was for V-REP versions before V-REP 2.6.0
    // version 5 was for V-REP versions before V-REP 3.1.0 
    // version 6 is for V-REP versions after V-REP 3.1.3
    // version 7 is for V-REP versions after V-REP 3.2.0 (completely rewritten)
    // version 8 is for V-REP versions after V-REP 3.3.0 (using stacks for data exchange with scripts)
    // version 9 is for V-REP versions after V-REP 3.4.0 (new API notation)
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
    finish();
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // This function should not generate any error messages:
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

    void* retVal=NULL;

    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}
