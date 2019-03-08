/**
 * @file TeamNUSTSPL/TeamNUSTSPL.cpp
 *
 * This file implements the class TeamNUSTSPL
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include "GameCommModule/include/GameCommModule.h"
#include "LocalizationModule/include/LocalizationModule.h"
#include "MotionModule/include/MotionModule.h"
#include "PlanningModule/include/PlanningModule.h"
#include "SBModule/include/SBModule.h"
#include "TeamNUSTSPL/include/TeamNUSTSPL.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "UserCommModule/include/UserCommModule.h"
#include "Utils/include/ConfigMacros.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"

#include "Utils/include/JsonUtils.h"
#include "Utils/include/DataHolders/Camera.h"

TeamNUSTSPL* TeamNUSTSPL::self;

TeamNUSTSPL::TeamNUSTSPL(
  ALBrokerPtr parentBroker, const string& parentName) :
  AL::ALModule(parentBroker, parentName)
{
  TeamNUSTSPL::self = this;
}

void TeamNUSTSPL::init()
{
  struct sigaction act;
  memset (&act, '\0', sizeof(act));
  act.sa_sigaction = &TeamNUSTSPL::signal_handler;
  act.sa_flags = SA_SIGINFO;
  if (
    sigaction(SIGINT, &act, NULL) < 0)
  {
    LOG_EXCEPTION("Problem in signal handler creation...");
    return;
  }
  LOG_INFO("Initializing TeamNUSTSPL Module...");
  #ifdef MODULE_IS_LOCAL_SIMULATED
  LOG_INFO("The module is built for local simulations...");
  ConfigManager::setDirPaths("Robots/Sim/");
  ConfigManager::createDirs();
  #else
  #ifndef MODULE_IS_REMOTE
  ConfigManager::createDirs();
  ConfigManager::setDirPaths("");
  #endif
  #endif
  LOG_INFO("Config directory path:\n\t" << ConfigManager::getConfigDirPath());
  LOG_INFO("Logs directory path:\n\t" << ConfigManager::getLogsDirPath());
  LOG_INFO("Getting Naoqi ALMemoryProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/almemoryproxy.h");
  try {
    memoryProxy = getParentBroker()->getMemoryProxy();
  } catch (const AL::ALError& e) {
    LOG_EXCEPTION(e.what()); return;
  }
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("Getting Naoqi ALMotionProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/almotionproxy.h");
  try {
    motionProxy = getParentBroker()->getMotionProxy();
  } catch (const AL::ALError& e) {
    LOG_EXCEPTION(e.what()); return;
  }
  #else
  LOG_INFO("*** [IMPORTANT] *** The module is built without Naoqi ALMotionProxy handle...");
  LOG_INFO("Set USE_NAOQI_MOTION_PROXY to ON in make/cmake/common.cmake if this is needed");
  #endif
  LOG_INFO("Getting Naoqi ALDCMProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/aldcmproxy.h");
  try {
    dcmProxy = getParentBroker()->getDcmProxy();
  } catch (const AL::ALError& e) {
    LOG_EXCEPTION(e.what()); return;
  }
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  LOG_INFO("Getting Naoqi ALVideoDeviceProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/alvideodeviceproxy.h");
  try {
    camProxy =
      getParentBroker()->getSpecialisedProxy<AL::ALVideoDeviceProxy>("ALVideoDevice");
  } catch (const AL::ALError& e) {
    LOG_EXCEPTION(e.what()); return;
  }
  #endif
  LOG_INFO("Initializing local shared memory...");
  LOG_INFO("For information on shared memory, see Team-NUST Documentation." <<
           "The shared memory definition is at src/TNRSBase/");
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  LOG_INFO("Initializing TNRSModules...");
  LOG_INFO("For information on these modules, see Team-NUST Documentation." <<
           "The modules are located at src/TNRSModules");
  setupTNRSModules();
  join();
  LOG_INFO("Exiting TeamNUSTSPL...")
  this->exit();
}

void TeamNUSTSPL::signal_handler(
  int sig, siginfo_t *siginfo, void *context)
{
  self->terminate();
}

void TeamNUSTSPL::setupTNRSModules()
{
  LOG_INFO("Starting module threads... To turn off/on the modules, see " <<
           ConfigManager::getConfigDirPath() << "TeamNUSTSPL.ini")
  vector<bool> modulesToRun(toUType(TNSPLModules::count));
  GET_CONFIG("TeamNUSTSPL",
    (bool, startPlanningModule, modulesToRun[toUType(TNSPLModules::planning)]),
    (bool, startMotionModule, modulesToRun[toUType(TNSPLModules::motion)]),
    (bool, startSBModule, modulesToRun[toUType(TNSPLModules::sb)]),
    (bool, startLocalizationModule, modulesToRun[toUType(TNSPLModules::localization)]),
    (bool, startVisionModule, modulesToRun[toUType(TNSPLModules::vision)]),
    (bool, startGameCommModule, modulesToRun[toUType(TNSPLModules::gameComm)]),
    (bool, startUserCommModule, modulesToRun[toUType(TNSPLModules::userComm)]),
  );

  childModules.resize(toUType(TNSPLModules::count));
  if (modulesToRun[toUType(TNSPLModules::planning)]) {
    LOG_INFO("Constructing PlanningModule... See src/TNRSModules/PlanningModule.")
    childModules[toUType(TNSPLModules::planning)] =
      boost::make_shared<PlanningModule>(this, memoryProxy);
  }
  
  if (modulesToRun[toUType(TNSPLModules::motion)]) {
    LOG_INFO("Constructing MotionModule... See src/TNRSModules/MotionModule.")
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    childModules[toUType(TNSPLModules::motion)] =
      boost::make_shared<MotionModule>(this, memoryProxy, dcmProxy, motionProxy);
    #else
    childModules[toUType(TNSPLModules::motion)] =
      boost::make_shared<MotionModule>(this, memoryProxy, dcmProxy);
    #endif
  }

  if (modulesToRun[toUType(TNSPLModules::sb)]) {
    LOG_INFO("Constructing SBModule... See src/TNRSModules/SBModule.")
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    childModules[toUType(TNSPLModules::sb)] =
      boost::make_shared<SBModule>(this, memoryProxy, dcmProxy, motionProxy);
    #else
    childModules[toUType(TNSPLModules::sb)] =
      boost::make_shared<SBModule>(this, memoryProxy, dcmProxy);
    #endif
  }

  if (modulesToRun[toUType(TNSPLModules::localization)]) {
    LOG_INFO("Constructing LocalizationModule... See src/TNRSModules/LocalizationModule.")
    childModules[toUType(TNSPLModules::localization)] =
      boost::make_shared<LocalizationModule>(this);
  }

  if (modulesToRun[toUType(TNSPLModules::vision)]) {
    LOG_INFO("Constructing VisionModule... See src/TNRSModules/VisionModule.")
    #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
    childModules[toUType(TNSPLModules::vision)] =
      boost::make_shared<VisionModule>(this, camProxy);
    #else
      childModules[toUType(TNSPLModules::vision)] =
        boost::make_shared<VisionModule>(this);
    #endif
  }
  
  #ifdef MODULE_IS_REMOTE
  if (SAVE_IMAGES != -1) {
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    auto sliRequest = boost::make_shared<SwitchLogImages>(true, SAVE_IMAGES);
    BaseModule::publishModuleRequest(sliRequest);
  }
  if (PROJECT_FIELD == 1) {
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    auto sfpRequest = boost::make_shared<SwitchFieldProjection>(true);
    BaseModule::publishModuleRequest(sfpRequest);
  }
  if (USE_LOGGED_IMAGES == 1) {
    auto vRequest = boost::make_shared<SwitchVision>(true);
    BaseModule::publishModuleRequest(vRequest);
    auto uliRequest = boost::make_shared<SwitchUseLoggedImages>(true);
    BaseModule::publishModuleRequest(uliRequest);
  }
  #endif
  
  if (modulesToRun[toUType(TNSPLModules::userComm)]) {
    LOG_INFO("Constructing UserCommModule... See src/TNRSModules/UserCommModule.")
    childModules[toUType(TNSPLModules::userComm)] =
      boost::make_shared<UserCommModule>(this);
  }

  if (modulesToRun[toUType(TNSPLModules::gameComm)]) {
    LOG_INFO("Constructing GameCommModule... See src/TNRSModules/GameCommModule.")
    childModules[toUType(TNSPLModules::gameComm)] =
      boost::make_shared<GameCommModule>(this);
  }

  LOG_INFO("Initializing TNRSModules... See src/TNRSModules/TNRSBase/include/BaseModule.")
  for (size_t i = 0; i < childModules.size(); ++i) {
    if (childModules[i] && modulesToRun[i]) {
      childModules[i]->setLocalSharedMemory(sharedMemory);
      childModules[i]->setup();
    }
  }

  for (size_t i = 0; i < childModules.size(); ++i) {
    if (childModules[i] && modulesToRun[i]) {
      childModules[i]->start();
    }
  }
  //auto vRequest = boost::make_shared<SwitchVision>(true);
  //BaseModule::publishModuleRequest(vRequest);
  /* DebugMsgs test
   * Json::Value value;
  Json::Value vars;
  for (size_t i = 0; i < toUType(CameraSettings::count); ++i)
    vars["settingParams"].append(static_cast<int>(i));
  value["CameraModule"] = vars;
  value["VisionModule"]["debug"] = 1;
  Json::FastWriter fastWriter;
  std::string output = fastWriter.write(value);

  auto vRequest = boost::make_shared<SwitchVision>(true);
  BaseModule::publishModuleRequest(vRequest);

  cout << DebugBase::processDebugMsg(output);*/
}
