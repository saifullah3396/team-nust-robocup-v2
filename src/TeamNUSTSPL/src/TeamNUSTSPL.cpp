/**
 * @file TeamNUSTSPL/src/TeamNUSTSPL.cpp
 *
 * This file implements the class TeamNUSTSPL
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include "GameCommModule/include/GameCommModule.h"
#include "LocalizationModule/include/LocalizationModule.h"
#ifdef V6_CROSS_BUILD
  #ifdef REALTIME_LOLA_AVAILABLE
    #include "ControlModule/include/LolaModule.h"
  #endif
#endif
#include "MotionModule/include/MotionModule.h"
#include "PlanningModule/include/PlanningModule.h"
#include "GBModule/include/GBModule.h"
#include "TeamNUSTSPL/include/TeamNUSTSPL.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "UserCommModule/include/UserCommModule.h"
#include "Utils/include/ConfigMacros.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/JsonUtils.h"
#include "Utils/include/DataHolders/Camera.h"
#include "Utils/include/PrintUtils.h"

TeamNUSTSPL* TeamNUSTSPL::self;

#ifndef V6_CROSS_BUILD
TeamNUSTSPL::TeamNUSTSPL(
  ALBrokerPtr parentBroker, const string& parentName) :
  AL::ALModule(parentBroker, parentName)
{
  TeamNUSTSPL::self = this;
}
#else
TeamNUSTSPL::TeamNUSTSPL(qi::SessionPtr session) :
  session(session)
{
  TeamNUSTSPL::self = this;
}
#endif

void TeamNUSTSPL::init()
{
  #ifdef MODULE_IS_LOCAL_SIMULATED
  LOG_INFO("The module is built for local simulations...");
  ConfigManager::setDirPaths("Robots/Sim/");
  ConfigManager::createDirs();
  #else
    #ifndef MODULE_IS_REMOTE
      ConfigManager::setDirPaths("");
      ConfigManager::createDirs();
    #endif
  #endif
  LOG_INFO("Initializing TeamNUSTSPL Module...");
  LOG_INFO("Getting Naoqi ALMemoryProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/almemoryproxy.h");
  #ifndef V6_CROSS_BUILD
    try {
      memoryProxy = getParentBroker()->getMemoryProxy();
    } catch (const exception& e) {
      LOG_EXCEPTION(e.what()); return;
    }
  #else
    memoryProxy = session->service("ALMemory");
  #endif
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  LOG_INFO("Getting Naoqi ALMotionProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/almotionproxy.h");
  #ifndef V6_CROSS_BUILD
    try {
      motionProxy = getParentBroker()->getMotionProxy();
    } catch (const exception& e) {
      LOG_EXCEPTION(e.what()); return;
    }
  #else
    motionProxy = session->service("ALMotion");
  #endif
  #else
  LOG_INFO("*** [IMPORTANT] *** The module is built without Naoqi ALMotionProxy handle...");
  LOG_INFO("Set USE_NAOQI_MOTION_PROXY to ON in make/cmake/common.cmake if this is needed");
  #endif
  #ifndef V6_CROSS_BUILD ///< DCM Replaced by LOLA in naoqi 2.8
  LOG_INFO("Getting Naoqi ALDCMProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/aldcmproxy.h");
  try {
    dcmProxy = getParentBroker()->getDcmProxy();
  } catch (const exception& e) {
    LOG_EXCEPTION(e.what()); return;
  }
  #endif
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  LOG_INFO("Getting Naoqi ALVideoDeviceProxy handle...");
  LOG_INFO("For information on this proxy, see naoqi-sdk/include/alproxies/alvideodeviceproxy.h");
  try {
    #ifndef V6_CROSS_BUILD
      camProxy =
        getParentBroker()->getSpecialisedProxy<AL::ALVideoDeviceProxy>("ALVideoDevice");
    #else
      camProxy = session->service("ALVideoDevice");
    #endif
  } catch (const exception& e) {
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
  /*struct sigaction act;
  memset (&act, '\0', sizeof(act));
  act.sa_sigaction = &TeamNUSTSPL::signal_handler;
  act.sa_flags = SA_SIGINFO;
  if (
    sigaction(SIGINT, &act, NULL) < 0)
  {
    LOG_EXCEPTION("Problem in signal handler creation...");
    return;
  }*/
  LOG_INFO("Terminating TeamNUSTSPL Module...");
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
    (bool, startGBModule, modulesToRun[toUType(TNSPLModules::gb)]),
    (bool, startLocalizationModule, modulesToRun[toUType(TNSPLModules::localization)]),
    (bool, startVisionModule, modulesToRun[toUType(TNSPLModules::vision)]),
    (bool, startGameCommModule, modulesToRun[toUType(TNSPLModules::gameComm)]),
    (bool, startUserCommModule, modulesToRun[toUType(TNSPLModules::userComm)]),
    #ifdef V6_CROSS_BUILD
      #ifdef REALTIME_LOLA_AVAILABLE
        (bool, startLolaModule, modulesToRun[toUType(TNSPLModules::lola)]),
      #endif
    #endif
  );

  childModules.resize(toUType(TNSPLModules::count));
  if (modulesToRun[toUType(TNSPLModules::planning)]) {
    LOG_INFO("Constructing PlanningModule... See src/TNRSModules/PlanningModule.")
    #ifndef V6_CROSS_BUILD
      childModules[toUType(TNSPLModules::planning)] =
        boost::make_shared<PlanningModule>(this, memoryProxy);
    #else
      #ifndef REALTIME_LOLA_AVAILABLE
        childModules[toUType(TNSPLModules::planning)] =
          boost::make_shared<PlanningModule>(this, memoryProxy);
      #else
        childModules[toUType(TNSPLModules::planning)] =
          boost::make_shared<PlanningModule>(this);
      #endif
    #endif
  }

  if (modulesToRun[toUType(TNSPLModules::motion)]) {
    LOG_INFO("Constructing MotionModule... See src/TNRSModules/MotionModule.")
    #ifndef V6_CROSS_BUILD
      #ifdef NAOQI_MOTION_PROXY_AVAILABLE
        childModules[toUType(TNSPLModules::motion)] =
          boost::make_shared<MotionModule>(this, memoryProxy, dcmProxy, motionProxy);
      #else
        childModules[toUType(TNSPLModules::motion)] =
          boost::make_shared<MotionModule>(this, memoryProxy, dcmProxy);
      #endif
    #else
      #ifndef REALTIME_LOLA_AVAILABLE
        childModules[toUType(TNSPLModules::motion)] =
          boost::make_shared<MotionModule>(this, memoryProxy, motionProxy);
      #else
        childModules[toUType(TNSPLModules::motion)] =
          boost::make_shared<MotionModule>(this);
      #endif
    #endif
  }

  if (modulesToRun[toUType(TNSPLModules::gb)]) {
    LOG_INFO("Constructing GBModule... See src/TNRSModules/GBModule.")
    #ifndef V6_CROSS_BUILD
      #ifdef NAOQI_MOTION_PROXY_AVAILABLE
        childModules[toUType(TNSPLModules::gb)] =
          boost::make_shared<GBModule>(this, memoryProxy, dcmProxy, motionProxy);
      #else
        childModules[toUType(TNSPLModules::gb)] =
          boost::make_shared<GBModule>(this, memoryProxy, dcmProxy);
      #endif
    #else
      #ifndef REALTIME_LOLA_AVAILABLE
        childModules[toUType(TNSPLModules::gb)] =
          boost::make_shared<GBModule>(this, memoryProxy, motionProxy);
      #else
        childModules[toUType(TNSPLModules::gb)] =
          boost::make_shared<GBModule>(this);
      #endif
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
    auto sliRequest = boost::make_shared<SwitchLogImages>(true, static_cast<CameraId>(SAVE_IMAGES));
    BaseModule::publishModuleRequest(sliRequest);
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(false, FeatureExtractionIds::segmentation));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(false, FeatureExtractionIds::field));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(false, FeatureExtractionIds::robot));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(false, FeatureExtractionIds::lines));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(false, FeatureExtractionIds::goal));
    BaseModule::publishModuleRequest(
      boost::make_shared<SwitchFeatureExtModule>(false, FeatureExtractionIds::ball));
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

  #ifdef V6_CROSS_BUILD
    #ifdef REALTIME_LOLA_AVAILABLE
      if (modulesToRun[toUType(TNSPLModules::lola)]) {
        LOG_INFO("Constructing LolaModule... See src/TNRSModules/LolaModule.")
        childModules[toUType(TNSPLModules::lola)] =
          boost::make_shared<LolaModule>(this);
      }
    #endif
  #endif

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
  //auto sliRequest1 = boost::make_shared<SwitchLogImages>(true, CameraId::headTop);
  //BaseModule::publishModuleRequest(sliRequest1);
  //auto sliRequest2 = boost::make_shared<SwitchLogImages>(true, CameraId::headBottom);
  //BaseModule::publishModuleRequest(sliRequest2);

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
  */
}

#ifdef V6_CROSS_BUILD
QI_REGISTER_MT_OBJECT(TeamNUSTSPL, init);
#endif
