/**

 * @file MotionModuleStandalone/MotionModuleStandalone.cpp
 *
 * This file implements the class MotionModuleStandalone
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */
#include <iostream>
#include <alcommon/albroker.h>
#include "MotionModule/include/MotionModule.h"
#include "MotionModuleStandalone.h"

MotionModuleStandalone::MotionModuleStandalone(
  boost::shared_ptr<AL::ALBroker> parentBroker,
  const string& parentName) :
  AL::ALModule(parentBroker, parentName)
{
}

void MotionModuleStandalone::init()
{
  LOG_INFO("Starting MotionModuleStandalone...")
  #ifdef MODULE_IS_LOCAL_SIMULATED
  ConfigManager::setDirPaths("Robots/Sim/");
  ConfigManager::createDirs();
  #else
  #ifndef MODULE_IS_REMOTE
  ConfigManager::createDirs();
  #endif
  #endif
  LOG_INFO("Setting up NaoQi's proxy handles...")
  memoryProxy = getParentBroker()->getMemoryProxy();
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  motionProxy = getParentBroker()->getMotionProxy();
  #endif
  dcmProxy = getParentBroker()->getDcmProxy();
  LOG_INFO("Initializing memory...")
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  LOG_INFO("SharedMemory Initialized.")
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  childModules.push_back(
    boost::make_shared<MotionModule>(
      this, memoryProxy, dcmProxy, motionProxy)
  );
  #else
  childModules.push_back(
    boost::make_shared<MotionModule>(
      this, memoryProxy, dcmProxy)
  );
  #endif
  for (size_t i = 0; i < childModules.size(); ++i) {
    if (childModules[i]) {
      childModules[i]->setLocalSharedMemory(sharedMemory);
      childModules[i]->setupModule();
    }
  }
  for (size_t i = 0; i < childModules.size(); ++i) {
    if (childModules[i])
      childModules[i]->startModule();
  }
}
