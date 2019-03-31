/**
 * @file MotionModuleTests/MotionModuleTests.cpp
 *
 * This file implements the class MotionModuleTests
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/tests/include/MotionModuleTests.h"

MotionModuleTests::MotionModuleTests(ALBrokerPtr parentBroker,
  const string& parentName) :
  AL::ALModule(parentBroker, parentName)
{
}

void
MotionModuleTests::init()
{
  #ifndef V6_CROSS_BUILD
    LOG_INFO("Starting MotionModuleTests...")
    LOG_INFO("Setting up NaoQi's Proxies...")
    memoryProxy = getParentBroker()->getMemoryProxy();
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    motionProxy = getParentBroker()->getMotionProxy();
    #endif
    dcmProxy = getParentBroker()->getDcmProxy();
    LOG_INFO("Initializing memory...")
    sharedMemory = boost::shared_ptr<SharedMemory>(new SharedMemory());
    sharedMemory->init();
    LOG_INFO("SharedMemory Initialized.")
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    motionProxy->wakeUp();
    #endif
    #ifdef NAOQI_MOTION_PROXY_AVAILABLE
    childModules.push_back(
      boost::shared_ptr <MotionModule> (
        new MotionModule(this, memoryProxy, dcmProxy, motionProxy)));
    #endif
    for (int i = 0; i < childModules.size(); ++i) {
      childModules[i]->setLocalSharedMemory(sharedMemory);
      childModules[i]->setup();
    }

    for (int i = 0; i < childModules.size(); ++i)
      childModules[i]->start();
  #endif
}
