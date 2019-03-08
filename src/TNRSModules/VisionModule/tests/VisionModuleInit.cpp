#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
#include <alproxies/alvideodeviceproxy.h>
#endif
#include "Utils/include/ConfigManager.h"
#include "TNRSBase/include/SharedMemory.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"

using namespace std;

#ifdef NAOQI_MOTION_PROXY_AVAILABLE
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
#endif
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
#endif
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

int
main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: say NAO_IP" << std::endl;
    exit(2);
  }
  ConfigManager::setDirPaths("Robots/Sim/");
  SharedMemoryPtr sharedMemory;
  sharedMemory = boost::make_shared<SharedMemory>();
  sharedMemory->init();
  cout << "Memory initiated successfully." << endl;
  //ALMemoryProxyPtr memoryProxy = boost::make_shared<AL::ALMemoryProxy>(argv[1], 9559);
  //ALDCMProxyPtr dcmProxy = boost::make_shared<AL::DCMProxy>(argv[1], 9559);
  //ALMotionProxyPtr motionProxy = boost::make_shared<AL::ALMotionProxy>(argv[1], 9559);
  int* dummyParent;
  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  ALVideoDeviceProxyPtr camProxy =
    boost::make_shared < AL::ALVideoDeviceProxy > (argv[1], 9559);
  auto vModule = boost::make_shared <VisionModule> (dummyParent, camProxy);
  #else
  auto vModule = boost::make_shared <VisionModule> (dummyParent);
  #endif
  vModule->setLocalSharedMemory(sharedMemory);
  vModule->setup();
  vModule->start();
  auto vRequest = boost::make_shared<SwitchVision>(true);
  auto fpRequest = boost::make_shared<SwitchFieldProjection>(true);
  auto litRequest = boost::make_shared<SwitchLogImages>(true, 0);
  auto libRequest = boost::make_shared<SwitchLogImages>(true, 1);
  BaseModule::publishModuleRequest(vRequest);
  //BaseModule::publishModuleRequest(litRequest);
  //BaseModule::publishModuleRequest(libRequest);
  //BaseModule::publishModuleRequest(fpRequest);
  vModule->join();
  return 1;
}
