/**
 * @file TeamNUSTSPL/TeamNUSTSPL.h
 *
 * This file declares the class TeamNUSTSPL
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#pragma once

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <string.h>
#include <assert.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>
#ifdef NAOQI_MOTION_PROXY_AVAILABLE
#include <alproxies/almotionproxy.h>
#endif
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
#include <alproxies/alvideodeviceproxy.h>
#endif
#include <alproxies/dcmproxy.h>
#include "TNRSBase/include/BaseIncludes.h"
#include "TNRSBase/include/BaseModuleHandler.h"
#include "Utils/include/PrintUtils.h"

/**
 * These constants are defined to remove the bug that causes the error
 * "cannot allocate memory in static TLS block." while loading the module
 * on the robot. For details on this bug, see:
 * https://sourceware.org/bugzilla/show_bug.cgi?format=multiple&id=14898
 */
#ifndef NO_TLS_OFFSET
# define NO_TLS_OFFSET  0
#endif
#ifndef FORCED_DYNAMIC_TLS_OFFSET
# if NO_TLS_OFFSET == 0
#  define FORCED_DYNAMIC_TLS_OFFSET -1
# elif NO_TLS_OFFSET == -1
#  define FORCED_DYNAMIC_TLS_OFFSET -2
# else
#  error "FORCED_DYNAMIC_TLS_OFFSET is not defined"
# endif
#endif

extern int USE_LOGGED_IMAGES;
extern int SAVE_IMAGES;
extern int PROJECT_FIELD;
extern string ROBOT_NAME;

class SharedMemory;

typedef boost::shared_ptr<AL::ALBroker> ALBrokerPtr;
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
#ifdef NAOQI_MOTION_PROXY_AVAILABLE
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
#endif
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;
#ifdef NAOQI_VIDEO_PROXY_AVAILABLE
typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;
#endif

/**
 * @class TeamNUSTSPL
 * @brief Loads the software into NaoQi as one of its local modules
 *
 * This class defines a NaoQi local module and initializes the main 
 * architecture modules. It also initializes the local shared memory for 
 * data storage and management.
 */
class TeamNUSTSPL : public AL::ALModule, public BaseModuleHandler
{
public:
  /**
   * @brief TeamNUSTSPL Constructor
   *
   * @param parentBroker: a broker for establishing connection with naoqi
   *   architecture
   * @param moduleName: name of the module with which it is loaded into
   *   naoqi
   */
  TeamNUSTSPL(ALBrokerPtr parentBroker, const string& moduleName);

  /**
   * @brief ~TeamNUSTSPL Destructor
   */
  ~TeamNUSTSPL() {}

  /**
   * @brief init Initializes the class along with its child modules
   */
  void init();

  /**
   * @brief getLocalSharedMemory Returns the pointer to SharedMemory object
   *
   * @return boost::shared_ptr<SharedMemory>
   */
  SharedMemoryPtr
  getLocalSharedMemory()
  {
    return sharedMemory;
  }

  /**
   * @brief signal_handler Termination signal handler
   * @param sig Signal type
   * @param siginfo Signal info
   * @param context Context
   */
  static void signal_handler (int sig, siginfo_t *siginfo, void *context);
private:
  /**
   * Sets up all the tnrs modules.
   */ 
  void setupTNRSModules();
  
  //! Pointer to itself
  static TeamNUSTSPL* self;

  //! Pointer to local data shared memory object
  SharedMemoryPtr sharedMemory;

  //! Pointer to NaoQi internal device communication manager (DCM)
  ALDCMProxyPtr dcmProxy;

  //! Pointer to NaoQi internal memory
  ALMemoryProxyPtr memoryProxy;

  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  //! Pointer to NaoQi internal motion class
  ALMotionProxyPtr motionProxy;
  #endif

  #ifdef NAOQI_VIDEO_PROXY_AVAILABLE
  //! Pointer to NaoQi internal camera class
  ALVideoDeviceProxyPtr camProxy;
  #endif
};
