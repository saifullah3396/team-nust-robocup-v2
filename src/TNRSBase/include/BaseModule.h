/**
 * @file TNRSBase/include/BaseModule.h
 *
 * This file declares the class BaseModule and defines a helper class
 * ThreadException
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include <chrono>
#include "TNRSBase/include/MemoryBase.h"
#include "TNRSBase/include/MemoryConnector.h"
#include "TNRSBase/include/SharedMemory.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ThreadSafeQueue.h"

using namespace std::chrono;

/**
 * @class ThreadException
 * @brief Thread exception management class
 */
class ThreadException : public exception
{
public:
  /**
   * @brief ThreadException Constructor
   *
   * @param message explanatory message
   * @param bSysMsg true if a system message (from strerror(errno))
   *   should be postfixed to the user provided message
   */
  ThreadException(const string& message, const bool& bSysMsg) throw ();

  /**
   * @brief ~ThreadExcpetion Destructor
   */
  virtual ~ThreadException() throw ();

  /**
   * @brief what() Returns a pointer to the (constant) error description
   *
   * @return a pointer to a const char
   */
  virtual const char* what() const throw ()
  {
    return sMsg.c_str();
  }

protected:
  string sMsg; ///< Error message.
};

/**
 * @class BaseModule
 * @brief A class that defines basic threaded modules
 *
 * The class BaseModule is the base class for all the core threaded
 *   modules in the software. A class that inherits from BaseModule
 *   gains its own thread with its period defined in SharedMemory
 *   and also directly gains access to the SharedMemory on definition.
 *   Furthermore, only the classes inheriting from BaseModule have the
 *   ability to define which of the SharedMemory variables will be
 *   their input and which variables will be taken as an output from
 *   the given class. These input and output variables must be defined
 *   for every class inheriting from BaseModule, by using the macros
 *   defined in InoutConnectors.h.
 */
class BaseModule : public MemoryBase
{
public:
  /**
   * @brief BaseModule Constructor
   *
   * @param parent a pointer to the parent class. Type of the parent
   *   can differ in each case.
   * @param moduleId a unique module identity
   * @param moduleName a unique module name
   * @param inRequestsMax maximum possible number of requests at a time
   */
  BaseModule(
    void* parent,
    const TNSPLModules& moduleId,
    const string& moduleName,
    const size_t& inRequestsMax = 20);

  /**
   * @brief ~BaseModule Destructor
   */
  virtual ~BaseModule();

  /**
   * @brief setup Sets up the module and initializes memory connections
   */
  void setup();

  /**
   * * @brief start Starts the module thread
   */
  void start();

  /**
   * @brief upperRoutine Upper layer of thread routine
   * @return True if thread is still running
   */
  bool upperRoutine();

  /**
   * @brief join Joints the module thread
   */
  virtual void join();

  /**
   * @brief suspendModule Suspends the module thread
   *
   * @param parentCommand whether the suspend command is from the parent
   * @return void
   */
  void suspend(const bool& parentCommand = false);

  /**
   * @brief resume Resumes the module thread
   *
   * @param parentCommand whether the resume command is from the parent
   * @return void
   */
  void resume(const bool& parentCommand = false);

  /**
   * @brief wait Pauses the module thread based on some condition
   *
   * @param condition condition to be true
   * @return void
   */
  void wait(const bool& condition);

  /**
   * @brief terminate Terminates the thread
   */
  virtual void terminate();

  /**
   * @brief addRequest Pushes a module request to the module request queue
   *
   * @param request: The request object
   */
  void addRequest(const ModuleRequestPtr& request) {
    if (inRequests.getSize() >= inRequestsMax)
      return;
    inRequests.pushToQueue(request);
  }

  /**
   * @brief  publishModuleRequest Can be called from any BaseModule to publish
   *   a module request to another BaseModule based on its id.
   *
   * @param request: The request to be published
   * @param moduleId: Id of the module to which the request is to be
   *   published
   */
  static void publishModuleRequest(const ModuleRequestPtr& request)
  {
    for (size_t i = 0; i < runningModules.size(); ++i) {
      if (request->getModuleId() == runningModules[i]->getModuleId()) {
        runningModules[i]->addRequest(request);
      }
    }
  }

  /**
   * @brief getModule Returns a base module that matches the given id
   *
   * @param moduleId: Id of requested module
   *
   * @return BaseModule*
   */
  static BaseModule* getModule(const TNSPLModules& moduleId)
  {
    for (size_t i = 0; i < runningModules.size(); ++i) {
      if (moduleId == runningModules[i]->getModuleId()) {
        return runningModules[i];
      }
    }
    return NULL;
  }

  ///< Getters
  const string& getModuleName() const { return moduleName; }
  const TNSPLModules& getModuleId() const { return moduleId; }
  void* getParent() const { return parent; }
  const int& getPeriodMinMS() const { return periodMinMS; }
  const double& getModuleTime() const { return moduleTime; }
  SharedMemoryPtr getLocalSharedMemory() const { return sharedMemory; }

  ///< Setters
  void setPeriodMinMS(const int& periodMinMS) { this->periodMinMS = periodMinMS; }
  void setLocalSharedMemory(const SharedMemoryPtr& sharedMemory)
  {
    this->sharedMemory = sharedMemory;
  }

protected:
  ThreadSafeQueue<ModuleRequestPtr> inRequests; ///< A queue for receiving requests
  int lastIterationTimeMS = {0}; ///< Time taken by the thread in milliseconds

private:
  /**
   * @brief handleRequests Defines how the incoming module requests are handled
   */
  virtual void handleRequests() = 0;

  /**
   * @brief mainRoutine Defines the main processing loop of this module
   */
  virtual void mainRoutine() = 0;

  /**
   * @brief Defines the input and output memory connections to shared memory
   */
  virtual void initMemoryConn() = 0;

  /**
   * @brief init Initializes the module and its variables
   */
  virtual void init() = 0;

  /**
   * @brief Sets the thread cycle time
   */
  virtual void setThreadPeriod() = 0;

  /**
   * @brief Sets the time taken by the thread to memory
   */
  virtual void setThreadTimeTaken() = 0;

  /**
   * @brief onIterationComplete Syncs the local memory variables of the module
   *   with the variables in memory and completes the thread time period loop
   */
  void onIterationComplete();

  /**
   * @brief checkSuspend Checks whether the module should be suspended
   */
  void checkSuspend();

  /**
   * @brief startupInputSync Syncs the local input memory variables of the
   *   module with the variables in memory
   */
  void startupInputSync();

  /**
   * @brief startupOutputSync Syncs the local output memory variables of
   *   the module with the variables in memory
   */
  void startupOutputSync();

  /**
   * @brief setIterationStartTime Sets the iteration start time for
   *   the given module
   */
  void setIterationStartTime()
  {
    iterationStartTime = high_resolution_clock::now();
  }

  /**
   * @brief createThread Creates the module thread
   */
  void createThread() throw (ThreadException);

  /**
   * @brief threadFunc Callback wrapper function for initializing the thread
   *
   * @param void* pTr a pointer to self
   * @return void pointer
   */
  static void* threadFunc(void* pTr);

  static vector<BaseModule*> runningModules; ///< A vector of all the constructed modules
  high_resolution_clock::time_point iterationStartTime; ///< Time at the start of iteration
  int periodMinMS = {0}; ///< Time period of the module in milliseconds
  void* parent; ///< Pointer to parent
  TNSPLModules moduleId; ///< Module identity
  string moduleName; ///< Module name
  double moduleTime = {0.0}; ///< Module time
  size_t inRequestsMax; ///< Max requests possible in the queue
  SharedMemoryPtr sharedMemory; ///< Pointer to local data memory object
  bool parentSuspendFlag = {false}; ///< Module suspension ordered by parent
  bool suspendFlag = {false}; ///< Module suspension ordered by itself
  bool terminateThread = {false}; ///< Whether to stop the thread
  pthread_mutex_t terminateMutex; ///< Module thread mutex
  pthread_mutex_t suspendMutex; ///< Module thread mutex
  pthread_cond_t resumeCond; ///< Module thread condition variable
  pthread_mutex_t condWaitMutex; ///< Mutex conditional wait
  pthread_t threadId; ///< Module thread id defined by pthread_t
};

typedef boost::shared_ptr<BaseModule> BaseModulePtr;
