/**
 * @file TNRSBase/src/BaseModule.cpp
 *
 * This file implements the class BaseModule and helper class
 * ThreadException
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/BaseModule.h"
#include "Utils/include/PrintUtils.h"
#include "Utils/include/DebugUtils.h"

vector<BaseModule*> BaseModule::runningModules;

ThreadException::ThreadException(const string& sMessage,
  const bool& blSysMsg) throw () :
  sMsg(sMessage)
{
  if (blSysMsg) {
    sMsg.append(": ");
  }
}

ThreadException::~ThreadException() throw ()
{
}

BaseModule::BaseModule(
  void* parent,
  const TNSPLModules& moduleId,
  const string& moduleName,
  const size_t& inRequestsMax) :
  MemoryBase(),
  parent(parent),
  moduleId(moduleId),
  moduleName(moduleName),
  inRequestsMax(inRequestsMax)
{
  pthread_mutex_init(&terminateMutex, NULL);
  pthread_mutex_init(&suspendMutex, NULL);
  pthread_cond_init(&resumeCond, NULL);
  runningModules.push_back(this);
}

BaseModule::~BaseModule()
{
  pthread_cond_destroy(&resumeCond);
  pthread_mutex_destroy(&terminateMutex);
  pthread_mutex_destroy(&suspendMutex);
}

void BaseModule::setup()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  initMemoryConn();
  startupInputSync();
  init();
  startupOutputSync();
}

void BaseModule::start()
{
  createThread();
}

void BaseModule::join()
{
  int rc = pthread_join(threadId, NULL);
  LOG_INFO(getModuleName() << " thread joined.");
  if (rc != 0) {
    throw ThreadException("Error in thread join... +(pthread_join())", true);
  }
}

void* BaseModule::threadFunc(void* pTr)
{
  BaseModule* pThis = static_cast<BaseModule*>(pTr);
  try {
     while (pThis->upperRoutine()) {}
     throw ThreadException(
       pThis->getModuleName() + " thread routine finished...",
       true);
  } catch (const ThreadException& e) {
    LOG_EXCEPTION(e.what());
    pthread_exit(0);
  }
  return NULL;
}

bool BaseModule::upperRoutine()
{
  pthread_mutex_lock(&terminateMutex);
  bool t = terminateThread;
  pthread_mutex_unlock(&terminateMutex);
  if (!t) {
    setIterationStartTime();
    if (inputConnector) {
      inputConnector->syncFromMemory();
    }
    handleRequests();
    mainRoutine();
    if (outputConnector)
      outputConnector->syncToMemory();
    onIterationComplete();
    return true;
  } else {
    return false;
  }
}

void BaseModule::createThread() throw (ThreadException)
{
  int rc = pthread_create(&threadId, NULL, threadFunc, this);
  if (rc != 0) {
    throw ThreadException(
      "Error in thread creation... (pthread_create())",
      true);
  }
}

void BaseModule::suspend(const bool& parentCommand)
{
  pthread_mutex_lock(&suspendMutex);
  if (parentCommand) {
    parentSuspendFlag = 1;
  } else {
    suspendFlag = 1;
  }
  pthread_mutex_unlock(&suspendMutex);
}

void BaseModule::resume(const bool& parentCommand)
{
  pthread_mutex_lock(&suspendMutex);
  if (parentCommand) {
    parentSuspendFlag = 0;
  } else {
    suspendFlag = 0;
  }
  pthread_cond_broadcast(&resumeCond);
  pthread_mutex_unlock(&suspendMutex);
}

void BaseModule::wait(const bool& condition)
{
  pthread_mutex_lock(&condWaitMutex);
  while (!condition)
    pthread_cond_wait(&resumeCond, &condWaitMutex);
  pthread_mutex_unlock(&condWaitMutex);
}

void BaseModule::terminate()
{
  pthread_mutex_lock(&terminateMutex);
  terminateThread = true;
  pthread_mutex_unlock(&terminateMutex);
}

void BaseModule::checkSuspend()
{
  pthread_mutex_lock(&suspendMutex);
  while (suspendFlag != 0 || parentSuspendFlag != 0)
    pthread_cond_wait(&resumeCond, &suspendMutex);
  pthread_mutex_unlock(&suspendMutex);
}

void BaseModule::startupOutputSync()
{
  if (outputConnector != 0) {
    LOG_INFO("Syncing " + moduleName + " memory output.")
    outputConnector->syncToMemory();
  }
  moduleTime = 0;
  checkSuspend();
}

void BaseModule::startupInputSync()
{
  if (inputConnector != 0) {
    LOG_INFO("Syncing " + moduleName + " memory input.")
    inputConnector->syncFromMemory();
  }
  setThreadPeriod();
}

void BaseModule::onIterationComplete()
{
  auto timeNow = high_resolution_clock::now();
  auto lastIterationTimeMS =
    duration_cast < milliseconds > (timeNow - iterationStartTime).count();
  //if (moduleName == "LocalizationModule")
  //    LOG_INFO(moduleName << " took time: " << lastIterationTimeMS)
  if (lastIterationTimeMS < periodMinMS) {
    int waitTimeMS = periodMinMS - lastIterationTimeMS;
    usleep(waitTimeMS * 1000);
    moduleTime = moduleTime + periodMinMS * 1e-3;
  } else {
    moduleTime = moduleTime + lastIterationTimeMS * 1e-3;
  }
  checkSuspend();
}
