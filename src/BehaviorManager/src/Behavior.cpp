/**
 * @file BehaviorManager/src/Behavior.cpp
 *
 * This file implements the classes Behavior and BehaviorException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#include <chrono>
#include "BehaviorManager/include/Behavior.h"
#include "Utils/include/JsonLogger.h"
#include "BehaviorManager/include/BehaviorRequest.h"
#include "BehaviorConfigs/include/BehaviorConfig.h"
//#include "Utils/include/ConfigManager.h"

using namespace chrono;

BehaviorException::BehaviorException(
  Behavior* behavior,
  const string& message,
  const bool& bSysMsg) throw () :
  TNRSException(message, bSysMsg),
  name(behavior->getName())
{
}

Behavior::Behavior(
  const BehaviorConfigPtr& config,
  const string& name) :
  name(name),
  config(config)
{
}

void Behavior::manage()
{
  if (!initiated) {
    auto startTime = high_resolution_clock::now();
    //! Setup a data logger if it is needed and not already initialized
    if (config->logData && !dataLogger) {
      //! Setup log dirs
      if (setupLogsDir())
        dataLogger = makeLogger();
      else {
        config->logData = false;
      }
    }
    loadExternalConfig();
    this->inBehavior = initiate();
    initTime =
      static_cast<duration<float>>(
        high_resolution_clock::now() - startTime).count();
    runTime = 0.0;
    //! Warning: initiated is not the same as inBehavior
    initiated = true;
  } else {
    update();
    runTime += cycleTime;
    if (dataLogger) {
      updateDataLogger();
      if (!this->inBehavior) {
        dataLogger->save();
      }
    }
  }
}

void Behavior::setupChildRequest(
  const BehaviorConfigPtr& config,
  const bool& runInParallel)
{
  this->childInParallel = runInParallel;
  childReq = boost::make_shared<BehaviorRequest>(config);
}

void Behavior::onMaxTimeReached()
{
  if (initTime + runTime > config->maxRuntime)
    kill();
}

void Behavior::kill()
{
  //! Kill all the behaviors recursively from child to parent
  if (child)
    child->kill();
  finish();
}

void Behavior::killChild()
{
  if (child)
    child->kill();
}

void Behavior::setLoggerFromParent()
{
  if (parent) {
    this->dataLogger = parent->getDataLogger();
    config->logData = parent->getBehaviorConfig()->logData;
  }
}

JsonLoggerPtr Behavior::makeLogger()
{
  return
    boost::make_shared<Utils::JsonLogger>(
      logsDirPath + "/" + name + ".json");
}

bool Behavior::setupLogsDir()
{
  if (config->logData) {
    logsDirPath = ConfigManager::getLogsDirPath() + name + "/";
    if (!boost::filesystem::exists(logsDirPath)) {
      if(!boost::filesystem::create_directory(logsDirPath))
        return false;
    }

    int num = 0;
    using namespace boost::filesystem;
    for(directory_iterator it(logsDirPath);
        it != directory_iterator(); ++it)
    {
       ++num;
    }
    logsDirPath += "log";
    logsDirPath += DataUtils::varToString(++num);
    boost::filesystem::path dir(logsDirPath);
    if(!boost::filesystem::create_directory(dir))
      return false;
    return true;
  }
  return false;
}
