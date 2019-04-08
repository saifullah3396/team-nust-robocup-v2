/**
 * @file BehaviorManager/src/BehaviorManager.cpp
 *
 * This file implements the class BehaviorManager and BManagerException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "BehaviorManager/include/Behavior.h"
#include "BehaviorManager/include/BehaviorRequest.h"
#include "BehaviorManager/include/BehaviorManager.h"

BManagerException::BManagerException(
  BehaviorManager* behaviorManager,
  const string& message,
  const bool& bSysMsg) throw () :
  TNRSException(message, bSysMsg),
  name(behaviorManager->getName())
{
}

void BehaviorManager::manageRequest(
  const BehaviorRequestPtr& req)
{
  manageRequest(currBehavior, req);
}

void BehaviorManager::manageRequest(
  BehaviorPtr& bPtr,
  const BehaviorRequestPtr& req)
{
  try {
    if (!req->getReqConfig()) {
      throw BManagerException(
        this,
        "Requested behavior has no configuration associated with it.",
        false);
    }
    auto reqConfig = req->getReqConfig();
    ///< Requested behavior configuration is valid or not
    reqConfig->init();
    reqConfig->validate();
    req->setReceived(true);
    ///< Behavior already exists
    if (bPtr) {
      /*if (name == "MBManager") {
        cout << "Behavior already running" << endl;
        cout << "reqConfig->id: " << reqConfig->id << endl;
        cout << "bPtr->getBehaviorConfig()->id: " << bPtr->getBehaviorConfig()->id << endl;
        cout << "reqConfig->type: " << reqConfig->type << endl;
        cout << "bPtr->getBehaviorConfig()->type: " << bPtr->getBehaviorConfig()->type << endl;
      }*/
      ///< Check if the requested behavior is the same as the behavior to
      ///< be managed
      if (reqConfig->id == bPtr->getBehaviorConfig()->id &&
          reqConfig->type == bPtr->getBehaviorConfig()->type)
      {
        //if (name == "PlanningBehavior")
        //  cout << "Requested is same as previous" << endl;
        ///< Reinitiate the current behavior with new requested
        ///< configuration
        bPtr->reinitiate(reqConfig);
        req->setAccepted(true);
        //lastBehaviorAccepted.id = req->id;
        //lastBehaviorAccepted.state =
        //  boost::make_shared < BehaviorState > (bId, this->behavior->getState());
        //updateBehaviorState();
      }
    } else {
      ///< Set up the current behavior with the requested configuration
      if(setupBehavior(bPtr, reqConfig)) {
        req->setAccepted(true);
      }
    }
  } catch (BManagerException& e) {
    LOG_EXCEPTION(e.what())
    req->setReceived(true);
    return;
  } catch (BConfigException& e) {
    LOG_EXCEPTION(e.what())
    req->setReceived(true);
    return;
  }
  //lastBehaviorAccepted.id = req->id;
  //cout << "Last behavior id: " << lastBehaviorAccepted.id << endl;
  //lastBehaviorAccepted.state =
  //  boost::make_shared < BehaviorState > (bId, this->behavior->getState());
  //updateBehaviorState();
}

bool BehaviorManager::setupBehavior(
  BehaviorPtr& bPtr,
  const BehaviorConfigPtr& cfg) throw (BManagerException)
{
  try {
  ///< Make a behavior based on the abstract function call
    if (!makeBehavior(bPtr, cfg)) {
      throw BManagerException(
        this,
        "Unable to construct the behavior with requested configuration.",
        false);
    }
  } catch (BManagerException& e) {
    LOG_EXCEPTION(e.what())
    return false;
  }
  //bPtr->loadExternalConfig();
  return true;
}

void BehaviorManager::update()
{
  ///< Kill the behavior if requested
  if (killRequested) {
    if (currBehavior) {
      currBehavior->kill();
    }
    killRequested = false;
  }

  ///< If a current behavior exists
  if (currBehavior) {
    behaviorInfo->clearConfigs();
    ///< Update the behavior
    updateBehavior(currBehavior);
    ////lastBehaviorAccepted.state->finished = true;
    ////updateBehaviorState();
    setBehaviorInfo(currBehavior);
  }
}

void BehaviorManager::updateBehavior(BehaviorPtr& bPtr)
{
  behaviorInfo->addConfig(bPtr->getBehaviorConfig());
  ///< Check if top behavior time limit is reached and kill it along
  ///< with all its childs if true
  bPtr->onMaxTimeReached();

  ///< Get pointer to child
  auto& child = bPtr->getChild();

  ///< Setup a new child if a new child request is found
  ///< This call does nothing if there is no request
  auto& childReq = bPtr->getChildRequest();
  if (childReq) {
    manageRequest(child, bPtr->getChildRequest());
    if (child) {
      bPtr->setLastChildCfg(bPtr->getChildRequest()->getReqConfig());
      child->setParent(bPtr);
      child->setLoggerFromParent();
    }
    childReq.reset();
  }

  ///< If a child exists now
  if (child) {
    ///< Update the child
    updateBehavior(child);
    if (!bPtr->getChildInParallel())
      return;
  } else {
    bPtr->setChildInParallel(false);
  }

  ///< Initiate if not, update otherwise
  bPtr->manage();

  ///< If behavior is initiated but still not valid for update
  if (bPtr->isInitiated() && !bPtr->isRunning()) {
    bPtr.reset(); ///< Behavior is finished so delete it
  }
}

void BehaviorManager::setBehaviorInfo(const BehaviorPtr& bPtr)
{
  if (bPtr) {
    behaviorInfo->setName(bPtr->getName());
    behaviorInfo->setFsmState(bPtr->getFsmState());
    behaviorInfo->setInitiated(bPtr->isInitiated());
    behaviorInfo->setRunning(bPtr->isRunning());
    behaviorInfo->setPaused(bPtr->isPaused());
    behaviorInfo->setConfig(bPtr->getBehaviorConfig());
  } else {
    if (behaviorInfo->isInitiated()) {
      behaviorInfo->setFinished(true);
      behaviorInfo->setRunning(false);
      behaviorInfo->setPaused(false);
    } else {
      behaviorInfo->setInitiated(false);
      behaviorInfo->setRunning(false);
      behaviorInfo->setPaused(false);
      behaviorInfo->resetConfig();
    }
  }
}
