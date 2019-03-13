/**
 * @file PlanningBehaviors/RobotStartup/Types/RequestBehavior.cpp
 *
 * This file implements the class RequestBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "PlanningModule/include/PlanningRequest.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/Types/RequestBehavior.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/PBConfigs/PBStartupConfig.h"

RequestBehavior::RequestBehavior(
  PlanningModule* planningModule,
  const boost::shared_ptr<RequestBehaviorConfig>& config) :
  RobotStartup(planningModule, config, "RequestBehavior")
{
  DEFINE_FSM_STATE(RequestBehavior, SetPosture, setPosture)
  DEFINE_FSM_STATE(RequestBehavior, ChestButtonWait, chestButtonWait)
  DEFINE_FSM_STATE(RequestBehavior, StartRequested, startRequested)
  DEFINE_FSM(fsm, RequestBehavior, setPosture)
}

bool RequestBehavior::initiate()
{
  LOG_INFO("RequestBehavior.initiate() called...");
  setStartPosture();
  return true;
}

void RequestBehavior::update() 
{
  if (requestInProgress()) return;
  if (shutdownCallBack()) return;
  if (fsm->update())
    finish();
}

void RequestBehavior::finish()
{
  LOG_INFO("RequestBehavior.finish() called...")
  inBehavior = false;
}

RequestBehaviorConfigPtr RequestBehavior::getBehaviorCast()
{
  return boost::static_pointer_cast <RequestBehaviorConfig> (config);
}

void RequestBehavior::setStartPosture()
{
  try {
    string requestedPosture = getBehaviorCast()->requestedPosture;
    if (requestedPosture == "Crouch") {
      startPosture = PostureState::crouch;
    } else if (requestedPosture == "Sit") {
      startPosture = PostureState::sit;
    } else if (requestedPosture == "StandZero") {
      startPosture = PostureState::standZero;
    } else if (requestedPosture == "Stand") {
      startPosture = PostureState::stand;
    } else if (requestedPosture == "StandHandsBehind") {
      startPosture = PostureState::standHandsBehind;
    } else if (requestedPosture == "StandKick") {
      startPosture = PostureState::standKick;
    } else {
      throw 
      BehaviorException(
        this,
        "Invalid start posture requested. See " +
        ConfigManager::getPBConfigsPath() +
        "RobotStartup/RequestBehavior.json.",
        true
      );
    } 
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
    startPosture = PostureState::unknown;
  }
}

void RequestBehavior::SetPosture::onRun() {
  if (bPtr->startPosture == PostureState::unknown)
    nextState = bPtr->chestButtonWait.get();
  else {
    if (bPtr->setPostureAndStiffness(bPtr->startPosture, StiffnessState::max, MOTION_1))
      nextState = bPtr->chestButtonWait.get();
  }
}

void RequestBehavior::ChestButtonWait::onRun() {
  #if defined(MODULE_IS_REMOTE) || defined(MODULE_IS_LOCAL_SIMULATED)
    static float wait = 0.f;
    if (wait > bPtr->getBehaviorCast()->startWaitTime) {
      nextState = bPtr->startRequested.get();
    }
    wait += bPtr->cycleTime;
  #else
  if (SWITCH_SENSORS_OUT_REL(PlanningModule, bPtr)[toUType(SwitchSensors::chestBoardButton)] > 0.1)
    nextState = bPtr->startRequested.get();
  #endif
}

void RequestBehavior::StartRequested::onRun() {
  try {
    //! Read the json configuration
    using namespace std;
    string jsonConfigPath;
    jsonConfigPath =
      ConfigManager::getPBConfigsPath() +
        bPtr->getBehaviorCast()->requestedBehavior;
    Json::Value json;
    Json::Reader reader;
    ifstream config(jsonConfigPath, ifstream::binary);
    bool success = reader.parse(config, json, false);
    if (!success) {
      throw
      BehaviorException(
        bPtr,
        "Unable to parse json config:\n\t" +
        jsonConfigPath,
        true
      );
    }
    PBConfigPtr planningConfig =
      boost::static_pointer_cast<PBConfig>(
        BehaviorConfig::makeFromJson(json));
    if (planningConfig) {
      PlanningRequestPtr request =
        boost::make_shared<RequestPlanningBehavior>(planningConfig);
      //! Add in queue twice since first one is disregarded due to this
      //! behavior already running
      BaseModule::publishModuleRequest(request);
      BaseModule::publishModuleRequest(request);
    }
  } catch (Json::Exception& e) {
    LOG_EXCEPTION(e.what());
  } catch (BehaviorException& e) {
    LOG_EXCEPTION(e.what());
  }
  nextState = nullptr;
}
