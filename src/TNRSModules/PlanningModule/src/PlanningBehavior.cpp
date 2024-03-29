/**
 * @file PlanningModule/PlanningBehavior.cpp
 *
 * This file implements the base class for all types of planning behaviors.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "PlanningModule/include/PlanningBehavior.h"
#include "Utils/include/PathPlanner/PathPlanner.h"
#include "GBModule/include/GBRequest.h"
#include "MotionModule/include/MotionRequest.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "Utils/include/MathsUtils.h"

bool PlanningBehavior::shutdownCallBack()
{
  static bool shutdown = false, removeStiffness = false;
  if (
    TOUCH_SENSORS_OUT(PlanningModule)[toUType(TouchSensors::headTouchRear)] > 0.1 ||
    TOUCH_SENSORS_OUT(PlanningModule)[toUType(TouchSensors::headTouchMiddle)] > 0.1 ||
    TOUCH_SENSORS_OUT(PlanningModule)[toUType(TouchSensors::headTouchFront)] > 0.1) {
    this->killAllMotionBehaviors();
    this->killGeneralBehavior();
    shutdown = true;
  }
  if (shutdown && !removeStiffness) {
    if (setPostureAndStiffness(PostureState::crouch, StiffnessState::max, 0, 2.0)) {
      removeStiffness = true;
    }
  }

  if (removeStiffness) {
    if (setPostureAndStiffness(PostureState::crouch, StiffnessState::min, 0, 2.0)) {
      this->inBehavior = false;
    }
  }
  return shutdown;
}

bool PlanningBehavior::setPostureAndStiffness(
  const PostureState& desPosture, 
  const StiffnessState& desStiffness,
  const unsigned& mbManagerId,
  const float& postureTime)
{
  if (POSTURE_STATE_IN(PlanningModule) == desPosture &&
      STIFFNESS_STATE_IN(PlanningModule) == desStiffness)
  {
    return true;
  } else if (STIFFNESS_STATE_IN(PlanningModule) != desStiffness) {
    if (!sbInProgress()) {
      auto sConfig =
        boost::make_shared <GBStiffnessConfig> (
          desStiffness);
      setupGBRequest(sConfig);
    }
    return false;
  } else if (POSTURE_STATE_IN(PlanningModule) != desPosture) {
    if (!mbInProgress()) {
      auto pConfig = 
        boost::make_shared<InterpToPostureConfig>();
      pConfig->targetPosture = desPosture;
      pConfig->timeToReachP = postureTime;
      setupMBRequest(mbManagerId, pConfig);
    }
    return false;
  }
}

void PlanningBehavior::killGeneralBehavior() {
  auto request = boost::make_shared<KillGeneralBehavior>();
  BaseModule::publishModuleRequest(request);
}

void PlanningBehavior::killMotionBehavior(const unsigned& mbManagerId) {
  int id = mbManagerId + mbIdOffset;
  auto request = boost::make_shared<KillMotionBehavior>(id);
  BaseModule::publishModuleRequest(request);
}

void PlanningBehavior::killAllMotionBehaviors() {
  for (size_t i = 0; i < mbManagerIds.size(); ++i) {
    auto request = boost::make_shared<KillMotionBehavior>(mbManagerIds[i]);
    BaseModule::publishModuleRequest(request);
  }
}

bool PlanningBehavior::matchLastMotionRequest(
  const unsigned& mbId)
{
  return lastMBConfig &&
         lastMBConfig->id == mbId;
}

bool PlanningBehavior::matchLastMotionRequest(
  const unsigned& mbId, const unsigned& mbType)
{
  return
    lastMBConfig &&
    lastMBConfig->id == mbId &&
    lastMBConfig->type == mbType;
}

bool PlanningBehavior::matchLastStaticRequest(
  const unsigned& sbId, const unsigned& sbType)
{
  return
    lastGBConfig &&
    lastGBConfig->id == sbId &&
    lastGBConfig->type == sbType;
}


void PlanningBehavior::setupGBRequest(const GBConfigPtr& config)
{
  auto rsb = boost::make_shared<RequestGeneralBehavior>(config);
  lastStaticRequest = rsb;
  BaseModule::publishModuleRequest(rsb);
  sRequestTime = pModule->getModuleTime();
}

void PlanningBehavior::setupMBRequest(
  const unsigned& mbManagerId, const MBConfigPtr& config)
{
  int id = mbManagerId + mbIdOffset;
  auto rmb = boost::make_shared<RequestMotionBehavior>(id, config);
  lastMotionRequest = rmb;
  BaseModule::publishModuleRequest(rmb);
  mRequestTime = pModule->getModuleTime();
  if (std::find(mbManagerIds.begin(), mbManagerIds.end(), id) == mbManagerIds.end()) {
    mbManagerIds.push_back(id);
  }
}

bool PlanningBehavior::sbInProgress()
{
  return behaviorInProgress(SB_INFO_IN(PlanningModule));
}

bool PlanningBehavior::mbInProgress()
{
  auto mbInfo = MB_INFO_IN(PlanningModule);
  for (size_t i = 0; i < mbManagerIds.size(); ++i)
  {
    if (mbInfo.find(mbManagerIds[i]) != mbInfo.end()) {
      if (behaviorInProgress(mbInfo[mbManagerIds[i]]))
        return true;
    }
  }
  return false;
}

bool PlanningBehavior::behaviorInProgress(const BehaviorInfo& info)
{
  if(info.isInitiated())
  {
    if (info.isRunning()) 
      return true;
  }
  return false;
}

bool PlanningBehavior::requestInProgress()
{
  bool inProgress = false;
  if(requestInProgress(lastStaticRequest, lastGBConfig, SB_INFO_IN(PlanningModule), sRequestTime))
    inProgress = true;
  auto mbInfo = MB_INFO_IN(PlanningModule);
  for (size_t i = 0; i < mbManagerIds.size(); ++i)
  {
    if (mbInfo.find(mbManagerIds[i]) == mbInfo.end()) {
      mbManagerIds.erase(mbManagerIds.begin()+i);
    } else {
      if (i == mbManagerIds.size()-1) {
        if(requestInProgress(lastMotionRequest, lastMBConfig, mbInfo[mbManagerIds[i]], mRequestTime))
          inProgress = true;
      }
    }
  }
  return inProgress;
}

bool PlanningBehavior::requestInProgress(
  BehaviorRequestPtr& req,
  BehaviorConfigPtr& acceptedBehavior,
  const BehaviorInfo& feedback,
  const float& requestStartTime)
{
  if (!req)
    return false;
    
  if (
    pModule->getModuleTime() - requestStartTime >
    maxRequestTimeout)
  {
    req.reset();
    return false;
  }
    
  if (req->isReceived()) {
    //! Request received on the other end
    if (req->getAccepted()) {
      //cout << "Accepted request" << endl;
      if (feedback.isInitiated() && 
          req->getReqConfig()->id == feedback.getConfig()->id) 
      {
        //cout << "Behavior initiated on request" << endl;
        acceptedBehavior = feedback.getConfig();
        req.reset();
        return false;
      }
      return true;
    } else {
      acceptedBehavior.reset();
      req.reset();
    }
    return false;
  }
  return true;
}

PathPlannerSpace::PathPlannerPtr PlanningBehavior::getPathPlanner()
{
  return pModule->getPathPlanner();
}

