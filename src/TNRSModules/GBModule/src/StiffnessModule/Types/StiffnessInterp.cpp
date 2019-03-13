/**
 * @file GBModule/src/StiffnessModule/Types/StiffnessInterp.cpp
 *
 * This file implements the class StiffnessInterp
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "TNRSBase/include/MemoryIOMacros.h"
#include "GBModule/include/GBModule.h"
#include "GBModule/include/StiffnessRequest.h"
#include "GBModule/include/StiffnessModule/Types/StiffnessInterp.h"
#include "Utils/include/MathsUtils.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"

StiffnessInterp::StiffnessInterp(
  GBModule* gbModule,
  const boost::shared_ptr<GBStiffnessConfig>& config) :
  StiffnessModule(gbModule, config, "StiffnessInterp")
{
}

bool StiffnessInterp::initiate()
{
  LOG_INFO("StiffnessInterp.initiate() called...")
  auto& sToReach = getBehaviorCast()->sToReach;
  auto& targetState = getBehaviorCast()->targetState;
  sI = JOINT_STIFFNESSES_OUT(GBModule);
  sDelta = vector<float>(toUType(Joints::count), NAN);
  #ifndef NAOQI_MOTION_PROXY_AVAILABLE
  unsigned removeCnt = 0;
  for (int i = 0; i < toUType(Joints::count); ++i) {
    if (sToReach[i] != sToReach[i]) continue;
    sDelta[i] = sToReach[i] - sI[i];
    if (abs(sDelta[i]) < 0.005) {
      removeCnt++;
      continue;
    }
  }
  if (removeCnt == toUType(Joints::count)) {
    STIFFNESS_STATE_OUT(GBModule) = targetState;
    return false;
  }
  return true;
  #else
  auto& timeToReachS = getBehaviorCast()->timeToReachS;
  vector<unsigned> jointIds;
  for (size_t i = 0; i < toUType(Joints::count); ++i) {
    if (sToReach[i] != sToReach[i]) continue;
    sI[i] = JOINT_STIFFNESSES_OUT(GBModule)[i];
    float diff = sToReach[i] - sI[i];
    if (abs(diff) < 0.005) continue;
    sDelta[i] = diff;
    jointIds.push_back(i);
  }
  size_t size = jointIds.size();
  if (size == 0) {
    LOG_INFO("Stiffness behavior not applicable...")
    STIFFNESS_STATE_OUT(GBModule) = targetState;
    StiffnessInterp::finish();
    return false;
  }

  AL::ALValue jointTimes;
  AL::ALValue jointStiffnesses;
  jointTimes.clear();
  jointStiffnesses.clear();
  jointTimes.arraySetSize(size);
  jointStiffnesses.arraySetSize(size);
  int totalSteps = timeToReachS / cycleTime;
  for (int i = 0; i < size; ++i) {
    jointStiffnesses[i].arraySetSize(totalSteps);
    jointTimes[i].arraySetSize(totalSteps);
  }
  float timeStep = cycleTime;
  for (int j = 0; j < totalSteps; ++j) {
    float timeParam = timeStep / timeToReachS;
    for (int i = 0; i < size; ++i) {
      int jointId = jointIds[i];
      float cStiffness =
        sI[jointId] + sDelta[jointId] * timeParam;
      cStiffness = cStiffness >= 0.99f ? 1.0f : cStiffness;
      cStiffness = cStiffness <= 0.01f ? 0.0f : cStiffness;
      jointStiffnesses[i][j] = cStiffness;
      jointTimes[i][j] = timeStep;
    }
    timeStep += cycleTime;
  }
  naoqiStiffnessInterpolation(jointIds, jointTimes, jointStiffnesses, true);
  return true;
  #endif
}

void StiffnessInterp::update()
{
  auto& timeToReachS = getBehaviorCast()->timeToReachS;
  #ifdef NAOQI_MOTION_PROXY_AVAILABLE
  if (this->runTime > timeToReachS) {
    STIFFNESS_STATE_OUT(GBModule) = getBehaviorCast()->targetState;
    finish();
  }
  #else
  if (this->runTime > timeToReachS) {
    STIFFNESS_STATE_OUT(GBModule) = getBehaviorCast()->targetState;
    finish();
  } else {
    auto& sToReach = getBehaviorCast()->sToReach;
    auto& timeToReachS = getBehaviorCast()->timeToReachS;
    StiffnessRequestPtr sr = boost::make_shared<StiffnessRequest>();
    auto timeParam = this->runTime / timeToReachS;
    for (size_t i = 0; i < toUType(Joints::count); ++i) {
      if (sToReach[i] != sToReach[i]) continue; // NAN
      sr->setValue(sI[i] + sDelta[i] * timeParam, i);
    }
    BaseModule::publishModuleRequest(sr);
  }
  #endif
}

void StiffnessInterp::finish()
{
  LOG_INFO("StiffnessInterp.finish() called...")
  inBehavior = false;
}
