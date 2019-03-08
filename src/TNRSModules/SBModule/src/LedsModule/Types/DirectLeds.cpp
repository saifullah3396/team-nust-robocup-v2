/**
 * @file SBModule/src/LedsModule/Types/DirectLeds.cpp
 *
 * This file implements the class DirectLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "TNRSBase/include/BaseModule.h"
#include "SBModule/include/LedsModule/Types/DirectLeds.h"
#include "Utils/include/Behaviors/SBConfigs/SBLedsConfig.h"

DirectLeds::DirectLeds(
  SBModule* sbModule,
  const boost::shared_ptr<SBLedsConfig>& config) :
  LedsModule(sbModule, config, "DirectLeds")
{
}

bool DirectLeds::initiate()
{
  LOG_INFO("DirectLeds.initiate() called...")
  return true;
}

void DirectLeds::update() // called once
{
  auto& inToReach = this->getBehaviorCast()->inToReach;
  vector<float> outLeds = ledRequest->getValue();
  for (int i = 0; i < toUType(LedActuators::count); ++i) {
    if (inToReach[i] != inToReach[i]) continue; // NAN
    outLeds[i] = inToReach[i];
  }
  ledRequest->setValue(outLeds);
  BaseModule::publishModuleRequest(ledRequest);
  finish();
}

void DirectLeds::finish()
{
  LOG_INFO("DirectLeds.finish() called...")
  inBehavior = false;
}
