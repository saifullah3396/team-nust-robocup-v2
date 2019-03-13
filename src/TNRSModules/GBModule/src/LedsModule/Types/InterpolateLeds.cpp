/**
 * @file GBModule/src/LedsModule/Types/InterpolateLeds.cpp
 *
 * This file implements the class InterpolateLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "GBModule/include/GBModule.h"
#include "GBModule/include/LedsModule/Types/InterpolateLeds.h"
#include "TNRSBase/include/BaseModule.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/GBConfigs/GBLedsConfig.h"

InterpolateLeds::InterpolateLeds(
  GBModule* gbModule,
  const boost::shared_ptr<GBLedsConfig>& config) :
  LedsModule(gbModule, config, "InterpolateLeds")
{
}

bool InterpolateLeds::initiate()
{
  LOG_INFO("InterpolateLeds.initiate() called...")
  auto& inToReach = this->getBehaviorCast()->inToReach;
  auto& timeToReachIn = this->getBehaviorCast()->timeToReachIn;
  ledsI = LED_SENSORS_OUT(GBModule);
  ledsDelta = vector<float>(toUType(LedActuators::count), NAN);

  for (int i = 0; i < toUType(LedActuators::count); ++i) {
    if (inToReach[i] != inToReach[i]) continue; // NAN
    ledsDelta[i] = inToReach[i] - ledsI[i];
  }
  return true;
}

void InterpolateLeds::update()
{
  //LOG_INFO("InterpolateLeds.update()")
  auto& inToReach = this->getBehaviorCast()->inToReach;
  auto& timeToReachIn = this->getBehaviorCast()->timeToReachIn;
  if (runTime > timeToReachIn) {
    finish();
  } else {
    //cout << "runTime: " << runTime << endl;
    vector<float> outLeds = ledRequest->getValue();
    for (int i = 0; i < toUType(LedActuators::count); ++i) {
      if (inToReach[i] != inToReach[i]) continue; // NAN
      outLeds[i] = ledsI[i] + ledsDelta[i] * runTime / timeToReachIn;
      //cout << "outLeds[" << i << "]: " << outLeds[i] << endl;
    }
    ledRequest->setValue(outLeds);
    BaseModule::publishModuleRequest(ledRequest);
  }
}

void InterpolateLeds::finish()
{
  LOG_INFO("InterpolateLeds.finish() called...")
  inBehavior = false;
}
