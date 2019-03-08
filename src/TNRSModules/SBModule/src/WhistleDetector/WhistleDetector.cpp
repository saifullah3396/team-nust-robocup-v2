/**
 * @file SBModule/src/WhistleDetector/WhistleDetector.cpp
 *
 * This file implements the class WhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "SBModule/include/SBModule.h"
#include "SBModule/include/WhistleDetector/WhistleDetector.h"
#include "SBModule/include/WhistleDetector/Types/AKWhistleDetector.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "Utils/include/Behaviors/SBConfigs/SBWDConfig.h"

WhistleDetector::WhistleDetector(
SBModule* sbModule,
  const boost::shared_ptr<SBWDConfig>& config,
  const string& name) :
  StaticBehavior(sbModule, config, name)
{
}

boost::shared_ptr<WhistleDetector> WhistleDetector::getType(
  SBModule* sbModule, const BehaviorConfigPtr& cfg) 
{ 
  WhistleDetector* detector;
  switch (cfg->type) {
    case static_cast<unsigned>(SBWDTypes::akWhistleDetector):
      detector = new AKWhistleDetector(sbModule, SPC(SBWDConfig, cfg)); break;
  }
  return WhistleDetectorPtr(detector);
}

SBWDConfigPtr WhistleDetector::getBehaviorCast()
{
  return SPC(SBWDConfig, config);
}

void WhistleDetector::whistleAction()
{
  WHISTLE_DETECTED_OUT(SBModule) = true;
}
