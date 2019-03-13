/**
 * @file GBModule/src/WhistleDetector/WhistleDetector.cpp
 *
 * This file implements the class WhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "GBModule/include/GBModule.h"
#include "GBModule/include/WhistleDetector/WhistleDetector.h"
#include "GBModule/include/WhistleDetector/Types/AKWhistleDetector.h"
#include "TNRSBase/include/MemoryIOMacros.h"
#include "BehaviorConfigs/include/GBConfigs/GBWDConfig.h"

WhistleDetector::WhistleDetector(
  GBModule* gbModule,
  const boost::shared_ptr<GBWDConfig>& config,
  const string& name) :
  GeneralBehavior(gbModule, config, name)
{
}

boost::shared_ptr<WhistleDetector> WhistleDetector::getType(
  GBModule* gbModule, const BehaviorConfigPtr& cfg) 
{ 
  WhistleDetector* detector;
  switch (cfg->type) {
    case static_cast<unsigned>(GBWDTypes::akWhistleDetector):
      detector = new AKWhistleDetector(gbModule, SPC(GBWDConfig, cfg)); break;
  }
  return WhistleDetectorPtr(detector);
}

GBWDConfigPtr WhistleDetector::getBehaviorCast()
{
  return SPC(GBWDConfig, config);
}

void WhistleDetector::whistleAction()
{
  WHISTLE_DETECTED_OUT(GBModule) = true;
}
