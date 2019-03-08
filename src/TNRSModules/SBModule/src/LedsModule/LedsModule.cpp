/**
 * @file SBModule/src/LedsModule/LedsModule.cpp
 *
 * This file implements the class LedsModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "SBModule/include/LedsModule/LedsModule.h"
#include "SBModule/include/LedsModule/Types/DirectLeds.h"
#include "SBModule/include/LedsModule/Types/InterpolateLeds.h"
#include "Utils/include/Behaviors/SBConfigs/SBLedsConfig.h"

LedsModule::LedsModule(
  SBModule* sbModule,
  const boost::shared_ptr<SBLedsConfig>& config,
  const string& name) :
StaticBehavior(sbModule, config, name)
{
  ledRequest = boost::make_shared<LedRequest>();
}

boost::shared_ptr<LedsModule> LedsModule::getType(
  SBModule* sbModule, const BehaviorConfigPtr& cfg) 
{ 
  LedsModule* lm;
  switch (cfg->type) {
      case toUType(SBLedsTypes::directLeds):
        lm = new DirectLeds(sbModule, SPC(SBLedsConfig, cfg)); break;
      case toUType(SBLedsTypes::interpolateLeds):
        lm = new InterpolateLeds(sbModule, SPC(SBLedsConfig, cfg)); break;
  }
  return boost::shared_ptr<LedsModule>(lm);
}

SBLedsConfigPtr LedsModule::getBehaviorCast()
{
  return boost::static_pointer_cast <SBLedsConfig> (config);
}
