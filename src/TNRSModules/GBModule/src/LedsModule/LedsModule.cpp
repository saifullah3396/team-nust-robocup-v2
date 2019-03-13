/**
 * @file GBModule/src/LedsModule/LedsModule.cpp
 *
 * This file implements the class LedsModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "GBModule/include/LedsModule/LedsModule.h"
#include "GBModule/include/LedsModule/Types/DirectLeds.h"
#include "GBModule/include/LedsModule/Types/InterpolateLeds.h"
#include "BehaviorConfigs/include/GBConfigs/GBLedsConfig.h"

LedsModule::LedsModule(
  GBModule* gbModule,
  const boost::shared_ptr<GBLedsConfig>& config,
  const string& name) :
GeneralBehavior(gbModule, config, name)
{
  ledRequest = boost::make_shared<LedRequest>();
}

boost::shared_ptr<LedsModule> LedsModule::getType(
  GBModule* gbModule, const BehaviorConfigPtr& cfg) 
{ 
  LedsModule* lm;
  switch (cfg->type) {
      case toUType(GBLedsTypes::directLeds):
        lm = new DirectLeds(gbModule, SPC(GBLedsConfig, cfg)); break;
      case toUType(GBLedsTypes::interpolateLeds):
        lm = new InterpolateLeds(gbModule, SPC(GBLedsConfig, cfg)); break;
  }
  return boost::shared_ptr<LedsModule>(lm);
}

GBLedsConfigPtr LedsModule::getBehaviorCast()
{
  return boost::static_pointer_cast <GBLedsConfig> (config);
}
