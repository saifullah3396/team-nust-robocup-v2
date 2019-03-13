/**
 * @file GBModule/src/StiffnessModule/StiffnessModule.h
 *
 * This file implements the class StiffnessModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#include "GBModule/include/StiffnessModule/StiffnessModule.h"
#include "GBModule/include/StiffnessModule/Types/StiffnessInterp.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"

StiffnessModule::StiffnessModule(
  GBModule* gbModule,
  const boost::shared_ptr<GBStiffnessConfig>& config,
  const string& name) :
  GeneralBehavior(gbModule, config, name)
{
}

boost::shared_ptr<StiffnessModule> StiffnessModule::getType(
  GBModule* gbModule, 
  const BehaviorConfigPtr& cfg)
{ 
  StiffnessModule* sm;
  switch (cfg->type) {
      case toUType(GBStiffnessTypes::stiffnessInterp):
        sm = new StiffnessInterp(gbModule, SPC(GBStiffnessConfig, cfg)); break;
  }
  return StiffnessModulePtr(sm);
}

GBStiffnessConfigPtr StiffnessModule::getBehaviorCast()
{
  return 
    boost::static_pointer_cast <GBStiffnessConfig> (config);
}
