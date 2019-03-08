/**
 * @file PlanningModule/src/PlanningBehaviors/RobotStartup/RobotStartup.cpp
 *
 * This file implements the class RobotStartup
 *
 * @author <A href="mailto:saifullah3396@rsail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "PlanningModule/include/PlanningBehaviors/RobotStartup/RobotStartup.h"
#include "PlanningModule/include/PlanningBehaviors/RobotStartup/Types/RequestBehavior.h"
#include "Utils/include/Behaviors/PBConfigs/PBStartupConfig.h"

RobotStartup::RobotStartup(
  PlanningModule* planningModule,
  const boost::shared_ptr<PBStartupConfig>& config,
  const string& name) :
  PlanningBehavior(planningModule, config, name)
{
}

boost::shared_ptr<RobotStartup> RobotStartup::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg) 
{ 
  RobotStartup* rs;
  switch (cfg->type) {
      case toUType(PBStartupTypes::requestBehavior):
        rs =
            new RequestBehavior(
              planningModule,
              boost::static_pointer_cast<RequestBehaviorConfig>(cfg));
        break;
  }
  return boost::shared_ptr<RobotStartup>(rs);
}

PBStartupConfigPtr RobotStartup::getBehaviorCast()
{
  return boost::static_pointer_cast <PBStartupConfig> (config);
}
