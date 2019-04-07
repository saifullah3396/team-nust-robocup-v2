/**
 * @file PlanningModule/src/PlanningBehaviors/ExternalInterface/ExternalInterface.cpp
 *
 * This file declares the class ExternalInterface
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "BehaviorConfigs/include/PBConfigs/PBExternalInterfaceConfig.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/ExternalInterface.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/NIHACognition.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/UserRequestsHandler.h"

boost::shared_ptr<ExternalInterface> ExternalInterface::getType(
  PlanningModule* planningModule, const BehaviorConfigPtr& cfg)
{
  ExternalInterface* cm;
  switch (cfg->type) {
      case toUType(PBExternalInterfaceTypes::nihaCognition):
        cm = new NIHACognition(planningModule, SPC(NIHACognitionConfig, cfg)); break;
      case toUType(PBExternalInterfaceTypes::userReqHandler):
        cm = new UserRequestsHandler(planningModule, SPC(UserReqHandlerConfig, cfg)); break;
  }
  return boost::shared_ptr<ExternalInterface>(cm);
}

PBExternalInterfaceConfigPtr ExternalInterface::getBehaviorCast()
{
  return SPC(PBExternalInterfaceConfig, config);
}
