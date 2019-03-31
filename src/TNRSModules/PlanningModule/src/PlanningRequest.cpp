/**
 * @file PlanningModule/src/PlanningRequest.cpp
 *
 * This file implements the class PlanningRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "PlanningModule/include/PlanningRequest.h"

DEFINE_MODULE_REQUEST(
  PlanningRequest, ModuleRequest, PlanningRequestPtr,
  (PlanningRequestIds, behaviorRequest, RequestPlanningBehavior),
  (PlanningRequestIds, killBehavior, KillPlanningBehavior),
)

