/**
 * @file PlanningModule/include/PlanningRequest.h
 *
 * This file defines the class PlanningRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "BehaviorManager/include/BehaviorRequest.h"

/**
 * Types of request valid for PlanningModule
 * 
 * @enum PlanningRequestIds
 */ 
enum class PlanningRequestIds {
  behaviorRequest,
  killBehavior,
};

/**
 * @class PlanningRequest
 * @brief A module request that can be handled by PlanningModule
 */ 
class PlanningRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  PlanningRequest(const PlanningRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::planning, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<PlanningRequest> PlanningRequestPtr;

/**
 * @class RequestPlanningBehavior
 * @brief A request to start a planning behavior
 */
struct RequestPlanningBehavior : public PlanningRequest, BehaviorRequest
{
  /**
   * Constructor
   */ 
  RequestPlanningBehavior(const PBConfigPtr& config) :
    PlanningRequest(PlanningRequestIds::behaviorRequest),
    BehaviorRequest(config)
  {
  }
};
typedef boost::shared_ptr<RequestPlanningBehavior> RequestPlanningBehaviorPtr;

/**
 * @class KillPlanningBehavior
 * @brief A request to kill a planning behavior
 */ 
struct KillPlanningBehavior : public PlanningRequest
{
  /**
   * Constructor
   */ 
  KillPlanningBehavior() :
    PlanningRequest(PlanningRequestIds::killBehavior)
  {
  }
};
typedef boost::shared_ptr<KillPlanningBehavior> KillPlanningBehaviorPtr;
