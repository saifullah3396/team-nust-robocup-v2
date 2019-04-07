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
#include "TNRSBase/include/ModuleRequestMacros.h"
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
DECLARE_MODULE_REQUEST(
  PlanningRequest,
  PlanningRequestPtr,
  ModuleRequest,
  TNSPLModules,
  planning,
  PlanningRequestIds
);

struct RequestPlanningBehavior : public PlanningRequest, BehaviorRequest
{
  /**
   * Constructor
   */
  RequestPlanningBehavior(const PBConfigPtr& config = PBConfigPtr()) :
    PlanningRequest(PlanningRequestIds::behaviorRequest),
    BehaviorRequest(config)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!PlanningRequest::assignFromJson(obj))
      return false;
    return true;
  }

  /**
   * @brief getJson Makes a json object from request paramters
   * @return Json object
   */
  virtual Json::Value getJson() {
    Json::Value obj = PlanningRequest::getJson();
    return obj;
  }
};
typedef boost::shared_ptr<RequestPlanningBehavior> RequestPlanningBehaviorPtr;

/**
 * @class KillPlanningBehavior
 * @brief A request to kill the running Planning behavior
 */
DECLARE_MODULE_REQUEST_TYPE(
  KillPlanningBehavior,
  KillPlanningBehaviorPtr,
  PlanningRequest,
  PlanningRequestIds,
  killBehavior
);
