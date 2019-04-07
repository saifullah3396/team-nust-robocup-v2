/**
 * @file GBModule/include/GBRequest.h
 *
 * This file defines the class GBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "BehaviorManager/include/BehaviorRequest.h"
#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"

struct GBConfig;
typedef boost::shared_ptr<GBConfig> GBConfigPtr;

/**
 * @enum GBRequestIds
 * @brief Types of request valid for GBModule
 */
enum class GBRequestIds {
  stiffnessRequest,
  ledRequest,
  behaviorRequest,
  killBehavior
};

/**
 * @class GBRequest
 * @brief A module request that can be handled by GBModule
 */
DECLARE_MODULE_REQUEST(
  GBRequest,
  GBRequestPtr,
  ModuleRequest,
  TNSPLModules,
  gb,
  GBRequestIds
);

struct RequestGeneralBehavior : public GBRequest, BehaviorRequest
{
  /**
   * Constructor
   */
  RequestGeneralBehavior(const GBConfigPtr& config = GBConfigPtr()) :
    GBRequest(GBRequestIds::behaviorRequest),
    BehaviorRequest(config)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!GBRequest::assignFromJson(obj))
      return false;
    return true;
  }

  /**
   * @brief getJson Makes a json object from request paramters
   * @return Json object
   */
  virtual Json::Value getJson() {
    Json::Value obj = GBRequest::getJson();
    return obj;
  }
};
typedef boost::shared_ptr<RequestGeneralBehavior> RequestGeneralBehaviorPtr;

/**
 * @class KillGeneralBehavior
 * @brief A request to kill the running general behavior
 */
DECLARE_MODULE_REQUEST_TYPE(
  KillGeneralBehavior,
  KillGeneralBehaviorPtr,
  GBRequest,
  GBRequestIds,
  killBehavior
);
