/**
 * @file MotionModule/include/MotionRequest.h
 *
 * This file defines the class MotionRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "BehaviorManager/include/BehaviorRequest.h"

/**
 * Types of request valid for MotionModule
 *
 * @enum MotionRequestIds
 */
enum class MotionRequestIds {
  jointRequest,
  handsRequest,
  behaviorRequest,
  killBehavior,
  killBehaviors
};

/**
 * @class MotionRequest
 * @brief A module request that can be handled by MotionModule
 */
DECLARE_MODULE_REQUEST(
  MotionRequest,
  MotionRequestPtr,
  ModuleRequest,
  TNSPLModules,
  motion,
  MotionRequestIds
);

struct RequestMotionBehavior : public MotionRequest, BehaviorRequest
{
  /**
   * Constructor
   */
  RequestMotionBehavior(const unsigned& mbManagerId = 0, const MBConfigPtr& config = MBConfigPtr()) :
    MotionRequest(MotionRequestIds::behaviorRequest),
    BehaviorRequest(config), mbManagerId(mbManagerId)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!MotionRequest::assignFromJson(obj))
      return false;
    try {
      FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (unsigned, mbManagerId, 0))
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Exception caught in RequestMotionBehavior: ");
      LOG_EXCEPTION(e.what());
      return false;
    }
    return true;
  }

  /**
   * @brief getJson Makes a json object from request paramters
   * @return Json object
   */
  virtual Json::Value getJson() {
    Json::Value obj = MotionRequest::getJson();
    try {
      FOR_EACH(GET_JSON_VAR_1, (unsigned, mbManagerId, 0));
    } catch (Json::Exception& e) {
     LOG_EXCEPTION(
       "Exception caught in module request ModuleRequest::RequestMotionBehavior: ")
      LOG_EXCEPTION(e.what());
    }
    return obj;
  }
  unsigned mbManagerId;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef boost::shared_ptr<RequestMotionBehavior> RequestMotionBehaviorPtr;

/**
 * @class KillMotionBehavior
 * @brief A request to kill a motion behavior
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  KillMotionBehavior,
  KillMotionBehaviorPtr,
  MotionRequest,
  MotionRequestIds,
  killBehavior,
  (unsigned, mbManagerId, 0),
);

/**
 * @class KillMotionBehaviors
 * @brief A request to kill all motion behaviors
 */
DECLARE_MODULE_REQUEST_TYPE(
  KillMotionBehaviors,
  KillMotionBehaviorsPtr,
  MotionRequest,
  MotionRequestIds,
  killBehaviors
);
