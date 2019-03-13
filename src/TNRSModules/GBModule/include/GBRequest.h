/**
 * @file GBModule/include/GBRequest.h
 *
 * This file defines the class GBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "TNRSBase/include/ModuleRequest.h"
#include "BehaviorManager/include/BehaviorRequest.h"

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
class GBRequest : public ModuleRequest 
{
public:
  /**
   * @brief GBRequest Constructor
   * 
   * @param id Request Id
   */ 
  GBRequest(const GBRequestIds& id);
};
typedef boost::shared_ptr<GBRequest> GBRequestPtr;

/**
 * @class RequestGeneralBehavior
 * @brief A request to start a static behavior
 */ 
struct RequestGeneralBehavior : public GBRequest, BehaviorRequest
{
  /**
   * @brief RequestGeneralBehavior Constructor
   * @param config Configuration of requested behavior
   */
  RequestGeneralBehavior(const GBConfigPtr& config);
};
typedef boost::shared_ptr<RequestGeneralBehavior> RequestGeneralBehaviorPtr;

/**
 * @class KillGeneralBehavior
 * @brief A request to kill a static behavior
 */ 
struct KillGeneralBehavior : public GBRequest
{
  /**
   * @brief KillGeneralBehavior Constructor
   */
  KillGeneralBehavior();
};
typedef boost::shared_ptr<KillGeneralBehavior> KillGeneralBehaviorPtr;
