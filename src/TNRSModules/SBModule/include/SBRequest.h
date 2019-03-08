/**
 * @file SBModule/include/SBRequest.h
 *
 * This file defines the class SBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "TNRSBase/include/ModuleRequest.h"
#include "BehaviorManager/include/BehaviorRequest.h"

struct SBConfig;
typedef boost::shared_ptr<SBConfig> SBConfigPtr;

/**
 * @enum SBRequestIds
 * @brief Types of request valid for SBModule
 */ 
enum class SBRequestIds {
  stiffnessRequest,
  ledRequest,
  behaviorRequest,
  killBehavior
};

/**
 * @class SBRequest
 * @brief A module request that can be handled by SBModule
 */ 
class SBRequest : public ModuleRequest 
{
public:
  /**
   * @brief SBRequest Constructor
   * 
   * @param id Request Id
   */ 
  SBRequest(const SBRequestIds& id);
};
typedef boost::shared_ptr<SBRequest> SBRequestPtr;

/**
 * @class RequestStaticBehavior
 * @brief A request to start a static behavior
 */ 
struct RequestStaticBehavior : public SBRequest, BehaviorRequest
{
  /**
   * @brief RequestStaticBehavior Constructor
   * @param config Configuration of requested behavior
   */
  RequestStaticBehavior(const SBConfigPtr& config);
};
typedef boost::shared_ptr<RequestStaticBehavior> RequestStaticBehaviorPtr;

/**
 * @class KillStaticBehavior
 * @brief A request to kill a static behavior
 */ 
struct KillStaticBehavior : public SBRequest
{
  /**
   * @brief KillStaticBehavior Constructor
   */
  KillStaticBehavior();
};
typedef boost::shared_ptr<KillStaticBehavior> KillStaticBehaviorPtr;
