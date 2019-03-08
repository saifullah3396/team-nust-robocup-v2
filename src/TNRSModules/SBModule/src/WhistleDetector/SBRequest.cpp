/**
 * @file SBModule/src/SBRequest.cpp
 *
 * This file implements the class SBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "SBModule/include/SBRequest.h"
#include "Utils/include/Behaviors/SBConfigs/SBConfig.h"

SBRequest::SBRequest(const SBRequestIds& id) :
  ModuleRequest(toUType(TNSPLModules::sb), toUType(id))
{
}

RequestStaticBehavior::RequestStaticBehavior(const SBConfigPtr& config) :
  SBRequest(SBRequestIds::behaviorRequest),
  BehaviorRequest(config)
{
}

KillStaticBehavior::KillStaticBehavior() :
  SBRequest(SBRequestIds::killBehavior)
{
}
