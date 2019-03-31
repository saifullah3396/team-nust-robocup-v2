/**
 * @file GBModule/src/GBRequest.cpp
 *
 * This file implements the class GBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "GBModule/include/GBRequest.h"
#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"

GBRequest::GBRequest(const GBRequestIds& id) :
  ModuleRequest(toUType(TNSPLModules::sb), toUType(id))
{
}

RequestGeneralBehavior::RequestGeneralBehavior(const GBConfigPtr& config) :
  GBRequest(GBRequestIds::behaviorRequest),
  BehaviorRequest(config)
{
}

KillGeneralBehavior::KillGeneralBehavior() :
  GBRequest(GBRequestIds::killBehavior)
{
}
