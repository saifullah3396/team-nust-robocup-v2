/**
 * @file GBModule/src/GBRequest.cpp
 *
 * This file implements the class GBRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "GBModule/include/GBRequest.h"
#include "GBModule/include/StiffnessRequest.h"
#include "GBModule/include/LedRequest.h"

DEFINE_MODULE_REQUEST(
  GBRequest, ModuleRequest, GBRequestPtr,
  (GBRequestIds, stiffnessRequest, StiffnessRequest),
  (GBRequestIds, ledRequest, LedRequest),
  (GBRequestIds, behaviorRequest, RequestGeneralBehavior),
  (GBRequestIds, killBehavior, KillGeneralBehavior),
);

