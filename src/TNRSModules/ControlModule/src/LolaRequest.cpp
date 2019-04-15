/**
 * @file LolaModule/src/LolaRequest.cpp
 *
 * This file implements the class LolaRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "ControlModule/include/LolaRequest.h"
#include "GBModule/include/StiffnessRequest.h"
#include "GBModule/include/LedRequest.h"

#ifdef V6_CROSS_BUILD
DEFINE_MODULE_REQUEST(
  LolaRequest, ModuleRequest, LolaRequestPtr,
  (LolaRequestIds, stiffnessRequest, StiffnessRequest),
  (LolaRequestIds, ledRequest, LedRequest),
);
#endif
