/**
 * @file MotionModule/src/MotionRequest.cpp
 *
 * This file implements the class MotionRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "MotionModule/include/MotionRequest.h"
#include "MotionModule/include/JointRequest.h"
#include "MotionModule/include/HandsRequest.h"

#ifndef V6_CROSS_BUILD
  DEFINE_MODULE_REQUEST(
    MotionRequest, ModuleRequest, MotionRequestPtr,
    (MotionRequestIds, jointRequest, JointRequest),
    (MotionRequestIds, handsRequest, HandsRequest),
    (MotionRequestIds, behaviorRequest, RequestMotionBehavior),
    (MotionRequestIds, killBehavior, KillMotionBehavior),
    (MotionRequestIds, killBehaviors, KillMotionBehaviors),
  );
#else
  #ifndef REALTIME_LOLA_AVAILABLE
  DEFINE_MODULE_REQUEST(
    MotionRequest, ModuleRequest, MotionRequestPtr, // No actuators regardless
    (MotionRequestIds, behaviorRequest, RequestMotionBehavior),
    (MotionRequestIds, killBehavior, KillMotionBehavior),
    (MotionRequestIds, killBehaviors, KillMotionBehaviors),
  );
  #else
  DEFINE_MODULE_REQUEST(
    MotionRequest, ModuleRequest, MotionRequestPtr,
    (MotionRequestIds, behaviorRequest, RequestMotionBehavior),
    (MotionRequestIds, killBehavior, KillMotionBehavior),
    (MotionRequestIds, killBehaviors, KillMotionBehaviors),
  );
  #endif
#endif
