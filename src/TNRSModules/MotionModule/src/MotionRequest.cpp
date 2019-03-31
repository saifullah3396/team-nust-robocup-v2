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

DEFINE_MODULE_REQUEST(
  MotionRequest, ModuleRequest, MotionRequestPtr,
  (MotionRequestIds, jointRequest, JointRequest),
  (MotionRequestIds, handsRequest, HandsRequest),
  (MotionRequestIds, behaviorRequest, RequestMotionBehavior),
  (MotionRequestIds, killBehavior, KillMotionBehavior),
  (MotionRequestIds, killBehaviors, KillMotionBehaviors),
)

