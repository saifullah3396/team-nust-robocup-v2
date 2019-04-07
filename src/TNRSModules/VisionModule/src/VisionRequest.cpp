/**
 * @file VisionModule/src/VisionRequest.cpp
 *
 * This file implements the class VisionRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "VisionModule/include/VisionRequest.h"

DEFINE_MODULE_REQUEST(
  VisionRequest, ModuleRequest, VisionRequestPtr,
  (VisionRequestIds, switchVision, SwitchVision),
  (VisionRequestIds, switchVideoWriter, SwitchVideoWriter),
  (VisionRequestIds, switchFieldProjection, SwitchFieldProjection),
  (VisionRequestIds, switchLogImages, SwitchLogImages),
  (VisionRequestIds, switchUseLoggedImages, SwitchUseLoggedImages),
  (VisionRequestIds, switchFeatureExtModule, SwitchFeatureExtModule),
);
