/**
 * @file LocalizationModule/src/LocalizationRequest.cpp
 *
 * This file implements the class LocalizationRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "LocalizationModule/include/LocalizationRequest.h"

DEFINE_MODULE_REQUEST(
  LocalizationRequest, ModuleRequest, LocalizationRequestPtr,
  (LocalizationRequestIds, switchLocalization, SwitchLocalization),
  (LocalizationRequestIds, switchParticleFilter, SwitchParticleFilter),
  (LocalizationRequestIds, switchFieldMap, SwitchFieldMap),
  (LocalizationRequestIds, switchBallObstacle, SwitchBallObstacle),
  (LocalizationRequestIds, initiateLocalizer, InitiateLocalizer),
  (LocalizationRequestIds, resetLocalizer, ResetLocalizer),
  (LocalizationRequestIds, positionUpdate, PositionUpdate),
  (LocalizationRequestIds, knownLandmarksUpdate, KnownLandmarksUpdate),
  (LocalizationRequestIds, unknownLandmarksUpdate, UnknownLandmarksUpdate),
)
