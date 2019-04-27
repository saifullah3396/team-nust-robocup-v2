/**
 * @file LocalizationModule/include/LocalizationRequest.h
 *
 * This file defines the class LocalizationRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"
#include "Utils/include/SwitchRequest.h"
#include "Utils/include/DataHolders/PositionInput.h"
#include "Utils/include/DataHolders/VelocityInput.h"
#include "Utils/include/DataHolders/Landmark.h"

/**
 * Types of request valid for LocalizationModule
 *
 * @enum LocalizationRequestIds
 */
enum class LocalizationRequestIds {
  switchLocalization,
  switchParticleFilter,
  switchFieldMap,
  switchBallObstacle,
  initiateLocalizer,
  resetLocalizer,
  positionUpdate,
  velocityUpdate,
  knownLandmarksUpdate,
  unknownLandmarksUpdate
};

/**
 * @class LocalizationRequest
 * @brief A module request that can be handled by LocalizationModule
 */
DECLARE_MODULE_REQUEST(
  LocalizationRequest,
  LocalizationRequestPtr,
  ModuleRequest,
  TNSPLModules,
  localization,
  LocalizationRequestIds
);

/**
 * @class SwitchLocalization
 * @brief A request to switch on or  off the Localization module
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchLocalization,
  SwitchLocalizationPtr,
  LocalizationRequest,
  LocalizationRequestIds,
  switchLocalization
);

/**
 * @class SwitchParticleFilter
 * @brief A request to switch on and off the particle filter update
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchParticleFilter,
  SwitchParticleFilterPtr,
  LocalizationRequest,
  LocalizationRequestIds,
  switchParticleFilter
);

/**
 * @class SwitchFieldMap
 * @brief A request to switch on and off the field map update
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchFieldMap,
  SwitchFieldMapPtr,
  LocalizationRequest,
  LocalizationRequestIds,
  switchFieldMap
);

/**
 * @class SwitchBallObstacle
 * @brief A request to switch on and off the ball to be used as obstacle
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchBallObstacle,
  SwitchBallObstaclePtr,
  LocalizationRequest,
  LocalizationRequestIds,
  switchBallObstacle
);

/**
 * @class InitiateLocalizer
 * @brief A request to initiate the localizer of the robot
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  InitiateLocalizer,
  InitiateLocalizerPtr,
  LocalizationRequest,
  LocalizationRequestIds,
  initiateLocalizer,
  (RobotPose2D<float>, pose2D, RobotPose2D<float>(0.0, 0.0, 0.0))
);

/**
 * @class ResetLocalizer
 * @brief A request to reset localizer of the robot
 */
DECLARE_MODULE_REQUEST_TYPE(
  ResetLocalizer,
  ResetLocalizerPtr,
  LocalizationRequest,
  LocalizationRequestIds,
  resetLocalizer
);

/**
 * @class PositionUpdate
 * @brief A request to update the position of the robot
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  PositionUpdate,
  PositionUpdatePtr,
  LocalizationRequest,
  LocalizationRequestIds,
  positionUpdate,
  (PositionInput<float>, input, PositionInput<float>(0.0, 0.0, 0.0))
);

/**
 * @class VelocityUpdate
 * @brief A request to update the velocity of the robot estimates
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  VelocityUpdate,
  VelocityUpdatePtr,
  LocalizationRequest,
  LocalizationRequestIds,
  velocityUpdate,
  (VelocityInput<float>, input, VelocityInput<float>(0.0, 0.0, 0.0))
);

/**
 * @class KnownLandmarksUpdate
 * @brief A request to update the observed known landmarks for the localizer
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  KnownLandmarksUpdate,
  KnownLandmarksUpdatePtr,
  LocalizationRequest,
  LocalizationRequestIds,
  knownLandmarksUpdate,
  (vector<boost::shared_ptr<KnownLandmark<float> > >, landmarks, vector<boost::shared_ptr<KnownLandmark<float> > >())
);


/**
 * @class UnknownLandmarksUpdate
 * @brief A request to update the observed unknown landmarks for the localizer
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  UnknownLandmarksUpdate,
  UnknownLandmarksUpdatePtr,
  LocalizationRequest,
  LocalizationRequestIds,
  unknownLandmarksUpdate,
  (vector<boost::shared_ptr<UnknownLandmark<float> > >, landmarks, vector<boost::shared_ptr<UnknownLandmark<float> > >())
);
