/**
 * @file VisionModule/include/VisionRequest.h
 *
 * This file defines the class VisionRequest
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
#include "Utils/include/HardwareIds.h"
#include "VisionModule/include/FeatureExtractionIds.h"

/**
 * Types of request valid for VisionModule
 *
 * @enum VisionRequestIds
 */
enum class VisionRequestIds {
  switchVision,
  switchVideoWriter,
  switchFieldProjection,
  switchLogImages,
  switchUseLoggedImages,
  switchFeatureExtModule,
  count
};

/**
 * @class VisionRequest
 * @brief A module request that can be handled by VisionModule
 */
DECLARE_MODULE_REQUEST(
  VisionRequest,
  VisionRequestPtr,
  ModuleRequest,
  TNSPLModules,
  vision,
  VisionRequestIds
);

/**
 * @class SwitchVision
 * @brief A request to switch on or  off the vision module
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchVision,
  SwitchVisionPtr,
  VisionRequest,
  VisionRequestIds,
  switchVision
);

/**
 * @class SwitchFieldProjection
 * @brief A request to switch on or  off the video writer
 */
DECLARE_SWITCH_REQUEST_TYPE_WITH_VARS(
  SwitchVideoWriter,
  SwitchVideoWriterPtr,
  VisionRequest,
  VisionRequestIds,
  switchVideoWriter,
  (CameraId, camIndex, CameraId::headTop)
);

/**
 * @class SwitchFieldProjection
 * @brief A request to switch on or off the field projection
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchFieldProjection,
  SwitchFieldProjectionPtr,
  VisionRequest,
  VisionRequestIds,
  switchFieldProjection
);

/**
 * @class SwitchLogImages
 * @brief A request to switch on or  off the image logging
 */
DECLARE_SWITCH_REQUEST_TYPE_WITH_VARS(
  SwitchLogImages,
  SwitchLogImagesPtr,
  VisionRequest,
  VisionRequestIds,
  switchLogImages,
  (CameraId, camIndex, CameraId::headTop)
);

/**
 * @class SwitchLogImages
 * @brief A request to switch on or  off the image logging
 */
DECLARE_SWITCH_REQUEST_TYPE(
  SwitchUseLoggedImages,
  SwitchUseLoggedImagesPtr,
  VisionRequest,
  VisionRequestIds,
  switchUseLoggedImages
);


/**
 * @class SwitchFeatureExtModule
 * @brief A request to switch on and off the usage of feature extraction modules
 */
DECLARE_SWITCH_REQUEST_TYPE_WITH_VARS(
  SwitchFeatureExtModule,
  SwitchFeatureExtModulePtr,
  VisionRequest,
  VisionRequestIds,
  switchFeatureExtModule,
  (FeatureExtractionIds, id, FeatureExtractionIds::segmentation),
);
