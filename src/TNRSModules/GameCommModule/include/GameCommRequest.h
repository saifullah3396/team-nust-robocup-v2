/**
 * @file GameCommModule/include/GameCommRequest.h
 *
 * This file defines the class GameCommRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"
#include "Utils/include/EnumUtils.h"

using namespace cv;

/**
 * Types of request valid for GameCommModule
 *
 * @enum GameCommRequestIds
 */
enum class GameCommRequestIds {
};

/**
 * @class GameCommRequest
 * @brief Base for all requests handled by UserCommModule
 */
DECLARE_MODULE_REQUEST(
  GameCommRequest,
  GameCommRequestPtr,
  ModuleRequest,
  TNSPLModules,
  gameComm,
  GameCommRequestIds
);

