/**
 * @file UserCommModule/include/LolaRequest.h
 *
 * This file defines the class LolaRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"

using namespace cv;

/**
 * Types of request valid for UserCommModule
 *
 * @enum LolaRequestIds
 */
enum class LolaRequestIds {
  jointRequest,
  handsRequest,
  stiffnessRequest,
  ledRequest,
  count
};

/**
 * @class LolaRequest
 * @brief Base for all requests handled by UserCommModule
 */
DECLARE_MODULE_REQUEST(
  LolaRequest,
  LolaRequestPtr,
  ModuleRequest,
  TNSPLModules,
  lola,
  LolaRequestIds
);
