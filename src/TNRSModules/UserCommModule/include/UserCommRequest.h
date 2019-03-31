/**
 * @file UserCommModule/include/UserCommRequest.h
 *
 * This file defines the class UserCommRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"
#include "Utils/include/DataHolders/CommMessage.h"
#include "Utils/include/EnumUtils.h"

using namespace cv;

/**
 * Types of request valid for UserCommModule
 *
 * @enum UserCommRequestIds
 */
enum class UserCommRequestIds {
  sendMsgRequest,
  sendImageRequest
};

/**
 * @class UserCommRequest
 * @brief Base for all requests handled by UserCommModule
 */
DECLARE_MODULE_REQUEST(
  UserCommRequest,
  UserCommRequestPtr,
  ModuleRequest,
  TNSPLModules,
  userComm,
  UserCommRequestIds
)

/**
 * @class SendMsgRequest
 * @brief A request that holds a message to be sent over the network
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  SendMsgRequest,
  SendMsgRequestPtr,
  UserCommRequest,
  UserCommRequestIds,
  sendMsgRequest,
  (CommMessage, cMsg, CommMessage()),
)

/**
 * @class SendImageRequest
 * @brief A request that holds an image to be sent over the network
 */
DECLARE_MODULE_REQUEST_TYPE_WITH_VARS(
  SendImageRequest,
  SendImageRequestPtr,
  UserCommRequest,
  UserCommRequestIds,
  sendImageRequest,
  (cv::Mat, image, cv::Mat()),
)
