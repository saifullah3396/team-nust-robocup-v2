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
#include "UserCommModule/include/CommMessage.h"
#include "Utils/include/EnumUtils.h"

using namespace cv;

/**
 * Types of request valid for UserCommModule
 * 
 * @enum UserCommRequestIds
 */ 
enum class UserCommRequestIds {
  SEND_MSG_REQUEST,
  SEND_IMAGE_REQUEST
};

/**
 * @class UserCommRequest
 * @brief A module request that can be handled by UserCommModule
 */ 
class UserCommRequest : public ModuleRequest
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  UserCommRequest(const UserCommRequestIds& id) :
    ModuleRequest(toUType(TNSPLModules::userComm), toUType(id))
  {
  }
};
typedef boost::shared_ptr<UserCommRequest> UserCommRequestPtr;

/**
 * @class SendMsgRequest
 * @brief A request that holds a message to be sent over the network
 */ 
struct SendMsgRequest : public UserCommRequest
{
  /**
   * Constructor
   * 
   * @param cMsg: The comm message associated with this request
   */ 
  SendMsgRequest(const CommMessage& cMsg) :
    UserCommRequest(UserCommRequestIds::SEND_MSG_REQUEST),
    cMsg(cMsg)
  {
  }

  CommMessage cMsg;
};

/**
 * @class SendImageRequest
 * @brief A request that holds an image to be sent over the network
 */
struct SendImageRequest : public UserCommRequest
{
  /**
   * Constructor
   *
   * @param cMsg: The comm message associated with this request
   */
  SendImageRequest(const cv::Mat& image) :
    UserCommRequest(UserCommRequestIds::SEND_IMAGE_REQUEST),
    image(image)
  {
  }

  cv::Mat image;
};
typedef boost::shared_ptr<SendMsgRequest> SendMsgRequestPtr;
