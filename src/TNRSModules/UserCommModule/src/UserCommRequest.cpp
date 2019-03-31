/**
 * @file UserCommModule/src/UserCommRequest.cpp
 *
 * This file implements the class UserCommRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "UserCommModule/include/UserCommRequest.h"

DEFINE_MODULE_REQUEST(
  UserCommRequest, ModuleRequest, UserCommRequestPtr,
  (UserCommRequestIds, sendMsgRequest, SendMsgRequest),
  (UserCommRequestIds, sendImageRequest, SendImageRequest),
)

