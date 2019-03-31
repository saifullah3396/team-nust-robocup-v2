/**
 * @file ControlModule/include/HandsRequest.h
 *
 * This file defines the class HandsRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#include "MotionModule/include/MotionRequest.h"

using namespace std;

/**
 * @class HandsRequest
 * @brief Defines a basic joint actuation request
 */
struct HandsRequest : public ActuatorRequest, public MotionRequest
{
  /**
   * Constructor
   */
  HandsRequest() :
    ActuatorRequest(toUType(RobotHands::count)),
    MotionRequest(MotionRequestIds::handsRequest)
  {
  }
};
typedef boost::shared_ptr<HandsRequest> HandsRequestPtr;
