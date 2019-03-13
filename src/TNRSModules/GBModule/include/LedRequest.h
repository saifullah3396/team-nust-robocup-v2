/**
 * @file ControlModule/include/LedRequest.h
 *
 * This file defines the class LedRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#include "GBModule/include/GBRequest.h"

/**
 * @struct LedRequest
 * @brief Defines a basic leds actuation request
 */
struct LedRequest : public ActuatorRequest, public GBRequest
{
  /**
   * @brief LedRequest Constructor
   */
  LedRequest() :
    ActuatorRequest(toUType(LedActuators::count)),
    GBRequest(GBRequestIds::ledRequest)
  {
  }
};
typedef boost::shared_ptr<LedRequest> LedRequestPtr;
