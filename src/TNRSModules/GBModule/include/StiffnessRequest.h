/**
 * @file ControlModule/include/StiffnessRequest.h
 *
 * This file defines the class StiffnessRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#include "GBModule/include/GBRequest.h"

/**
 * @class StiffnessRequest
 * @brief Defines a basic stiffness actuator request
 */
struct StiffnessRequest : public ActuatorRequest, public GBRequest
{
  /**
   * @brief StiffnessRequest Constructor
   */
  StiffnessRequest() :
    ActuatorRequest(static_cast<unsigned>(Joints::count)),
    GBRequest(GBRequestIds::stiffnessRequest)
  {
  }
};
typedef boost::shared_ptr<StiffnessRequest> StiffnessRequestPtr;
