/**
 * @file ControlModule/include/JointRequest.h
 *
 * This file defines the class JointRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#include "MotionModule/include/MotionRequest.h"

using namespace std;

/**
 * @class JointRequest
 * @brief Defines a basic joint actuation request
 */
struct JointRequest : public ActuatorRequest, public MotionRequest
{
  /**
   * Constructor
   */
  JointRequest() :
    ActuatorRequest(toUType(Joints::count)),
    MotionRequest(MotionRequestIds::jointRequest)
  {
  }
};
typedef boost::shared_ptr<JointRequest> JointRequestPtr;
