/**
 * @file MotionModule/include/JointRequest.h
 *
 * This file defines the class JointRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include "ControlModule/include/ActuatorRequests.h"
#ifndef V6_CROSS_BUILD
#include "MotionModule/include/MotionRequest.h"
#else
#include "ControlModule/include/LolaRequest.h"
#endif

#ifndef V6_CROSS_BUILD
#define JOINT_REQUEST_BASE MotionRequest
#define JOINT_REQUEST_BASE_IDS MotionRequestIds
#else
#ifndef REALTIME_LOLA_AVAILABLE
#define JOINT_REQUEST_BASE LolaRequest
#define JOINT_REQUEST_BASE_IDS LolaRequestIds
#endif
#endif

using namespace std;

/**
 * @class JointRequest
 * @brief Defines a basic joint actuation request
 */
struct JointRequest : public ActuatorRequest, public JOINT_REQUEST_BASE
{
  /**
   * Constructor
   */
  JointRequest() :
    ActuatorRequest(toUType(Joints::count)),
    JOINT_REQUEST_BASE(JOINT_REQUEST_BASE_IDS::jointRequest)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!JOINT_REQUEST_BASE::assignFromJson(obj))
      return false;
    try {
      FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),)
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Exception caught in JointRequest: ");
      LOG_EXCEPTION(e.what());
      return false;
    }
    return true;
  }

  /**
   * @brief getJson Makes a json object from request paramters
   * @return Json object
   */
  virtual Json::Value getJson() {
    Json::Value obj = JOINT_REQUEST_BASE::getJson();
    try {
      FOR_EACH(GET_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),);
    } catch (Json::Exception& e) {
     LOG_EXCEPTION(
       "Exception caught in module request ModuleRequest::JointRequest: ")
      LOG_EXCEPTION(e.what());
    }
    return obj;
  }
};
typedef boost::shared_ptr<JointRequest> JointRequestPtr;
