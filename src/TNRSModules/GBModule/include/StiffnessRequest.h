/**
 * @file GBModule/include/StiffnessRequest.h
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

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!GBRequest::assignFromJson(obj))
      return false;
    try {
      FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),)
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Exception caught in StiffnessRequest: ");
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
    Json::Value obj = GBRequest::getJson();
    try {
      FOR_EACH(GET_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),);
    } catch (Json::Exception& e) {
     LOG_EXCEPTION(
       "Exception caught in module request ModuleRequest::StiffnessRequest: ")
      LOG_EXCEPTION(e.what());
    }
    return obj;
  }
};
typedef boost::shared_ptr<StiffnessRequest> StiffnessRequestPtr;
