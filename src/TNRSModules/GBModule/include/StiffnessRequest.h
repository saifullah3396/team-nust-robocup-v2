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
#ifndef V6_CROSS_BUILD
#include "GBModule/include/GBRequest.h"
#else
#include "ControlModule/include/LolaRequest.h"
#endif

#ifndef V6_CROSS_BUILD
#define STIFFNESS_REQUEST_BASE GBRequest
#define STIFFNESS_REQUEST_BASE_IDS GBRequestIds
#else //! V6 handles realtime requests through Lola
#define STIFFNESS_REQUEST_BASE LolaRequest
#define STIFFNESS_REQUEST_BASE_IDS LolaRequestIds
#endif

/**
 * @class StiffnessRequest
 * @brief Defines a basic stiffness actuator request
 */
struct StiffnessRequest : public ActuatorRequest, public STIFFNESS_REQUEST_BASE
{
  /**
   * @brief StiffnessRequest Constructor
   */
  StiffnessRequest() :
    ActuatorRequest(static_cast<unsigned>(Joints::count)),
    STIFFNESS_REQUEST_BASE(STIFFNESS_REQUEST_BASE_IDS::stiffnessRequest)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!STIFFNESS_REQUEST_BASE::assignFromJson(obj))
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
    Json::Value obj = STIFFNESS_REQUEST_BASE::getJson();
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
