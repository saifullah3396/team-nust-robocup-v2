/**
 * @file MotionModule/include/HandsRequest.h
 *
 * This file defines the class HandsRequest
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
#define HANDS_REQUEST_BASE MotionRequest
#define HANDS_REQUEST_BASE_IDS MotionRequestIds
#else
#define HANDS_REQUEST_BASE LolaRequest
#define HANDS_REQUEST_BASE_IDS LolaRequestIds
#endif

using namespace std;

/**
 * @class HandsRequest
 * @brief Defines a basic joint actuation request
 */
struct HandsRequest : public ActuatorRequest, public HANDS_REQUEST_BASE
{
  /**
   * Constructor
   */
  HandsRequest() :
    ActuatorRequest(toUType(RobotHands::count)),
    HANDS_REQUEST_BASE(HANDS_REQUEST_BASE_IDS::handsRequest)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!HANDS_REQUEST_BASE::assignFromJson(obj))
      return false;
    try {
      FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),)
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Exception caught in HandsRequest: ");
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
    Json::Value obj = HANDS_REQUEST_BASE::getJson();
    try {
      FOR_EACH(GET_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),);
    } catch (Json::Exception& e) {
     LOG_EXCEPTION(
       "Exception caught in module request ModuleRequest::HandsRequest: ")
      LOG_EXCEPTION(e.what());
    }
    return obj;
  }
};
typedef boost::shared_ptr<HandsRequest> HandsRequestPtr;
