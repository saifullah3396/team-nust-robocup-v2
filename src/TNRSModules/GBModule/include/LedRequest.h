/**
 * @file GBModule/include/LedRequest.h
 *
 * This file defines the class LedRequest
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
#define LED_REQUEST_BASE GBRequest
#define LED_REQUEST_BASE_IDS GBRequestIds
#else //! V6 handles realtime requests through Lola
#define LED_REQUEST_BASE LolaRequest
#define LED_REQUEST_BASE_IDS LolaRequestIds
#endif
/**
 * @struct LedRequest
 * @brief Defines a basic leds actuation request
 */
struct LedRequest : public ActuatorRequest, public LED_REQUEST_BASE
{
  /**
   * @brief LedRequest Constructor
   */
  LedRequest() :
    ActuatorRequest(toUType(LedActuators::count)),
    LED_REQUEST_BASE(LED_REQUEST_BASE_IDS::ledRequest)
  {
  }

  /**
   * @brief assignFromJson Assigns requesturation parameters from json
   * @param obj Json requesturation
   * @return true if successful
   */
  virtual bool assignFromJson(const Json::Value& obj) {
    if (!LED_REQUEST_BASE::assignFromJson(obj))
      return false;
    try {
      FOR_EACH(ASSIGN_FROM_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),)
    } catch (Json::Exception& e) {
      LOG_EXCEPTION("Exception caught in LedRequest: ");
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
    Json::Value obj = LED_REQUEST_BASE::getJson();
    try {
      FOR_EACH(GET_JSON_VAR_1, (vector<float>, value, vector<float>(size, NAN)),);
    } catch (Json::Exception& e) {
     LOG_EXCEPTION(
       "Exception caught in module request ModuleRequest::LedRequest: ")
      LOG_EXCEPTION(e.what());
    }
    return obj;
  }
};
typedef boost::shared_ptr<LedRequest> LedRequestPtr;
