/**
 * @file TNRSBase/src/ModuleRequest.cpp
 *
 * This file implements the struct ModuleRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/shared_ptr.hpp>
#include <json/json.h>
#include "PlanningModule/include/PlanningRequest.h"
#include "MotionModule/include/MotionRequest.h"
#include "GBModule/include/GBRequest.h"
#include "VisionModule/include/VisionRequest.h"
#include "LocalizationModule/include/LocalizationRequest.h"
#include "GameCommModule/include/GameCommRequest.h"
#include "UserCommModule/include/UserCommRequest.h"
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "TNRSBase/include/ModuleRequestMacros.h"
#include "Utils/include/PrintUtils.h"

ModuleRequest::ModuleRequest(
  const TNSPLModules& moduleId,
  const unsigned& requestId) :
  moduleId(moduleId),
  requestId(requestId)
{
}

boost::shared_ptr<ModuleRequest> ModuleRequest::makeFromJson(const Json::Value& obj)
{
  LOG_INFO("ModuleRequest::makeFromJson() called...");
  boost::shared_ptr<ModuleRequest> request;
  try {
    if (!obj.isNull()) {
      switch (obj["moduleId"].asUInt()) {
        case toUType(TNSPLModules::planning): request = PlanningRequest::makeFromJson(obj); break;
        case toUType(TNSPLModules::motion): request = MotionRequest::makeFromJson(obj); break;
        case toUType(TNSPLModules::gb): request = GBRequest::makeFromJson(obj); break;
        case toUType(TNSPLModules::vision): request = VisionRequest::makeFromJson(obj); break;
        case toUType(TNSPLModules::localization): request = LocalizationRequest::makeFromJson(obj); break;
        case toUType(TNSPLModules::gameComm): request = GameCommRequest::makeFromJson(obj); break;
        case toUType(TNSPLModules::userComm): request = UserCommRequest::makeFromJson(obj); break;
      }
    }
  } catch (Json::Exception& e) {
    LOG_EXCEPTION("Exception caught making a behavior config from json object." << e.what());
    request.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    request.reset();
  }
  return request;
}

bool ModuleRequest::assignFromJson(const Json::Value& obj)
{
  return true;
}

Json::Value ModuleRequest::getJson() {
  Json::Value obj;
  try {
    FOR_EACH(GET_JSON_VAR_1, (TNSPLModules, moduleId), (unsigned, requestId),)
  } catch (Json::Exception& e) {
   LOG_EXCEPTION(
     "Exception caught in ModuleRequest")
    LOG_EXCEPTION(e.what());
  }
  return obj;
}
