/**
 * @file MotionModule/src/MotionConfigs/MBBallThrowConfig.cpp
 *
 * This file implements the struct MBBallThrowConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"
#include "Utils/include/DataUtils.h"

MBBallThrowConfig::MBBallThrowConfig(
  const MBBallThrowTypes& type,
  const float& timeToThrow,
  const bool& headTapToStart) :
  MBConfig(MBIds::ballThrow, 45.f, (int)type),
  timeToThrow(timeToThrow),
  headTapToStart(headTapToStart)
{
}

void MBBallThrowConfig::validate() {
  if (timeToThrow <= 0.f) 
  { 
    throw 
      BConfigException(
        this, 
        "Invalid behavior configuration parameters passed.", 
        false, 
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

bool MBBallThrowConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBConfig::assignFromJson(obj))
    return false;
  try {
    timeToThrow = obj.get("timeToThrow", -1.f).asFloat(); // An invalid default value -1
    headTapToStart = obj.get("headTapToStart", false).asBool();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
    return false;
  }
  return true;
}

Json::Value MBBallThrowConfig::getJson() 
{ 
  Json::Value obj = MBConfig::getJson();
  try {
    obj["timeToThrow"] = timeToThrow;
    obj["headTapToStart"] = headTapToStart;
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
  }
  return obj;
}

boost::shared_ptr<MBBallThrowConfig> 
  MBBallThrowConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBBallThrowConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::ballThrow) &&
        obj["baseType"].asUInt() == toUType( BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBBallThrowTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBBallThrowTypes::wbBallThrow):
          config = 
            boost::make_shared<WBBallThrowConfig>(); break;
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a ball throw behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

void WBBallThrowConfig::validate() {}

