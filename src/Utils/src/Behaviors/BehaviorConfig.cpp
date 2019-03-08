/**
 * @file BehaviorManager/src/BehaviorConfig.cpp
 *
 * This file implements the classes BehaviorConfig and BConfigException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/Behaviors/BehaviorConfig.h"
#include "Utils/include/Behaviors/MBConfigs/MBConfig.h"
#include "Utils/include/Behaviors/SBConfigs/SBConfig.h"
#include "Utils/include/Behaviors/PBConfigs/PBConfig.h"
#include "Utils/include/EnumUtils.h"

BConfigException::BConfigException(
  BehaviorConfig* config, 
  const string& message,
  const bool& bSysMsg, 
  const BConfigExceptionType& type) throw () :
  TNRSException(message, bSysMsg),
  config(config),
  type(type)
{
}

string BConfigException::getExcPrefix()
{ 
  return 
    string("Exception caught in behavior config \n\tType: ") + 
    DataUtils::varToString(static_cast<unsigned>(config->baseType)) +
    string(", Id: ") + 
    DataUtils::varToString(config->id) + 
    string(";\t"); 
}

boost::shared_ptr<BehaviorConfig>
  BehaviorConfig::makeFromJson(const Json::Value& obj)
{
  LOG_INFO("BehaviorConfig::makeFromJson() called...")
  boost::shared_ptr<BehaviorConfig> config;
  try {
    if (!obj.isNull()) {
      switch (obj["baseType"].asUInt()) {
        case toUType(BaseBehaviorType::motion):
          config = MBConfig::makeFromJson(obj); break;
        case toUType(BaseBehaviorType::sb):
          config = SBConfig::makeFromJson(obj); break;
        case toUType(BaseBehaviorType::planning):
          config = PBConfig::makeFromJson(obj); break;
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a behavior config from json object.";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

bool BehaviorConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("BehaviorConfig::assignFromJson() called")
  ASSIGN_CONFIG_FROM_JSON(
    (float, maxRuntime, maxRuntime),
    (bool, logData, logData),
  )
}

Json::Value BehaviorConfig::getJson() {
  //LOG_INFO("BehaviorConfig::getJson()")
  Json::Value obj;
  GET_BEHAVIOR_CONFIG_JSON(
    (int, id), 
    (int, type), 
    (unsigned, baseType), 
    (float, maxRuntime),
    (bool, logData),
  );
  return obj;
}
