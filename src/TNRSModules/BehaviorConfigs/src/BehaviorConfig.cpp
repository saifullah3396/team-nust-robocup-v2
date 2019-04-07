/**
 * @file TNRSModules/BehaviorConfigs/src/BehaviorConfig.cpp
 *
 * This file implements the classes BehaviorConfig and BConfigException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#include "Utils/include/DataUtils.h"
#include "BehaviorConfigs/include/BehaviorConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"
#include "BehaviorConfigs/include/PBConfigs/PBConfig.h"
#include "Utils/include/EnumUtils.h"

BConfigException::BConfigException(
  BehaviorConfig* config,
  const string& message,
  const bool& bSysMsg) throw () :
  TNRSException(message, bSysMsg),
  config(config)
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
        case toUType(BaseBehaviorType::general):
          config = GBConfig::makeFromJson(obj); break;
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
  ASSIGN_CONFIG_FROM_JSON(BehaviorConfig,
    (bool, logData, logData),
    (float, maxRuntime, maxRuntime),
  )
  return true;
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
