/**
 * @file MotionModule/src/MotionConfigs/MBDiveConfig.cpp
 *
 * This file implements the structs MBDiveConfig and KFMDiveConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/Behaviors/MBConfigs/MBDiveConfig.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"

MBDiveConfig::MBDiveConfig(const MBDiveTypes& type) :
  MBConfig(MBIds::dive, 5.f, (int)type)
{
}

bool MBDiveConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBDiveConfig::getJson() { 
  Json::Value obj = MBConfig::getJson();
  return obj;
}

MBDiveConfigPtr MBDiveConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBDiveConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::dive) &&
        obj["baseType"].asUInt() == toUType(BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBDiveTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBDiveTypes::kfmDive):
          config = 
            boost::make_shared<KFMDiveConfig>(); break;
      case toUType(MBDiveTypes::handSaveDive):
        config =
          boost::make_shared<HandSaveDiveConfig>(); break;
        
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a dive behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  cout << "config: " << config << endl;
  return config;
}

KFMDiveConfig::KFMDiveConfig(
  const KeyFrameDiveTypes& keyFrameDiveType) :
  MBDiveConfig(MBDiveTypes::kfmDive),
  keyFrameDiveType(keyFrameDiveType)
{
}

void KFMDiveConfig::validate() 
{
}

bool KFMDiveConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBDiveConfig::assignFromJson(obj))
    return false;
  try {
    // Will throw if key doesn't exist
    keyFrameDiveType = 
      (KeyFrameDiveTypes) obj.get("keyFrameDiveType", -1).asUInt();
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

Json::Value KFMDiveConfig::getJson() 
{ 
  Json::Value obj = MBDiveConfig::getJson();
  try {
    obj["keyFrameDiveType"] = toUType(keyFrameDiveType);
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

void HandSaveDiveConfig::validate() {}

