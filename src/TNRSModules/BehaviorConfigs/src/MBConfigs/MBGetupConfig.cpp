/**
 * @file MotionModule/src/MotionConfigs/MBGetupConfig.cpp
 *
 * This file implements the structs MBGetupConfig and KFMGetupConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/GetupModule/KeyFrameGetupTypes.h"
#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "Utils/include/DataUtils.h"

MBGetupConfig::MBGetupConfig(const MBGetupTypes& type) :
  MBConfig(MBIds::getup, 10.f, (int)type)
{
}

bool MBGetupConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBGetupConfig::getJson() { 
  Json::Value obj = MBConfig::getJson();
  return obj;
}
  
MBGetupConfigPtr MBGetupConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBGetupConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::getup) &&
        obj["baseType"].asUInt() == toUType(BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBGetupTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBGetupTypes::kfmGetup):
          config = 
            boost::make_shared<KFMGetupConfig>(); break;
        
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a getup behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
}

KFMGetupConfig::KFMGetupConfig(
  const KeyFrameGetupTypes& keyFrameGetupType) :
  MBGetupConfig(MBGetupTypes::kfmGetup),
  keyFrameGetupType(keyFrameGetupType)
{
}

void KFMGetupConfig::validate() {
}

bool KFMGetupConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBGetupConfig::assignFromJson(obj))
    return false;
  try {
    // Will throw if key doesn't exist
    keyFrameGetupType = 
      (KeyFrameGetupTypes) obj.get("keyFrameGetupType", -1).asUInt();
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
  
Json::Value KFMGetupConfig::getJson() { 
  Json::Value obj = MBGetupConfig::getJson();
  try {
    obj["keyFrameGetupType"] = toUType(keyFrameGetupType);
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

