/**
 * @file MotionModule/src/MotionConfigs/MBHeadControlConfig.cpp
 *
 * This file implements the structs MBHeadControlConfig, 
 * HeadTargetSearchConfig and HeadTargetTrackConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/DataUtils.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/HeadControl/HeadTargetTypes.h"

MBHeadControlConfig::MBHeadControlConfig(
  const MBHeadControlTypes& type, 
  const float& maxRunTime) :
  MBConfig(MBIds::headControl, maxRunTime, (int)type)
{
}

bool MBHeadControlConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("MBHeadControlConfig::assignFromJson() called...")
  if (!MBConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBHeadControlConfig::getJson() { 
  //LOG_INFO("MBHeadControlConfig::getJson() called...")
  Json::Value obj = MBConfig::getJson();
  return obj;
}

MBHeadControlConfigPtr MBHeadControlConfig::makeFromJson(const Json::Value& obj)
{
  //LOG_INFO("MBHeadControlConfig::makeFromJson() called...")
  boost::shared_ptr<MBHeadControlConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::headControl) &&
        obj["baseType"].asUInt() == toUType(BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBHeadControlTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBHeadControlTypes::headTargetSearch):
          //LOG_INFO("Making a HeadTargetSearchConfig...")
          config = 
            boost::make_shared<HeadTargetSearchConfig>(); break;
        case toUType(MBHeadControlTypes::headTargetTrack):
          //LOG_INFO("Making a HeadTargetTrackConfig...")
          config = 
            boost::make_shared<HeadTargetTrackConfig>(); break;
        case toUType(MBHeadControlTypes::headScan):
          //LOG_INFO("Making a HeadScanConfig...")
          config =
            boost::make_shared<HeadScanConfig>(); break;
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a posture behavior config from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a head control behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}
  
HeadTargetSearchConfig::HeadTargetSearchConfig(
  const HeadTargetTypes& headTargetType,
  const bool& scanLowerArea,
  const float& totalWaitTime,
  const float& scanMaxYaw,
  const float& scanMaxPitch) :
  MBHeadControlConfig(MBHeadControlTypes::headTargetSearch),
  headTargetType(headTargetType),
  totalWaitTime(totalWaitTime),
  scanMaxYaw(scanMaxYaw),
  scanMaxPitch(scanMaxPitch),
  scanLowerArea(scanLowerArea)
{
}

void HeadTargetSearchConfig::validate() 
{
}


bool HeadTargetSearchConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("HeadTargetSearchConfig::assignFromJson() called...")
  if (!MBHeadControlConfig::assignFromJson(obj))
    return false;
  try {
    scanLowerArea = obj.get("scanLowerArea", false).asBool();
    scanMaxYaw = obj.get("scanMaxYaw", 1.f).asFloat();
    scanMaxPitch = obj.get("scanMaxPitch", 100.f * M_PI / 180).asFloat();
    totalWaitTime = obj.get("totalWaitTime", 16.f * M_PI / 180).asFloat();
    headTargetType = (HeadTargetTypes) obj.get("headTargetType", -1).asUInt();
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

Json::Value HeadTargetSearchConfig::getJson() 
{
  //LOG_INFO("HeadTargetSearchConfig::getJson() called...")
  Json::Value obj = MBHeadControlConfig::getJson();
  try {
    obj["scanLowerArea"] = scanLowerArea;
    obj["scanMaxYaw"] = scanMaxYaw;
    obj["scanMaxPitch"] = scanMaxPitch;
    obj["totalWaitTime"] = totalWaitTime;
    obj["headTargetType"] = toUType(headTargetType);
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

HeadTargetTrackConfig::HeadTargetTrackConfig(
  const HeadTargetTypes& headTargetType,
  const float& maxRunTime) : // Basically stay non stop in track
  MBHeadControlConfig(
    MBHeadControlTypes::headTargetTrack, maxRunTime),
  headTargetType(headTargetType)
{
}

void HeadTargetTrackConfig::validate() 
{
}

bool HeadTargetTrackConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("HeadTargetTrackConfig::assignFromJson() called...")
  if (!MBHeadControlConfig::assignFromJson(obj))
    return false;
  try {
    headTargetType = 
      (HeadTargetTypes) obj.get("headTargetType", -1).asUInt();
    const Json::Value& htsConfigObj = obj["htsConfig"];
    if (!htsConfigObj.isNull()) {
      auto htsConfig = BehaviorConfig::makeFromJson(htsConfigObj);
      if(htsConfig->assignFromJson(htsConfigObj)) {
        this->htsConfig = boost::static_pointer_cast<HeadTargetSearchConfig>(htsConfig);
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << static_cast<unsigned>(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    LOG_EXCEPTION(e.what());
  }
  return true;
}

Json::Value HeadTargetTrackConfig::getJson() 
{
  //LOG_INFO("HeadTargetTrackConfig::getJson() called...")
  Json::Value obj = MBHeadControlConfig::getJson();
  try {
    obj["headTargetType"] = toUType(headTargetType);
    if (htsConfig)
      obj["htsConfig"] = htsConfig->getJson();
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

HeadScanConfig::HeadScanConfig(
  const bool& scanLowerArea,
  const float& totalWaitTime,
  const float& scanMaxYaw,
  const float& scanMaxPitch) :
  MBHeadControlConfig(MBHeadControlTypes::headScan),
  totalWaitTime(totalWaitTime),
  scanMaxYaw(scanMaxYaw),
  scanMaxPitch(scanMaxPitch),
  scanLowerArea(scanLowerArea)
{
}

void HeadScanConfig::validate()
{
}


bool HeadScanConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("HeadTargetSearchConfig::assignFromJson() called...")
  if (!MBHeadControlConfig::assignFromJson(obj))
    return false;
  try {
    scanLowerArea = obj.get("scanLowerArea", false).asBool();
    scanMaxYaw = obj.get("scanMaxYaw", 1.f).asFloat();
    scanMaxPitch = obj.get("scanMaxPitch", 100.f * M_PI / 180).asFloat();
    totalWaitTime = obj.get("totalWaitTime", 16.f * M_PI / 180).asFloat();
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

Json::Value HeadScanConfig::getJson()
{
  //LOG_INFO("HeadScanConfig::getJson() called...")
  Json::Value obj = MBHeadControlConfig::getJson();
  try {
    obj["scanLowerArea"] = scanLowerArea;
    obj["scanMaxYaw"] = scanMaxYaw;
    obj["scanMaxPitch"] = scanMaxPitch;
    obj["totalWaitTime"] = totalWaitTime;
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

