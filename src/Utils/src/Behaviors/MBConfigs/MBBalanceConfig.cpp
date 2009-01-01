/**
 * @file MotionModule/src/MotionConfigs/MBBalanceConfig.cpp
 *
 * This file defines the structs MBBalanceConfig, MPComControlConfig,
 * PIDComControlConfig, and ZmpControlConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"
#include "Utils/include/Behaviors/MBConfigs/MBBalanceConfig.h"
#include "Utils/include/HardwareIds.h"

MBBalanceConfig::MBBalanceConfig(
  const MBBalanceTypes& type,
  const LinkChains& supportLeg,
  const float& timeToReachB) :
  MBConfig(MBIds::balance, 45.f, (int)type), // Basically a lot of time
  supportLeg(supportLeg), timeToReachB(timeToReachB)
{
}
  
bool MBBalanceConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBConfig::assignFromJson(obj))
    return false;
  try {
    // Reset max runtime from json cfg
    supportLeg = static_cast<LinkChains>(obj.get("supportLeg", 0).asUInt());
    timeToReachB = obj.get("timeToReachB", 0).asFloat();
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

Json::Value MBBalanceConfig::getJson() { 
  Json::Value obj = MBConfig::getJson();
  try {
    obj["supportLeg"] = static_cast<unsigned>(supportLeg);
    obj["timeToReachB"] = timeToReachB;
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

MBBalanceConfigPtr MBBalanceConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBBalanceConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::balance) &&
        obj["baseType"].asUInt() == toUType( BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBBalanceTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBBalanceTypes::mpComControl):
          config = boost::make_shared<MPComControlConfig>(); break;
        case toUType(MBBalanceTypes::pidComControl):
          config = boost::make_shared<PIDComControlConfig>(); break;
        case toUType(MBBalanceTypes::zmpControl):
          config = boost::make_shared<ZmpControlConfig>(); break;
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object.");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a balance behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

MPComControlConfig::MPComControlConfig(
  const LinkChains& supportLeg,
  const float& timeToReachB) :
  MBBalanceConfig(MBBalanceTypes::mpComControl, supportLeg, timeToReachB)
{
}

void MPComControlConfig::validate()
{
  if (
    supportLeg != LinkChains::lLeg &&
    supportLeg != LinkChains::rLeg ||
    timeToReachB < 0.5)
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

bool MPComControlConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBBalanceConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MPComControlConfig::getJson() 
{ 
  Json::Value obj = MBBalanceConfig::getJson();
  return obj;
}

PIDComControlConfig::PIDComControlConfig(const LinkChains& supportLeg) :
  MBBalanceConfig(MBBalanceTypes::pidComControl, supportLeg)
{
}

void PIDComControlConfig::validate() 
{
  if (
    supportLeg != LinkChains::lLeg &&
    supportLeg != LinkChains::rLeg)
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

bool PIDComControlConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBBalanceConfig::assignFromJson(obj))
    return false;
  return true;
}


Json::Value PIDComControlConfig::getJson() 
{ 
  Json::Value obj = MBBalanceConfig::getJson();
  return obj;
}

ZmpControlConfig::ZmpControlConfig(
  const LinkChains& supportLeg,
  const bool& keepOtherLegContact,
  const bool& regularizePosture,
  const bool& keepTorsoUpright) :
  MBBalanceConfig(MBBalanceTypes::zmpControl, supportLeg, 1.0),
  keepOtherLegContact(keepOtherLegContact),
  regularizePosture(regularizePosture),
  keepTorsoUpright(keepTorsoUpright),
  targetX(0.f),
  targetY(0.f)
{
  // Use all 5 joints each of lower body (except hip yaw pitch) as default
  activeJoints = vector<bool> (toUType(Joints::count), false);
  for (int i = toUType(Joints::lHipRoll); i <= toUType(Joints::lAnkleRoll); ++i)
    activeJoints[i] = true;
  for (int i = toUType(Joints::rHipRoll); i <= toUType(Joints::rAnkleRoll); ++i)
    activeJoints[i] = true;
}

ZmpControlConfig::ZmpControlConfig(
  const LinkChains& supportLeg,
  const boost::shared_ptr<ZmpRefGenerator<float> >& refGenerator,
  const bool& keepOtherLegContact,
  const bool& regularizePosture,
  const bool& keepTorsoUpright) :
  MBBalanceConfig(MBBalanceTypes::zmpControl, supportLeg),
  refGenerator(refGenerator),
  keepOtherLegContact(keepOtherLegContact),
  regularizePosture(regularizePosture),
  keepTorsoUpright(keepTorsoUpright),
  targetX(0.f),
  targetY(0.f)
{
  // Use all 5 joints each of lower body (except hip yaw pitch) as default
  activeJoints = vector<bool> (toUType(Joints::count), false);
  for (int i = toUType(Joints::lHipRoll); i <= toUType(Joints::lAnkleRoll); ++i)
    activeJoints[i] = true;
  for (int i = toUType(Joints::rHipRoll); i <= toUType(Joints::rAnkleRoll); ++i)
    activeJoints[i] = true;
}

void ZmpControlConfig::validate() 
{
  return;//@TODO REMOVE LATER
  if (!refGenerator) 
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

bool ZmpControlConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBBalanceConfig::assignFromJson(obj))
    return false;
  try {
    // Reset max runtime from json cfg
    keepTorsoUpright = obj.get("keepTorsoUpright", 0).asBool();
    regularizePosture = obj.get("regularizePosture", 0).asBool();
    keepOtherLegContact = obj.get("keepOtherLegContact", 0).asBool();
    targetX = obj.get("targetX", 0.0).asFloat();
    targetY = obj.get("targetY", 0.0).asFloat();
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

Json::Value ZmpControlConfig::getJson() 
{ 
  Json::Value obj = MBBalanceConfig::getJson();
  try {
    obj["keepTorsoUpright"] = keepTorsoUpright;
    obj["regularizePosture"] = regularizePosture;
    obj["keepOtherLegContact"] = keepOtherLegContact;
    obj["targetX"] = targetX;
    obj["targetY"] = targetY;
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
