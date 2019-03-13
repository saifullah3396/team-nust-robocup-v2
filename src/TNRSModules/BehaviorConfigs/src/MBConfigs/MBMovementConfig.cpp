/**
 * @file MotionModule/src/MotionConfigs/MBMovementConfig.cpp
 *
 * This file implements the structs MBMovementConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "Utils/include/DataHolders/RobotPose2D.h"
#include "Utils/include/PathPlanner/PathPlanner.h"

using namespace PathPlannerSpace;

MBMovementConfig::MBMovementConfig(
  const MBMovementTypes& type) :
  MBConfig(MBIds::movement, 3600.f, (int)type)
{
}

bool MBMovementConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBMovementConfig::getJson()
{
  Json::Value obj = MBConfig::getJson();
  return obj;
}

MBMovementConfigPtr MBMovementConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBMovementConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::movement) &&
        obj["baseType"].asUInt() == toUType(BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBMovementTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBMovementTypes::naoqiFootsteps):
          config =
            boost::make_shared<NaoqiFootstepsConfig>(); break;
        case toUType(MBMovementTypes::naoqiMoveToward):
          config =
            boost::make_shared<NaoqiMoveTowardConfig>(); break;
        case toUType(MBMovementTypes::speedWalk):
          config =
            boost::make_shared<SpeedWalkConfig>(); break;
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout
      << "Exception caught making a kick behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

NaoqiFootstepsConfig::NaoqiFootstepsConfig(
  const vector<PathPlannerSpace::State>& plannedPath) :
  MBMovementConfig(MBMovementTypes::naoqiFootsteps), plannedPath(plannedPath)
{
}

void NaoqiFootstepsConfig::validate()
{

}

bool NaoqiFootstepsConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBMovementConfig::assignFromJson(obj))
    return false;
  try {
    const Json::Value& pcfgObj = obj["startPosture"];
    if (!pcfgObj.isNull()) {
      auto pcfg = MBPostureConfig::makeFromJson(pcfgObj);
      if(pcfg->assignFromJson(pcfgObj)) {
        startPosture = pcfg;
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
    return false;
  }
  return true;
}

Json::Value NaoqiFootstepsConfig::getJson()
{
  Json::Value obj = MBMovementConfig::getJson();
  try {
    if (startPosture)
      obj["startPosture"] = startPosture->getJson();
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

NaoqiMoveTowardConfig::NaoqiMoveTowardConfig(
  const VelocityInput<float>& velocityInput) :
  MBMovementConfig(MBMovementTypes::naoqiMoveToward), velocityInput(velocityInput)
{
}

void NaoqiMoveTowardConfig::validate()
{

}

bool NaoqiMoveTowardConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBMovementConfig::assignFromJson(obj))
    return false;
  try {
    const Json::Value& pcfgObj = obj["startPosture"];
    if (!pcfgObj.isNull()) {
      auto pcfg = MBPostureConfig::makeFromJson(pcfgObj);
      if(pcfg->assignFromJson(pcfgObj)) {
        startPosture = pcfg;
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
    return false;
  }
  return true;
}

Json::Value NaoqiMoveTowardConfig::getJson()
{
  Json::Value obj = MBMovementConfig::getJson();
  try {
    if (startPosture)
      obj["startPosture"] = startPosture->getJson();
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

void SpeedWalkConfig::validate() {}
