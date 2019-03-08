/**
 * @file MotionModule/src/MotionConfigs/MBPostureConfig.cpp
 *
 * This file implements the structs MBPostureConfig 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
 
#include "Utils/include/Behaviors/MBConfigs/MBPostureConfig.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/DataHolders/PostureState.h"
#include "Utils/include/HardwareIds.h"

MBPostureConfig::MBPostureConfig() :
  MBConfig(MBIds::posture, 10.f, (int)MBPostureTypes::interpToPosture),
  timeToReachP(-1.0f)
{
}

MBPostureConfig::MBPostureConfig(
  const VectorXf& jointsToReach,
  const float& timeToReachP,
  const MBPostureTypes type) :
  MBConfig(MBIds::posture, 10.f, (int)type),
  jointsToReach(jointsToReach),
  timeToReachP(timeToReachP),
  targetPosture(PostureState::unknown)
{
}


MBPostureConfig::MBPostureConfig(
  const PostureState& targetPosture,
  const float& timeToReachP,
  const MBPostureTypes type) :
  MBConfig(MBIds::posture, 10.f, (int)type),
  targetPosture(targetPosture),
  timeToReachP(timeToReachP)
{
  if (toUType(targetPosture) >= toUType(PostureState::staticPostures))
    return;
  jointsToReach = 
    VectorXf::Map(
      &postureDefinitions[toUType(targetPosture)][0],
      sizeof(postureDefinitions[toUType(targetPosture)]) /
      sizeof(postureDefinitions[toUType(targetPosture)][0])
    );
}

void MBPostureConfig::validate() 
{
  if (timeToReachP <= 0.f || // Undefined time given
      jointsToReach.size() !=
      static_cast<unsigned>(Joints::count))
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

boost::shared_ptr<MBPostureConfig> 
MBPostureConfig::makeFromJson(const Json::Value& obj)
{
  //LOG_INFO("MPPostureConfig::makeFromJson() called...")
  boost::shared_ptr<MBPostureConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == toUType(MBIds::posture) &&
        obj["baseType"].asUInt() == toUType(BaseBehaviorType::motion) &&
        obj["type"].asUInt() <= toUType(MBPostureTypes::count)
    ) {
      switch (obj["type"].asUInt()) {
        case toUType(MBPostureTypes::interpToPosture):
          config = 
            boost::make_shared<MBPostureConfig>(
              VectorXf(), 2.f, MBPostureTypes::interpToPosture); 
          break;
        
      }
      if (!config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a posture behavior config from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a posture behavior config from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

bool MBPostureConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("MPPostureConfig::assignFromJson() called...")
  if (!MBConfig::assignFromJson(obj))
    return false;
  try {
    int tp = obj.get("targetPosture", -1).asInt();
    timeToReachP = obj.get("timeToReachP", 0.f).asFloat();
    if (tp < 0) {
      const Json::Value& joints = obj["jointsToReach"];
      jointsToReach.resize(joints.size());
      for (int i = 0; i < joints.size(); ++i) {
        jointsToReach[i] = joints[i].asFloat();
      }
    } else {
      targetPosture = (PostureState) tp;
      if (toUType(targetPosture) <= toUType(PostureState::staticPostures)) {
        jointsToReach = 
          VectorXf::Map(
            &postureDefinitions[toUType(targetPosture)][0],
            sizeof(postureDefinitions[toUType(targetPosture)]) /
            sizeof(postureDefinitions[toUType(targetPosture)][0])
          );
      } else {
        return false;
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

Json::Value MBPostureConfig::getJson() 
{
  //LOG_INFO("MPPostureConfig::getJson() called...")
  Json::Value obj = BehaviorConfig::getJson();
  try {
    obj["targetPosture"] = toUType(targetPosture);
    obj["timeToReachP"] = timeToReachP;
    Json::Value joints;
    for (int i = 0; i < joints.size(); ++i) {
      joints[i] = jointsToReach[i];
    }
    obj["jointsToReach"] = joints;
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
