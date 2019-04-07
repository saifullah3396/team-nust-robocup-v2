/**
 * @file TNRSModules/BehaviorConfigs/src/MBConfigs/MBConfig.cpp
 *
 * This file implements the struct MBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/DataUtils.h"
#include "BehaviorConfigs/include/MBConfigs/MBBalanceConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBBallThrowConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBDiveConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBGetupConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBHeadControlConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBKickConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMovementConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBMotionPlaybackConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBPostureConfig.h"
#include "BehaviorConfigs/include/MBConfigs/MBTeleopConfig.h"

MBConfig::MBConfig(
  const MBIds& id,
  const float& maxRunTime,
  const int& childType) :
BehaviorConfig(toUType(id), BaseBehaviorType::motion, maxRunTime, childType)
{
}

boost::shared_ptr<MBConfig>
MBConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBConfig> config;
  try {
    if (!obj.isNull()) {
      switch (obj["id"].asUInt()) {
        case toUType(MBIds::posture):
          config = MBPostureConfig::makeFromJson(obj); break;
        case toUType(MBIds::kick):
          config = MBKickConfig::makeFromJson(obj); break;
        case toUType(MBIds::ballThrow):
          config = MBBallThrowConfig::makeFromJson(obj); break;
        case toUType(MBIds::dive):
          config = MBDiveConfig::makeFromJson(obj); break;
        case toUType(MBIds::getup):
          config = MBGetupConfig::makeFromJson(obj); break;
        case toUType(MBIds::balance):
          config = MBBalanceConfig::makeFromJson(obj); break;
        case toUType(MBIds::headControl):
          config = MBHeadControlConfig::makeFromJson(obj); break;
        case toUType(MBIds::movement):
          config = MBMovementConfig::makeFromJson(obj); break;
        case toUType(MBIds::motionPlayback):
          config = MBMotionPlaybackConfig::makeFromJson(obj); break;
      case toUType(MBIds::teleop):
        config = MBTeleopConfig::makeFromJson(obj); break;
      }
    }
  } catch (Json::Exception& e) {
    cout
      << "Error creating a MBConfig from given Json object";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}

bool MBConfig::assignFromJson(const Json::Value& obj)
{
  //LOG_INFO("MBConfig::assignFromJson() called...")
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  return true;
}

/**
 * @brief getJson Makes a json object from config paramters
 * @return Json object
 */
Json::Value MBConfig::getJson() {
  return BehaviorConfig::getJson();
}
