/**
 * @file TNRSModules/BehaviorConfigs/src/GBConfigs/GBConfig.cpp
 *
 * This file implements the struct GBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "BehaviorConfigs/include/GBConfigs/GBConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBLedsConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBStiffnessConfig.h"
#include "BehaviorConfigs/include/GBConfigs/GBWDConfig.h"

/**
 * @brief makeFromJson Returns a child config of given type
 * @param obj Json object of configuration
 * @return GBConfigPtr
 */
boost::shared_ptr<GBConfig>
GBConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<GBConfig> config;
  try {
    if (!obj.isNull()) {
      switch (obj["id"].asUInt()) {
        case toUType(GBIds::ledsModule):
          config = GBLedsConfig::makeFromJson(obj); break;
        case toUType(GBIds::stiffnessModule):
          config = GBStiffnessConfig::makeFromJson(obj); break;
        case toUType(GBIds::whistleDetector):
          config = GBWDConfig::makeFromJson(obj); break;
      }
    }
  } catch (Json::Exception& e) {
    cout
      << "Exception caught making a GBConfig from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}
