/**
 * @file SBModule/src/SBConfigs/SBConfig.cpp
 *
 * This file implements the struct SBConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "Utils/include/Behaviors/SBConfigs/SBConfig.h"
#include "Utils/include/Behaviors/SBConfigs/SBLedsConfig.h"
#include "Utils/include/Behaviors/SBConfigs/SBStiffnessConfig.h"
#include "Utils/include/Behaviors/SBConfigs/SBWDConfig.h"

/**
 * @brief makeFromJson Returns a child config of given type
 * @param obj Json object of configuration
 * @return SBConfigPtr
 */
boost::shared_ptr<SBConfig>
SBConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<SBConfig> config;
  try {
    if (!obj.isNull()) {
      switch (obj["id"].asUInt()) {
        case toUType(SBIds::ledsModule):
          config = SBLedsConfig::makeFromJson(obj); break;
        case toUType(SBIds::stiffnessModule):
          config = SBStiffnessConfig::makeFromJson(obj); break;
        case toUType(SBIds::whistleDetector):
          config = SBWDConfig::makeFromJson(obj); break;
      }
    }
  } catch (Json::Exception& e) {
    cout
      << "Exception caught making a SBConfig from json object;\t";
    LOG_EXCEPTION(e.what());
    config.reset();
  } catch (TNRSException& e) {
    LOG_EXCEPTION(e.what());
    config.reset();
  }
  return config;
}
